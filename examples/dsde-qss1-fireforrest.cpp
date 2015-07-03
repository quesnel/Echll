/*
 * Copyright (C) 2015 INRA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <iterator>
#include <boost/progress.hpp>
#include <vle/dsde/qss1.hpp>
#include <vle/vle.hpp>

using state_type = std::array <double, 5>;

enum FirePosition {
    NORTH = 1, SOUTH, EAST, WEST
};

struct NoStorage {
    using input_type = vle::dsde::qss1::inputport <5>;
    using output_type = vle::dsde::qss1::doubleport;

    inline void push(std::size_t, double, const input_type&) {}
    inline void push(std::size_t, double, const output_type&) {}
    inline void resize(std::size_t) {}
};

struct Storage {
    using input_type = vle::dsde::qss1::inputport <5>;
    using output_type = vle::dsde::qss1::doubleport;

    using input_data = std::vector <input_type>;
    using output_data = std::vector <output_type>;

    struct Event {
        Event(const input_type &input)
        {
            in.emplace_back(input);
        }

        Event(const output_type &output)
        {
            out.emplace_back(output);
        }

        input_data in;
        output_data out;
    };

    ~Storage()
    {
        try {
            std::cout << "Storage in progress:\n" << std::flush;

            for (auto i = 0ul, e = vector.size(); i != e; ++i) {
                std::string filename(vle::stringf("model-%lu.dat", i));
                std::ofstream ofs(filename);
                std::cout << "\rWrite: " << filename << std::flush;

                for (const auto &event : vector[i]) {
                    ofs << event.first << ' ' << event.second.in.size() << ' ';

                    for (const auto &in : event.second.in)
                        ofs << in[0] << ' ' << in[1] << ' ' << in[2] << ' ' << in[3] << ' ' <<
                            in[4] << ' ';

                    ofs << event.second.out.size() << " ";

                    for (const auto &out : event.second.out)
                        ofs << out[0] << ' ';
                }

                ofs << '\n';
            }
        } catch (...) {
            std::cerr << "failure\n";
        }
    }

    void push(std::size_t id, double time, const input_type &data)
    {
        auto ret = vector.at(id).emplace(time, data);

        if (ret.second == false)
            ret.first->second.in.emplace_back(data);
    }

    void push(std::size_t id, double time, const output_type &data)
    {
        auto ret = vector.at(id).emplace(time, data);

        if (ret.second == false)
            ret.first->second.out.emplace_back(data);
    }

    void resize(std::size_t size)
    {
        vector.resize(size);
    }

    std::vector <std::map <double, Event>> vector;
};

class Fire
{
public:
    Fire(double alpha, double K, double k, double enthalpy,
         double mass, double surrounding, double dx, double dy)
        : coeff_n(K / (dy * dy))
        , coeff_s(K / (dy * dy))
        , coeff_e(K / (dx * dx))
        , coeff_w(K / (dx * dx))
        , coeff_c(- k - 2.0 * K * (1.0 / (dx * dx) + 1.0 / (dy * dy)))
        , coeff_cste(k * surrounding)
        , alpha(alpha)
        , f_t0(enthalpy * mass * alpha)
        , t0(0.0)
        , fire(false)
    {}

    double operator()(const state_type &variables, const double t)
    {
        if (!fire && variables[0] > 400.0) {
            fire = true;
            t0 = t;
        }

        if (fire && variables[0] < 333.0) {
            fire = false;
        }

        double ret = coeff_n * ((variables[NORTH] > 0.) ?
                                variables[NORTH] : variables[0]) +
                     coeff_s * ((variables[SOUTH] > 0.) ?
                                variables[SOUTH] : variables[0]) +
                     coeff_e * ((variables[EAST] > 0.) ?
                                variables[EAST] : variables[0]) +
                     coeff_w * ((variables[WEST] > 0.) ?
                                variables[WEST] : variables[0]) +
                     coeff_c * variables[0] +
                     coeff_cste;

        if (fire) {
            double f_t = f_t0 * std::exp(alpha * (t0 - t));
            f_t0 = f_t;
            t0 = t;
            ret += f_t;
        }

        return ret;
    }

private:
    const double coeff_n, coeff_s, coeff_e, coeff_w, coeff_c, coeff_cste;
    const double alpha;
    double f_t0, t0;
    bool fire;
};

template <typename StoragePolicy>
class FireForrest : public vle::dsde::CoupledModel <
    vle::DoubleTime, vle::PortList <double>, vle::PortList <double>,
    vle::dsde::qss1::inputport <5>,
    vle::dsde::qss1::doubleport,
    vle::dsde::TransitionPolicyDefault <vle::DoubleTime >>
{
public:
    using parent_type = vle::dsde::CoupledModel <
                        vle::DoubleTime, vle::PortList <double>, vle::PortList <double>,
                        vle::dsde::qss1::inputport <5>,
                        vle::dsde::qss1::doubleport,
                        vle::dsde::TransitionPolicyDefault <vle::DoubleTime >>;
    using children_t = parent_type::children_t;
    using child_type = parent_type::child_type;

    FireForrest(const vle::Context &ctx,
                std::size_t width, std::size_t height,
                double alpha, double K, double k, double enthalpy,
                double mass, double surrounding, double dx, double dy,
                double dq, double epsilon)
        : parent_type(ctx)
        , m_width(width)
        , m_height(height)
    {
        m_storage.resize(m_width * m_height);
        m_models.reserve(width * height);

        for (auto y = 0ul; y != height; ++y) {
            for (auto x = 0ul; x != width; ++x) {
                double init = (x == width / 4 && y == height / 4) ? 600 : 300;
                double localmass = (x > width / 2 && y > height / 2) ? mass /
                                   10 : mass;
                m_models.emplace_back(
                    ctx, dq, epsilon, init, 0u,
                    Fire(alpha, K, k, enthalpy, localmass, surrounding, dx, dy));
            }
        }
    }

    FireForrest(const vle::Context &ctx,
                std::ifstream &ifs,
                double alpha, double K, double k, double enthalpy,
                double mass, double surrounding, double dx, double dy,
                double dq, double epsilon)
        : parent_type(ctx)
        , m_time(0)
    {
        ifs.exceptions(std::ios_base::failbit | std::ios_base::badbit);
        ifs >> m_width >> m_height;

        if (m_width == 0 or m_height == 0)
            return;

        m_storage.resize(m_width * m_height);
        m_models.reserve(m_width * m_height);
        std::vector <int> type(m_width * m_height, 0);
        std::vector <double> init(m_width * m_height, 0);
        std::copy_n(std::istream_iterator <int>(ifs),
                    m_width * m_height,
                    type.begin());
        std::copy_n(std::istream_iterator <double>(ifs),
                    m_width * m_height,
                    init.begin());
        std::cout << m_width << "x" << m_height << "→"
                  << m_height *m_width << '\n';

        for (auto i = 0ul; i != (m_height * m_width); ++i) {
            switch (type[i]) {
            case 0:
            case 1:
                std::cout << ((init[i] <= 300) ? ": " : "⚡ ");
                m_models.emplace_back(
                    ctx, dq, epsilon, init[i], 0u,
                    Fire(alpha, K, k, enthalpy, mass, surrounding, dx, dy));
                break;

            case 2:
                std::cout << ((init[i] <= 300) ? ". " : "⚡ ");
                m_models.emplace_back(
                    ctx, dq, epsilon, init[i], 0u,
                    Fire(alpha, K, k, enthalpy, mass / 4.0, surrounding, dx, dy));
                break;
            }

            if (((1 + i) % m_width) == 0)
                std::cout << '\n';
        }
    }

    virtual children_t children(const vle::Common &) override final
    {
        children_t ret(m_models.size(), nullptr);

        for (auto i = 0ul, e = m_models.size(); i != e; ++i)
            ret[i] = &m_models[i];

        return ret;
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const override final
    {
        for (auto *mdl : out) {
            auto tmp = static_cast <const decltype(&m_models[0])>(mdl) - &m_models[0];
            std::size_t position = boost::numeric_cast<std::size_t>(tmp);
            auto x = position % m_width;
            auto y = position / m_width;
            m_storage.push(position, CoupledModel::tn, mdl->y);

            if (x > 0) {
                m_models[y * m_width + (x - 1)].x[EAST] = mdl->y[0];
                in.emplace(&m_models[y * m_width + (x - 1)]);
            }

            if (x + 1 < m_width) {
                m_models[y * m_width + (x + 1)].x[WEST] = mdl->y[0];
                in.emplace(&m_models[y * m_width + (x + 1)]);
            }

            if (y > 0) {
                m_models[(y - 1) * m_width + x].x[NORTH] = mdl->y[0];
                in.emplace(&m_models[(y - 1) * m_width + x]);
            }

            if (y + 1 < m_height) {
                m_models[(y + 1) * m_width + x].x[SOUTH] = mdl->y[0];
                in.emplace(&m_models[(y + 1) * m_width + x]);
            }
        }

        for (auto *mdl : in)
            m_storage.push(
                boost::numeric_cast<std::size_t>(
                    static_cast <const decltype(&m_models[0])>(mdl) - &m_models[0]),
                CoupledModel::tn,
                mdl->x);
    }

    void value(std::ostream &os)
    {
        for (auto i = 0ul, e = m_models.size(); i != e; ++i) {
            os << m_models[i].value();

            if ((i + 1) % m_width == 0)
                os << "\n";
            else
                os << " ";
        }

        os << "\n";
    }

    std::vector <vle::dsde::qss1::Equation<vle::DoubleTime, 5>> m_models;
    std::size_t m_width;
    std::size_t m_height;
    double m_time;
    mutable StoragePolicy m_storage;
};

template <typename StoragePolicy>
void run(FireForrest <StoragePolicy> &model, vle::Context &ctx)
{
    constexpr const double finish = 200.0;
    constexpr const unsigned int tostore = 40u;
    vle::dsde::Engine <vle::DoubleTime> dsde_engine;
    vle::SimulationStep <vle::dsde::Engine <vle::DoubleTime>> sim(ctx, dsde_engine);
    double current = sim.init(model, 0.0);
    double before = current;
    auto step = 0u;
    {
        boost::progress_timer timer;
        boost::progress_display progress(tostore);

        while (sim.step(model, current, finish)) {
            if ((current - before) >= (finish / tostore)) {
                std::ofstream ofs(vle::stringf("fire-%u.dat", step));
                model.value(ofs);
                before = current;
                ++step;
                ++progress;
            }
        }

        progress += (progress.expected_count() - progress.count());
    }
    std::ofstream ofs("fireforrest.gnuplot");
    ofs << "set terminal png nocrop enhanced size 4000,3000 font \"arial,9\"\n"
        << "set output 'fireforrest.png'\n"
        << "set multiplot layout "
        << 4u + (((step % 4u) > 0) ? 1u : 0u) << ','
        << step / 4u << " rowsfirst\n"
        << "set size ratio 1\n"
        << "set palette model XYZ rgbformulae 7,5,15\n"
        << "set xrange[-0.5:" << model.m_width << ".5]\n"
        << "set yrange[-0.5:" << model.m_height << ".5]\n"
        << "set cbrange[150.0:800.0]\n"
        << "set xtics 0,2," << model.m_width << "\n"
        << "set ytics 0,2," << model.m_height << "\n"
        << "set xtics offset -0.5,0\n";

    for (auto i = 0ul; i < step; ++i)
        ofs << "set title \"fire-" << i << ".dat" << "\"\n"
            << "plot \"fire-" << i << ".dat\" matrix w image noti\n";

    ofs << "unset multiplot\n";
}

int main(int argc, char **argv)
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    constexpr const std::size_t width = 30u;
    constexpr const std::size_t height = 30u;
    constexpr const double dq = 0.5;
    constexpr const double epsilon = 0.5;
    constexpr const double alpha = 0.19;
    constexpr const double K = 0.000031;
    constexpr const double k = 0.071;
    constexpr const double enthalpy = 3605;
    constexpr const double mass = 2;
    constexpr const double surrounding = 300;
    constexpr const double dx = 1. / 20.;
    constexpr const double dy = 1. / 20.;
    bool withstorage = false;
    std::vector <char *> files;
    files.reserve(4);

    for (int i = 1; i < argc; ++i)
        if (std::strcmp(argv[i], "-s") == 0)
            withstorage = true;
        else
            files.push_back(argv[i]);

    if (files.empty()) {
        std::cout << "Default simulation\n";
        FireForrest <NoStorage> model(ctx, width, height,
                                      alpha, K, k, enthalpy, mass,
                                      surrounding,  dx,  dy, dq, epsilon);
        run(model, ctx);
    } else {
        for (auto *str : files) {
            std::cout << str << " simulation\n";
            std::ifstream ifs(str);

            if (ifs.is_open()) {
                if (withstorage) {
                    FireForrest <Storage> model(ctx, ifs, alpha, K, k,
                                                enthalpy, mass, surrounding,
                                                dx,  dy, dq, epsilon);
                    run(model, ctx);
                } else {
                    FireForrest <NoStorage> model(ctx, ifs, alpha, K, k,
                                                  enthalpy, mass, surrounding,
                                                  dx,  dy, dq, epsilon);
                    run(model, ctx);
                }
            }
        }
    }

    return 0;
}
