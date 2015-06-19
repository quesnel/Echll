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

#include <boost/numeric/odeint.hpp>
#include <boost/progress.hpp>
#include <vle/dsde/qss1.hpp>
#include <vle/vle.hpp>

using state_type = std::vector <double>;

enum FirePosition
{
    NORTH = 1, SOUTH, EAST, WEST
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

class FireForrest : public vle::dsde::CoupledModel <
    vle::DoubleTime, vle::PortList <double>, vle::PortList <double>,
    vle::dsde::qss::QssInputPort,
    vle::dsde::qss::QssOutputPort,
    vle::dsde::TransitionPolicyThread <vle::DoubleTime>>
{
public:
    using parent_type = vle::dsde::CoupledModel <
                        vle::DoubleTime, vle::PortList <double>, vle::PortList <double>, vle::dsde::qss::QssInputPort,
                        vle::dsde::qss::QssOutputPort,
                        vle::dsde::TransitionPolicyThread <vle::DoubleTime>>;
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
        m_models.reserve(width * height);

        for (auto y = 0ul; y != height; ++y) {
            for (auto x = 0ul; x != width; ++x) {
                double init = (x == width / 2 && y == height / 2) ? 600 : 300;
                m_models.emplace_back(
                    ctx, dq, epsilon, init, 5u, 0u,
                    Fire(alpha, K, k, enthalpy, mass, surrounding, dx, dy));
            }
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

    std::vector <vle::dsde::qss::Equation<vle::DoubleTime, state_type>> m_models;
    std::size_t m_width;
    std::size_t m_height;
};

int main()
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    constexpr const std::size_t width = 40u;
    constexpr const std::size_t height = 40u;
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
    constexpr const double finish = 50.0;
    constexpr const unsigned int tostore = 20u;
    vle::dsde::Engine <vle::DoubleTime> dsde_engine;

    FireForrest model(ctx, width, height, alpha, K, k, enthalpy,
                      mass, surrounding,  dx,  dy,
                      dq, epsilon);
    vle::SimulationStep <vle::dsde::Engine <vle::DoubleTime>> sim(ctx, dsde_engine);
    double current = sim.init(model, 0.0);
    double before = current;
    auto step = 0u;

    {
        std::cout << "main/qss1/fire\n";

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
        << "set xrange[-0.5:" << width << ".5]\n"
        << "set yrange[-0.5:" << height << ".5]\n"
        << "set cbrange[150.0:800.0]\n"
        << "set xtics 0,2," << width << "\n"
        << "set ytics 0,2," << height << "\n"
        << "set xtics offset -0.5,0\n";

    for (auto i = 0ul; i < step; ++i)
        ofs << "set title \"fire-" << i << ".dat" << "\"\n"
            << "plot \"fire-" << i << ".dat\" matrix w image noti\n";

    ofs << "unset multiplot\n";

    return 0;
}
