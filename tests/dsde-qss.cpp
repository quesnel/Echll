/*
 * Copyright (C) 2013-2014 INRA
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
#include <vle/dsde/qss1.hpp>
#include <vle/vle.hpp>
#include <chrono>

#define CATCH_CONFIG_MAIN
#include "catch.hpp"

using MyTime = vle::DoubleTime;
using MyValue = double;
using MyDSDE = vle::dsde::Engine <MyTime, MyValue>;
using UpdatedPort = vle::dsde::UpdatedPort <MyTime, MyValue>;
using AtomicModel = vle::dsde::AtomicModel <MyTime, MyValue>;
using CoupledModelThread = vle::dsde::CoupledModel <MyTime, MyValue,
      vle::dsde::TransitionPolicyThread <MyTime, MyValue>>;
using CoupledModelMono = vle::dsde::CoupledModel <MyTime, MyValue,
      vle::dsde::TransitionPolicyDefault <MyTime, MyValue>>;

using state_type = std::vector <double>;
constexpr double X_init = 1.0;
constexpr double Y_init = 1.0;
constexpr double Alpha = 5.2;
constexpr double Beta = 3.4;
constexpr double Gamma = 2.1;
constexpr double Delta = 1.4;
constexpr double Finish = 10.0;

//
// helper class.
//

class ShowDuration
{
    std::chrono::time_point<std::chrono::steady_clock> m_begin;
    std::string m_experience;

public:
    ShowDuration(const std::string &str)
        : m_begin(std::chrono::steady_clock::now())
        , m_experience(str)
    {}

    ~ShowDuration()
    {
        try {
            auto end = std::chrono::steady_clock::now();
            auto diff = end - m_begin;
            std::cout << m_experience << ": "
                      << std::chrono::duration <double, std::milli> (diff).count()
                      << " ms\n";
        } catch (...) {
        }
    }
};

//
// function defintion.
//

struct prey_fct {
    double m_a, m_b;

    prey_fct(const double a = Alpha, const double b = Beta)
        : m_a(a)
        , m_b(b)
    {}

    double operator()(const state_type &x, const double) const
    {
        return m_a * x[0] - m_b * x[0] * x[1];
    }
};

struct predator_fct {
    double m_c, m_d;

    predator_fct(const double c = Gamma, const double d = Delta)
        : m_c(c)
        , m_d(d)
    {}

    double operator()(const state_type &x, const double) const
    {
        return - m_c * x[1] + m_d * x[0] * x[1];
    }
};

struct prey_predator_fct {
    double m_a, m_b, m_c, m_d;

    prey_predator_fct(const double a = Alpha, const double b = Beta,
                      const double c = Gamma, const double d = Delta)
        : m_a(a)
        , m_b(b)
        , m_c(c)
        , m_d(d)
    {}

    void operator()(const state_type &x,
                    state_type &dxdt,
                    const double) const
    {
        dxdt[0] = m_a * x[0] - m_b * x[0] * x[1];
        dxdt[1] = - m_c * x[1] + m_d * x[0] * x[1];
    }
};

struct harm_osc {
    double m_gam;

    harm_osc(double gam)
        : m_gam(gam)
    {}

    void operator()(const state_type &x, state_type &dxdt, const double)
    {
        dxdt[0] = x[1];
        dxdt[1] = -x[0] - m_gam * x[1];
    }
};

class SingleEvent : public AtomicModel
{
public:
    SingleEvent(const vle::Context &ctx)
        : AtomicModel(ctx, 0, 1)
    {}

    virtual double init(const vle::Common &, const double &) override final
    {
        return 57;
    }

    virtual double delta(const double &, const double &,
                         const double &) override final
    {
        return MyTime::infinity();
    }

    virtual void lambda() const override final
    {
        y[0] = { .37 };
    }
};

struct MySystem1 : CoupledModelMono {
    vle::dsde::qss::EquationBlock <MyTime, state_type> prey;
    vle::dsde::qss::EquationBlock <MyTime, state_type> predator;

    MySystem1(const vle::Context &ctx, double dq, double epsilon)
        : CoupledModelMono(ctx)
        , prey(ctx, dq, epsilon, X_init, 2u, 0u, prey_fct())
        , predator(ctx, dq, epsilon, Y_init, 2u, 1u, predator_fct())
    {}

    virtual typename CoupledModelMono::children_t children(
        const vle::Common &) override final
    {
        return { &prey, &predator };
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const override final
    {
        (void)out;

        if (not prey.y.empty()) {
            vle::copy_values(prey.y[0], predator.x[0]);
            in.emplace(&predator);
        }

        if (not predator.y.empty()) {
            vle::copy_values(predator.y[0], prey.x[1]);
            in.emplace(&prey);
        }
    }
};

class GenericSystem : public CoupledModelMono
{
public:
    GenericSystem(const vle::Context &ctx,
                  std::initializer_list<vle::dsde::qss::Equation<MyTime,
                  state_type>> models)
        : CoupledModelMono(ctx)
        , m_models(models)
    {}

    virtual typename CoupledModelMono::children_t children(
        const vle::Common &) override final
    {
        CoupledModelMono::children_t ret(m_models.size(), nullptr);
        std::transform(m_models.begin(), m_models.end(),
                       ret.begin(),
        [](vle::dsde::qss::Equation<MyTime, state_type> &mdl) {
            return &mdl;
        });
        return ret;
    }

    virtual void post(const UpdatedPort &/*out*/,
                      UpdatedPort &in) const override final
    {
        for (auto i = 0ul, e = m_models.size(); i != e; ++i) {
            if (not m_models[i].y.empty()) {
                for (auto j = 0ul, f = m_models.size(); j != f; ++j) {
                    if (i != j) {
                        vle::copy_values(m_models[i].y[0], m_models[j].x[i]);
                        in.emplace(&m_models[j]);
                    }
                }
            }
        }
    }

    //private:
    std::vector <vle::dsde::qss::Equation<MyTime, state_type>> m_models;
};

class MySystem11 : public CoupledModelMono
{
public:
    vle::dsde::qss::Equation <MyTime, state_type> prey;
    vle::dsde::qss::Equation <MyTime, state_type> predator;

    MySystem11(const vle::Context &ctx, double dq, double epsilon)
        : CoupledModelMono(ctx)
        , prey(ctx, dq, epsilon, X_init, 2u, 0u, prey_fct())
        , predator(ctx, dq, epsilon, Y_init, 2u, 1u, predator_fct())
    {}

    virtual typename CoupledModelMono::children_t children(
        const vle::Common &) override final
    {
        return { &prey, &predator };
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const override final
    {
        (void)out;

        if (not prey.y.empty()) {
            vle::copy_values(prey.y[0], predator.x[0]);
            in.emplace(&predator);
        }

        if (not predator.y.empty()) {
            vle::copy_values(predator.y[0], prey.x[1]);
            in.emplace(&prey);
        }
    }
};

struct push_back_state_and_time {
    std::vector<state_type> &m_states;
    std::vector<double> &m_times;

    push_back_state_and_time(std::vector<state_type> &states,
                             std::vector<double> &times)
        : m_states(states)
        , m_times(times)
    {}

    void operator()(const state_type &x, double t)
    {
        m_states.push_back(x);
        m_times.push_back(t);
    }
};

TEST_CASE("main/simple")
{
    ShowDuration d("main/simple");

    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    const double dq = 1;
    const double epsilon = 1;
    vle::dsde::qss::EquationBlock<MyTime, state_type> model(
        ctx, dq, epsilon, 0, 1u, 0u,
    [](const state_type & x, const double) {
        return -x[0] + 9.5;
    });
    std::ofstream ofs("simple.dat");
    REQUIRE(ofs);
    vle::SimulationStep <MyDSDE> sim(ctx, dsde_engine, model);
    double current = sim.init(0.0);

    while (sim.step(current, 20.0)) {
        ofs << current << '\t'
            << model.value() << '\n';
    }

    sim.finish();
}

TEST_CASE("main/rk", "run")
{
    {
        ShowDuration d("prey/predator: rk4");
        using namespace boost::numeric::odeint;
        state_type x = { X_init, Y_init};
        runge_kutta4 <state_type> stepper;
        std::vector <state_type> x_vec;
        std::vector <double> times;
        std::size_t steps = integrate_const(
                                stepper,
                                prey_predator_fct(),
                                x, 0.0, Finish, 0.01,
                                push_back_state_and_time(x_vec, times));
        std::ofstream ofs("RK4.dat");
        REQUIRE(ofs);

        for (auto i = 0ul; i <= steps; ++i)
            ofs << times[i] << '\t'
                << x_vec[i][0] << '\t'
                << x_vec[i][1] << '\n';
    }
    {
        ShowDuration d("prey/predator: qss1 big block");
        vle::Context ctx = std::make_shared <vle::ContextImpl>();
        MyDSDE dsde_engine;
        vle::dsde::qss::Block <MyTime, state_type> system(
            ctx, {0.001, 0.001}, {0.001, 0.001},
             {X_init, Y_init}, prey_predator_fct());
        std::ofstream ofs("EquationBigBlock.dat");
        REQUIRE(ofs);
        vle::SimulationStep <MyDSDE> sim(ctx, dsde_engine, system);
        double current = sim.init(0.0);

        while (sim.step(current, Finish)) {
            ofs << current << '\t'
                << system.value(0) << '\t'
                << system.value(1) << '\n';
        }

        sim.finish();
    }
    {
        ShowDuration d("prey/predator: qss1 merged integrator");
        vle::Context ctx = std::make_shared <vle::ContextImpl>();
        MyDSDE dsde_engine;
        const double dq = 0.001;
        const double epsilon = 0.001;
        MySystem11 model(ctx, dq, epsilon);
        std::ofstream ofs("Equation.dat");
        REQUIRE(ofs);
        vle::SimulationStep <MyDSDE> sim(ctx, dsde_engine, model);
        double current = sim.init(0.0);

        while (sim.step(current, Finish)) {
            ofs << current << '\t'
                << model.prey.value() << '\t'
                << model.predator.value() << '\n';
        }

        sim.finish();
    }
    {
        ShowDuration d("prey/predator: qss1");
        vle::Context ctx = std::make_shared <vle::ContextImpl>();
        MyDSDE dsde_engine;
        const double dq = 0.001;
        const double epsilon = 0.001;
        MySystem1 model(ctx, dq, epsilon);
        std::ofstream ofs("EquationBlock.dat");
        REQUIRE(ofs);
        vle::SimulationStep <MyDSDE> sim(ctx, dsde_engine, model);
        double current = sim.init(0.0);

        while (sim.step(current, Finish)) {
            ofs << current << '\t'
                << model.prey.value() << '\t'
                << model.predator.value() << '\n';
        }

        sim.finish();
    }

    std::ofstream ofs("plot.gnuplot");
    REQUIRE(ofs);
    ofs << "set terminal wxt\n"
        << "plot 'RK4.dat' using 1:2 w l\n"
        << "replot 'RK4.dat' using 1:3 w l\n"
        << "replot 'Equation.dat' using 1:2 w l\n"
        << "replot 'Equation.dat' using 1:3 w l\n"
        << "replot 'EquationBlock.dat' using 1:3 w l\n"
        << "replot 'EquationBlock.dat' using 1:2 w l\n"
        << "replot 'EquationBigBlock.dat' using 1:2 w l\n"
        << "replot 'EquationBigBlock.dat' using 1:3 w l\n"
        ;
}
