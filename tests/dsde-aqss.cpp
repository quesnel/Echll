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

#include <iostream>
#include <vle/dsde.hpp>
#include <vle/dsde/aqss.hpp>
#include <vle/dsde/qss1.hpp>
#include <vle/dsde/qss2.hpp>
#include <vle/vle.hpp>

typedef vle::DoubleTime MyTime;
typedef double MyValue;
typedef vle::dsde::Engine <MyTime, MyValue> MyDSDE;

using UpdatedPort = vle::dsde::UpdatedPort <MyTime, MyValue>;
using AtomicModel = vle::dsde::AtomicModel <MyTime, MyValue>;
using CoupledModelThread = vle::dsde::CoupledModel <MyTime, MyValue,
      vle::dsde::TransitionPolicyThread <MyTime, MyValue>>;
using CoupledModelMono = vle::dsde::CoupledModel <MyTime, MyValue,
      vle::dsde::TransitionPolicyDefault <MyTime, MyValue>>;
using ExecutiveThread = vle::dsde::Executive <MyTime, MyValue,
      vle::dsde::TransitionPolicyThread <MyTime, MyValue>>;
using ExecutiveMono = vle::dsde::Executive <MyTime, MyValue,
      vle::dsde::TransitionPolicyDefault <MyTime, MyValue>>;

vle::dsde::qss1::compute_fct x_fct  = [](const std::valarray <double>& csts,
                                         const std::valarray <double>& values)
{
    return csts[0] * values[0] - csts[1] * values[0] * values[1];
};

vle::dsde::qss1::compute_fct y_fct = [](const std::valarray <double>& csts,
                                        const std::valarray <double>& values)
{
    return csts[2] * values[0] * values[1] - csts[3] * values[0];
};

struct MySystem1 : CoupledModelMono
{
    const std::valarray <double> csts = { 1.0, 0.05, 0.02, 0.5 };
    vle::dsde::qss1::Equation <MyTime, MyValue> prey;
    vle::dsde::qss1::Equation <MyTime, MyValue> predator;

    MySystem1(const vle::Context& ctx, double dq, double epsilon)
        : CoupledModelMono(ctx)
        , prey(ctx, dq, epsilon, 10.0, 2u, x_fct, csts)
        , predator(ctx, dq, epsilon, 5.0, 2u, y_fct, csts)
    {
    }

    virtual ~MySystem1()
    {
    }

    virtual typename CoupledModelMono::children_t children(
        const vle::Common&) override final
    {
        return { &prey, &predator };
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const override final
    {
        (void)out;

        if (not prey.y.empty()) {
            vle::copy_values(prey.y[0], predator.x[1]);
            in.emplace(&predator);
        }

        if (not predator.y.empty()) {
            vle::copy_values(predator.y[0], prey.x[1]);
            in.emplace(&prey);
        }
    }
};

struct MySystem2 : CoupledModelMono
{
    const std::valarray <double> csts = { 1.0, 0.05, 0.02, 0.5 };
    vle::dsde::qss2::Equation <MyTime, MyValue> prey;
    vle::dsde::qss2::Equation <MyTime, MyValue> predator;

    MySystem2(const vle::Context& ctx, double dq)
        : CoupledModelMono(ctx)
        , prey(ctx, dq, 10.0, 2u, x_fct, csts)
        , predator(ctx, dq, 5.0, 2u, y_fct, csts)
    {
    }

    virtual ~MySystem2()
    {
    }

    virtual typename CoupledModelMono::children_t children(
        const vle::Common&) override final
    {
        return { &prey, &predator };
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const override final
    {
        (void)out;

        if (not prey.y.empty()) {
            vle::copy_values(prey.y[0], predator.x[1]);
            in.emplace(&predator);
        }

        if (not predator.y.empty()) {
            vle::copy_values(predator.y[0], prey.x[1]);
            in.emplace(&prey);
        }
    }
};

#define CATCH_CONFIG_MAIN
#include "catch.hpp"

TEST_CASE("main/dsde/Qss_1", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    const double dq = 0.1;
    const double epsilon = 0.1;

    MySystem1 model(ctx, dq, epsilon);
    std::ofstream ofs("output1.dat");

    vle::SimulationStep <MyDSDE> sim(ctx, dsde_engine, model);

    double current = sim.init(0.0);

    while (sim.step(current, 100.0)) {
        ofs << current << '\t'
            << model.prey.m_integrator.X << '\t'
            << model.predator.m_integrator.X << '\n';
    }

    sim.finish();
}

TEST_CASE("main/dsde/Qss_2", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    const double dq = 0.1;

    MySystem2 model(ctx, dq);
    std::ofstream ofs("output2.dat");

    vle::SimulationStep <MyDSDE> sim(ctx, dsde_engine, model);

    double current = sim.init(0.0);

    while (sim.step(current, 100.0)) {
        ofs << current << '\t'
            << model.prey.m_integrator.X << '\t'
            << model.predator.m_integrator.X << '\n';
    }

    sim.finish();
}
