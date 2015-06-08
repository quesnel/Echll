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
#include <vle/vle.hpp>
#include <vle/devs/devs.hpp>
#include <vle/devs/devs-debug.hpp>
#include <boost/format.hpp>

typedef vle::DoubleTime MyTime;
typedef std::string MyValue;

using Model = vle::devs::Model
              <MyTime, vle::PortList<std::string>,
              vle::PortList<std::string>>;
using AtomicModel = vle::devs::AtomicModel
                    <MyTime, vle::PortList<std::string>,
                    vle::PortList<std::string>>;
using CoupledModel = vle::devs::CoupledModel
                     <MyTime, vle::PortList<std::string>,
                     vle::PortList<std::string>, vle::PortList<std::string>,
                     vle::PortList<std::string>>;
using MyDEVS = vle::devs::Engine <MyTime>;

struct Generator : AtomicModel {
    Generator(const vle::Context &ctx)
        : AtomicModel(ctx, 0u, 1u)
    {}

    virtual ~Generator()
    {}

    virtual double ta() const override final
    {
        return 1.0;
    }

    virtual void lambda() const override final
    {
        y[0].emplace_back("coucou");
    }

    virtual void internal() override final
    {
    }

    virtual void external(const double &time) override final
    {
        (void)time;
    }
};

struct Counter : AtomicModel {
    unsigned long int msg;

    Counter(const vle::Context &ctx)
        : AtomicModel(ctx, 1u, 0u)
        , msg(0u)
    {}

    virtual ~Counter()
    {}

    virtual double ta() const override final
    {
        return MyTime::infinity();
    }

    virtual void lambda() const override final
    {
    }

    virtual void internal() override final
    {
    }

    virtual void external(const double &time) override final
    {
        (void)time;
        msg += x[0].size();
    }
};

struct Network : CoupledModel {
    Generator gen1, gen2;
    Counter counter;

    Network(const vle::Context &ctx)
        : CoupledModel(ctx, 0u, 0u)
        , gen1(ctx)
        , gen2(ctx)
        , counter(ctx)
    {}

    virtual ~Network()
    {}

    virtual CoupledModel::children_t children()
    {
        return { &gen1, &gen2, &counter };
    }

    virtual void post(const Model &out, UpdatedPort &in) const
    {
        (void)out;

        if (not gen1.y.empty()) {
            vle::copy_values(gen1.y[0], counter.x[0]);
            in.emplace(&counter);
        }

        if (not gen2.y.empty()) {
            vle::copy_values(gen2.y[0], counter.x[0]);
            in.emplace(&counter);
        }
    }

    virtual std::size_t select(const std::vector <Model *> &models) const
    {
        /* Alway generator have the priority */
        for (std::size_t i = 0, e = models.size(); i != e; ++i)
            if (models[i] == &gen1 or models[i] == &gen2)
                return i;

        return 0u;
    }

};

#define CATCH_CONFIG_MAIN
#include "catch.hpp"

TEST_CASE("engine/devs/model_a", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDEVS devs_engine;
    Network model(ctx);
    vle::SimulationStep <MyDEVS> sim(ctx, devs_engine);
    double current = sim.init(model, 0.0);
    std::cout << "Init:\n" << model << '\n';

    while (sim.step(model, current, 10.0))
        std::cout << current << " " << model << "\n";

    sim.finish(model);
    /* We need to receive 9 * 2 messages since simulation stops at 10.0. */
    REQUIRE(model.counter.msg == 18u);
}
