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

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <string>
#include <cmath>
#include <chrono>
#include <ctime>
#include <vector>
#include <iostream>
#include <random>
#include <limits>
#include <vle/vle.hpp>
#include <vle/dsde.hpp>
#include <vle/dbg.hpp>
#include <boost/format.hpp>
#include <list>

template <typename T>
struct Infinity
{
    static constexpr T negative = -std::numeric_limits<T>::infinity();
    static constexpr T positive = std::numeric_limits<T>::infinity();
};

typedef vle::Time <double, Infinity<double>> MyTime;
typedef vle::dsde::Engine <MyTime, std::string> MyDSDE;

using UpdatedPort = vle::dsde::UpdatedPort <MyTime, std::string>;
using AtomicModel = vle::dsde::AtomicModel <MyTime, std::string>;
using CoupledModel = vle::dsde::CoupledModel <MyTime, std::string>;
using CoupledModelMono = vle::dsde::CoupledModel <MyTime, std::string, vle::dsde::TransitionPolicyDefault <MyTime, std::string>>;

using Executive = vle::dsde::Executive <MyTime, std::string>;

struct ModelB : AtomicModel
{
    int i;

    ModelB()
        : AtomicModel({"in"}, {"out"})
    {}

    virtual ~ModelB()
    {}

    virtual double init(const double&) override final
    {
        i = 0;
        return 0.;
    }

    virtual double delta(const double&) override final
    {
        i++;
        return .1;
    }

    virtual void lambda() const override final
    {}

    std::string observation() const
    {
        return std::to_string(i);
    }
};

struct ModelA : AtomicModel
{
    int i;

    ModelA()
        : AtomicModel()
    {
        x.add("in");
        y.add("out");
    }

    virtual ~ModelA() override
    {}

    virtual double init(const double&) override final
    {
        i = 0;
        return 0.;
    }

    virtual double delta(const double&) override final
    {
        i++;
        return 1.;
    }

    virtual void lambda() const override final
    {}

    virtual std::string observation() const
    {
        return std::to_string(i);
    }
};

struct Counter : AtomicModel
{
    int i;

    Counter()
        : AtomicModel({"in"}, {}), i(0)
    {}

    Counter(const Counter& other)
        : AtomicModel(other), i(other.i)
    {}

    virtual ~Counter()
    {}

    virtual double init(const double&) override final
    {
        i = 0;
        return Infinity<double>::positive;
    }

    virtual double delta(const double&) override final
    {
        dWarning("Counter make a delta with ", x[0].size(), " message(s)",
                 " so, number of received message is ", i);

        i += x[0].size();
        return Infinity<double>::positive;
    }

    virtual void lambda() const override final
    {}

    virtual std::string observation() const
    {
        return std::to_string(i);
    }
};

struct Generator : AtomicModel
{
    unsigned int timestep;
    int i;
    std::mt19937 prng;
    std::normal_distribution < double > dist;

    Generator(unsigned int timestep = 1)
        : AtomicModel({}, {"out"}),
        timestep(timestep), i(0), prng(1234), dist(0., 5.)
    {}

    Generator(const Generator& other)
        : AtomicModel(other),
          timestep(other.timestep), i(other.i),
          prng(other.prng), dist(other.dist)
    {}

    virtual ~Generator()
    {}

    virtual double init(const double&) override final
    {
        return timestep; // return std::abs(dist(prng));
    }

    virtual double delta(const double&) override final
    {
        dWarning("Generator delta");
        i++;
        return timestep; // return std::abs(dist(prng));
    }

    virtual void lambda() const override final
    {
        dWarning("Generator two messages");
        y[0] = { std::string("msg"), std::string("msg2") };
    }

    virtual std::string observation() const
    {
        return std::to_string(i);
    }
};

struct MyModel : AtomicModel
{
    MyModel() : AtomicModel() {}
    virtual ~MyModel() {}

    virtual double init(const double& t) override final
    {
        return 0.0;
    }

    virtual double delta(const double& e) override final
    {
        return 1;
    }

    virtual void lambda() const override final
    {
    }

    virtual std::string observation() const
    {
        return std::string();
    }
};

struct MyNetwork : CoupledModel
{
    Generator gen1, gen2;
    Counter cpt;

    MyNetwork() :
        CoupledModel()
    {}

    MyNetwork(unsigned int timestep1, unsigned int timestep2)
        : CoupledModel(), gen1(timestep1),
        gen2(timestep2)
    {}

    virtual ~MyNetwork() {}

    virtual CoupledModel::children_t children() override final
    {
        return {&gen1, &gen2, &cpt};
    }

    virtual std::string observation() const
    {
        std::string ret = cpt.observation() + " " + gen1.observation() + " " +
            gen2.observation();

        return ret;
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const override final
    {
        if (out.count(&gen1) + out.count(&gen2) > 0) {
            in.emplace(&cpt);
            cpt.x[0] = gen1.y[0];
            cpt.x[0].insert(cpt.x[0].end(),
                            gen2.y[0].begin(),
                            gen2.y[0].end());
        }
    }
};

struct MyBigNetworkMono : CoupledModelMono
{
    std::vector <Generator> gens;
    Counter cpt;

    MyBigNetworkMono() :
        CoupledModel(), gens(1000)
    {}

    virtual ~MyBigNetworkMono() {}

    virtual CoupledModel::children_t children() override final
    {
        CoupledModel::children_t cs;

        for (auto& gen : gens)
            cs.push_back(&gen);

        cs.push_back(&cpt);

        return cs;
    }

    virtual std::string observation() const
    {
        return std::string();
    }

    virtual void post(const UpdatedPort &/*out*/,
                      UpdatedPort &in) const override final
    {
        for (auto& gen : gens) {
            if (gen.y[0].size())
                in.emplace(&cpt);

            cpt.x[0].insert(cpt.x[0].end(),
                            gen.y[0].begin(),
                            gen.y[0].end());
        }
    }
};

struct MyBigNetwork : CoupledModel
{
    std::vector <Generator> gens;
    Counter cpt;

    MyBigNetwork() :
        CoupledModel(), gens(1000)
    {}

    virtual ~MyBigNetwork() {}

    virtual CoupledModel::children_t children() override final
    {
        CoupledModel::children_t cs;

        for (auto& gen : gens)
            cs.push_back(&gen);

        cs.push_back(&cpt);

        return cs;
    }

    virtual std::string observation() const
    {
        return std::string();
    }

    virtual void post(const UpdatedPort &/*out*/,
                      UpdatedPort &in) const override final
    {
        for (auto& gen : gens) {
            if (gen.y[0].size())
                in.emplace(&cpt);

            cpt.x[0].insert(cpt.x[0].end(),
                            gen.y[0].begin(),
                            gen.y[0].end());
        }
    }
};

struct MyGenNetwork : CoupledModel
{
    Generator gen;

    MyGenNetwork(unsigned int timestep)
        : CoupledModel({}, {"out"}), gen(timestep)
    {}

    MyGenNetwork(const MyGenNetwork& other)
        : CoupledModel(other), gen(other.gen)
    {}

    virtual ~MyGenNetwork() {}

    virtual CoupledModel::children_t children() override final
    {
        return {&gen};
    }

    virtual std::string observation() const
    {
        return gen.observation();
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const override final
    {
        if (!out.empty()) {
            y[0] = gen.y[0];
        }
    }
};

struct MyCptNetwork : CoupledModel
{
    Counter cpt;

    MyCptNetwork()
        : CoupledModel({"in"}, {}), cpt()
    {}

    MyCptNetwork(const MyCptNetwork& other)
        : CoupledModel(other), cpt(other.cpt)
    {}

    virtual ~MyCptNetwork() {}

    virtual CoupledModel::children_t children() override final
    {
        return {&cpt};
    }

    virtual std::string observation() const
    {
        return cpt.observation();
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const override final
    {
        if (!out.empty()) {
            cpt.x[0] = x[0];
            in.emplace(&cpt);
        }
    }
};

struct MyGlobalNetwork : CoupledModel
{
    MyGenNetwork gen1;
    MyGenNetwork gen2;
    MyCptNetwork cpt;

    MyGlobalNetwork(unsigned int timestep1, unsigned int timestep2)
        : CoupledModel(), gen1(timestep1),
        gen2(timestep2), cpt()
    {}

    MyGlobalNetwork(const MyGlobalNetwork& other)
        : CoupledModel(other),
          gen1(other.gen1), gen2(other.gen2), cpt(other.cpt)
    {}

    virtual ~MyGlobalNetwork() {}

    virtual CoupledModel::children_t children() override final
    {
        return {&cpt, &gen1, &gen2};
    }

    virtual std::string observation() const
    {
        return (boost::format("%1% %2% %3%") % cpt.observation()
            % gen1.observation() % gen2.observation()).str();
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const override final
    {
        dWarning("MyGlobalNetwork::post out.is_empty=", out.empty(),
                 " event:", out.count(&gen1), ", ", out.count(&gen2));

        if (out.count(&gen1) + out.count(&gen2) > 0) {
            in.emplace(&cpt);

            dWarning("MyGlobalNetwork"
                     " gen1.y[0].size() == ", gen1.y[0].size(),
                     " && gen2.y[0].size() == ", gen2.y[0].size());

            cpt.x[0] = gen1.y[0];
            cpt.x[0].insert(cpt.x[0].end(),
                            gen2.y[0].begin(),
                            gen2.y[0].end());

            dWarning("MyGlobalNetwork::post message to cpt (", cpt.x[0].size(),
                     ")");
        }
    }
};

struct MyExecutive : Executive
{
    std::list <MyGenNetwork> generators;

    /* TODO segfault when resize: lost the vle::Child* in vle::HeapElement. */
    // std::vector <MyGenNetwork> generators;
    MyCptNetwork cpt;
    double previous, next;

    MyExecutive()
        : Executive()
    {}

    virtual ~MyExecutive() {}

    virtual Executive::children_t children() override final
    {
        return {&cpt};
    }

    virtual double init(const double &t) override final
    {
        generators.clear();

        previous = t;
        next = t + 1;

        return next - previous;
    }

    virtual double delta(const double &e) override final
    {
        previous += e;

        if (next == previous) {
            dWarning("MyExecutive: previsous == next == ", next);
            if (previous == 5.) {
                dError("MyExecutive: destroy a model");
                erase(&generators.back());
                generators.pop_back();
            } else {
                dError("MyExecutive: build a new model");
                generators.emplace_back(1u);
                insert(&generators.back());
            }
            next++;
        }

        return next - previous;
    }

    virtual void lambda() const override final
    {
    }

    virtual std::string observation() const
    {
        return cpt.observation();
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const override final
    {
        dWarning("MyExecutive::post out.is_emtpy=", out.empty());

        if (!out.empty()) {
            for (const auto *child : out) {
                dWarning("MyExecutive need to copy message from a child");
                cpt.x[0].insert(cpt.x[0].end(),
                                child->y[0].begin(),
                                child->y[0].end());
            }
            in.emplace(&cpt);
            dWarning("MyExecutive::post `", cpt.x[0].size(),"' message to cpt");
        }
    }
};

#define CATCH_CONFIG_MAIN
#include "catch.hpp"

TEST_CASE("main/synchronizer/hierarchy/simple_model_api1", "run")
{
    MyDSDE dsde_engine;
    MyModel model;
    vle::Simulation <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0, 10);

    REQUIRE(final_date == 10);
}

TEST_CASE("main/synchronizer/hierarchy/simple_model_api2", "run")
{
    MyDSDE dsde_engine;
    MyModel model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0, 10);

    REQUIRE(final_date == 10);
    REQUIRE(sim.bag == 10);
}

TEST_CASE("main/synchronizer/hierarchy/network-1", "run")
{
    MyDSDE dsde_engine;
    MyNetwork model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy/bignetwork-1-thread", "run")
{
    auto mstart = std::chrono::system_clock::now();

    MyDSDE dsde_engine;
    MyBigNetwork model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    auto mend = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = mend - mstart;
    dInfo("MyBigNetworkThread elapsed time: ", elapsed_seconds.count(), "s");
}

TEST_CASE("main/synchronizer/hierarchy/bignetwork-1-mono", "run")
{
    auto mstart = std::chrono::system_clock::now();

    MyDSDE dsde_engine;
    MyBigNetworkMono model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    auto mend = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = mend - mstart;
    dInfo("MyBigNetworkMono elapsed time: ", elapsed_seconds.count(), "s");
}

TEST_CASE("main/synchronizer/hierarchy/network-2", "run")
{
    MyDSDE dsde_engine;
    MyNetwork model(1, 2);
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "26 9 4");
}

TEST_CASE("main/synchronizer/hierarchy/recursivenetwork-1", "run")
{
    std::cerr << std::boolalpha;

    MyDSDE dsde_engine;
    MyGlobalNetwork model(1, 1);
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy/recursivenetwork-2", "run")
{
    MyDSDE dsde_engine;
    MyGlobalNetwork model(1, 2);
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "26 9 4");
}

SCENARIO("User API can use copy constructor", "run")
{
    GIVEN("A simple simulation")
    {
        MyDSDE dsde_engine;
        MyGlobalNetwork model(1, 1);

        WHEN("I run a simulation") {
            vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);
            double final_date = sim.run(0.0, 10);
            std::string result = model.observation();

            THEN("The simulation results are correct") {
                REQUIRE(final_date == 10.0);
                REQUIRE(result == "36 9 9");
            }
        }

        WHEN("I copy the NetworkDynamics") {
            MyGlobalNetwork copy(model);
            WHEN("I run a simulation") {
                vle::SimulationDbg <MyDSDE> sim(dsde_engine, copy);
                double final_date = sim.run(0.0, 10);
                std::string result = copy.observation();

                THEN("The simulation results are the same") {
                    REQUIRE(final_date == 10.0);
                    REQUIRE(result == "36 9 9");
                }
            }
        }

        WHEN("I run a simulation with same object") {
            vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);
            double final_date = sim.run(0.0, 10);
            std::string result = model.observation();

            THEN("The simulation results are correct") {
                REQUIRE(final_date == 10.0);
                REQUIRE(result == "36 9 9");
            }
        }
    }
}

TEST_CASE("main/synchronizer/hierarchy/executive-network-1", "run")
{
    MyDSDE dsde_engine;
    MyExecutive model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    /*
     56 messages must be observed:
     - at time 0, no generator model
     - at time 1, 2, 3, 4, 6, 7, 8, 9, new model build
     - at time 5, a model is destroyed
     - generator send two message by output
     - generator send output after 1 time unit

         time | nb model | nb message send
            0      0          0
            1      1          0
            2      2          2
            3      3          4
            4      4          6
            5      3          8
            6      4          6
            7      5          8
            8      6          10
            9      7          12
       --------------------------
                              56
       */

    REQUIRE(model.observation() == "56");
}
