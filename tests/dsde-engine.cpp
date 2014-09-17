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
using CoupledModelThread = vle::dsde::CoupledModel <MyTime, std::string,
      vle::dsde::TransitionPolicyThread <MyTime, std::string>>;
using CoupledModelMono = vle::dsde::CoupledModel <MyTime, std::string,
      vle::dsde::TransitionPolicyDefault <MyTime, std::string>>;
using ExecutiveThread = vle::dsde::Executive <MyTime, std::string,
      vle::dsde::TransitionPolicyThread <MyTime, std::string>>;
using ExecutiveMono = vle::dsde::Executive <MyTime, std::string,
      vle::dsde::TransitionPolicyDefault <MyTime, std::string>>;

struct ModelB : AtomicModel
{
    int i;

    ModelB()
        : AtomicModel({"in"}, {"out"})
    {}

    virtual ~ModelB()
    {}

    virtual double init(const vle::Common&, const double&) override final
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

    virtual double init(const vle::Common&, const double&) override final
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

    virtual double init(const vle::Common&, const double&) override final
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

    Generator()
        : AtomicModel({}, {"out"}),
        timestep(1), i(0), prng(1234), dist(0., 5.)
    {}

    Generator(unsigned int timestep)
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

    virtual double init(const vle::Common&, const double&) override final
    {
        return timestep; // return std::abs(dist(prng));
    }

    virtual double delta(const double&) override final
    {
        dWarning("Generator ", this, " delta");
        i++;
        return timestep; // return std::abs(dist(prng));
    }

    virtual void lambda() const override final
    {
        dWarning("Generator ", this, " two messages");
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

    virtual double init(const vle::Common&, const double& t) override final
    {
        (void)t;
        return 0.0;
    }

    virtual double delta(const double& e) override final
    {
        (void)e;
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

template <typename Parent>
struct MyNetwork : Parent
{
    Generator gen1, gen2;
    Counter cpt;

    MyNetwork() :
        Parent()
    {}

    MyNetwork(unsigned int timestep1, unsigned int timestep2)
        : Parent()
          , gen1(timestep1)
          , gen2(timestep2)
    {}

    virtual ~MyNetwork() {}

    virtual typename Parent::children_t children(const vle::Common&) override final
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
        assert(Parent::heap.size() == 3);

        if (out.count(&gen1) + out.count(&gen2) > 0) {
            in.emplace(&cpt);
            cpt.x[0].insert(cpt.x[0].begin(),
                            gen1.y[0].begin(),
                            gen1.y[0].end());
            cpt.x[0].insert(cpt.x[0].end(),
                            gen2.y[0].begin(),
                            gen2.y[0].end());
        }
    }
};

template <typename Parent>
struct MyFlatNetwork : Parent
{
    std::vector <Generator> gens;
    std::vector <Counter> cpts;

    MyFlatNetwork() :
        Parent(), gens(8), cpts(4)
    {}

    virtual ~MyFlatNetwork() {}

    virtual typename Parent::children_t children(const vle::Common&) override final
    {
        typename Parent::children_t ret;
        ret.reserve(gens.size() + cpts.size());

        for (auto& gen : gens)
            ret.push_back(&gen);

        for (auto& cpt : cpts)
            ret.push_back(&cpt);

        return ret;
    }

    virtual std::string observation() const
    {
        std::string ret;

        for (int i = 0, e = cpts.size(); i != e; ++i) {
            ret += cpts[i].observation();

            if ((i + 1) != e)
                ret += ' ';
        }

        return ret;
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const override final
    {
        auto have_post = 0;
        for (auto& gen : gens)
            have_post += out.count(&gen);

        if (have_post) {
            for (auto& cpt : cpts)
                in.emplace(&cpt);
        }

        for (int i = 0, e = cpts.size(); i != e; ++i) {
            cpts[i].x[0].insert(cpts[i].x[0].begin(),
                                gens[i * 2].y[0].begin(),
                                gens[i * 2].y[0].end());
            cpts[i].x[0].insert(cpts[i].x[0].begin(),
                                gens[i * 2 + 1].y[0].begin(),
                                gens[i * 2 + 1].y[0].end());

        }
    }
};

template <typename Parent>
struct MyNetworkOfNetwork : Parent
{
    std::vector <MyNetwork <Parent>> models;

    MyNetworkOfNetwork() :
        Parent(), models(4)
    {}

    virtual ~MyNetworkOfNetwork() {}

    virtual typename Parent::children_t children(const vle::Common&) override final
    {
        typename Parent::children_t cs;
        cs.reserve(models.size());

        for (auto& mds : models)
            cs.push_back(&mds);

        return cs;
    }

    virtual std::string observation() const
    {
        std::string ret;
        ret.reserve(256);

        for (int i = 0, e = models.size(); i != e; ++i) {
            ret += models[i].observation();
            if ((i + 1) != e)
                ret += " ";
        }

        return ret;
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &/*in*/) const override final
    {
        assert(out.empty());
        assert(Parent::heap.size() == 4);
        assert(out.size() == 0 or out.size() == 4);
    }
};

template <typename Parent>
struct MyNetworkOfNetworkMono : Parent
{
    std::vector <MyNetwork <Parent>> models;

    MyNetworkOfNetworkMono() :
        Parent(), models(4)
    {}

    virtual ~MyNetworkOfNetworkMono() {}

    virtual typename Parent::children_t children(const vle::Common&) override final
    {
        typename Parent::children_t cs;
        cs.reserve(models.size());

        for (auto& mds : models)
            cs.push_back(&mds);

        return cs;
    }

    virtual std::string observation() const
    {
        std::string ret;
        ret.reserve(256);

        for (int i = 0, e = models.size(); i != e; ++i) {
            ret += models[i].observation();
            if ((i + 1) != e)
                ret += " ";
        }

        return ret;
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &/*in*/) const override final
    {
        assert(out.empty());
    }
};

template <typename Parent>
struct MyBigNetworkMono : Parent
{
    std::vector <Generator> gens;
    Counter cpt;

    MyBigNetworkMono() :
        Parent(), gens(1000)
    {}

    virtual ~MyBigNetworkMono() {}

    virtual typename Parent::children_t children(const vle::Common&) override final
    {
        typename Parent::children_t cs;

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

template <typename Parent>
struct MyBigNetwork : Parent
{
    std::vector <Generator> gens;
    Counter cpt;

    MyBigNetwork() :
        Parent(), gens(1000)
    {}

    virtual ~MyBigNetwork() {}

    virtual typename Parent::children_t children(const vle::Common&) override final
    {
        typename Parent::children_t cs;

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

template <typename Parent>
struct MyGenNetwork : Parent
{
    Generator gen;

    MyGenNetwork(unsigned int timestep)
        : Parent({}, {"out"}), gen(timestep)
    {}

    MyGenNetwork(const MyGenNetwork& other)
        : Parent(other), gen(other.gen)
    {}

    virtual ~MyGenNetwork() {}

    virtual typename Parent::children_t children(const vle::Common&) override final
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
        (void)in;

        if (!out.empty()) {
            Parent::y[0] = gen.y[0];
        }
    }
};

template <typename Parent>
struct MyCptNetwork : Parent
{
    Counter cpt;

    MyCptNetwork()
        : Parent({"in"}, {}), cpt()
    {}

    MyCptNetwork(const MyCptNetwork& other)
        : Parent(other), cpt(other.cpt)
    {}

    virtual ~MyCptNetwork() {}

    virtual typename Parent::children_t children(const vle::Common&) override final
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
            cpt.x[0] = Parent::x[0];
            in.emplace(&cpt);
        }
    }
};

template <typename Parent>
struct MyGlobalNetwork : Parent
{
    MyGenNetwork <Parent> gen1;
    MyGenNetwork <Parent> gen2;
    MyCptNetwork <Parent> cpt;

    MyGlobalNetwork(unsigned int timestep1, unsigned int timestep2)
        : Parent(), gen1(timestep1),
        gen2(timestep2), cpt()
    {}

    MyGlobalNetwork(const MyGlobalNetwork& other)
        : Parent(other),
          gen1(other.gen1), gen2(other.gen2), cpt(other.cpt)
    {}

    virtual ~MyGlobalNetwork() {}

    virtual typename Parent::children_t children(const vle::Common&) override final
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

template <typename Parent>
struct MyExecutive : ExecutiveMono
{
    std::list <MyGenNetwork <Parent>> generators;

    /* TODO segfault when resize: lost the vle::Child* in vle::HeapElement. */
    // std::vector <MyGenNetwork> generators;
    MyCptNetwork <Parent> cpt;
    double previous, next;

    MyExecutive()
        : ExecutiveMono()
    {}

    virtual ~MyExecutive() {}

    virtual ExecutiveMono::children_t children() override final
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

TEST_CASE("main/synchronizer/hierarchy/flat-network", "run")
{
    MyDSDE dsde_engine;
    MyFlatNetwork <CoupledModelMono> model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 36 36 36");
}

TEST_CASE("main/synchronizer/hierarchy/network-of-network", "run")
{
    MyDSDE dsde_engine;
    MyNetworkOfNetwork <CoupledModelMono> model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 2);
    REQUIRE(final_date == 2);

    std::string result = model.observation();
    REQUIRE(result == "4 1 1 4 1 1 4 1 1 4 1 1");
}

TEST_CASE("main/synchronizer/hierarchy/network-of-network2", "run")
{
    MyDSDE dsde_engine;
    MyNetworkOfNetwork <CoupledModelMono> model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 5);
    REQUIRE(final_date == 5);

    std::string result = model.observation();
    REQUIRE(result == "16 4 4 16 4 4 16 4 4 16 4 4");
}

TEST_CASE("main/synchronizer/hierarchy/network-of-network-mono", "run")
{
    MyDSDE dsde_engine;
    MyNetworkOfNetworkMono <CoupledModelMono> model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 9 9 36 9 9 36 9 9 36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy/network-1", "run")
{
    MyDSDE dsde_engine;
    MyNetwork <CoupledModelMono> model;
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
    MyBigNetwork <CoupledModelMono> model;
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
    MyBigNetworkMono <CoupledModelMono> model;
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
    MyNetwork <CoupledModelMono> model(1, 2);
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
    MyGlobalNetwork <CoupledModelMono> model(1, 1);
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy/recursivenetwork-2", "run")
{
    MyDSDE dsde_engine;
    MyGlobalNetwork <CoupledModelMono> model(1, 2);
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
        MyGlobalNetwork <CoupledModelMono> model(1, 1);

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
            MyGlobalNetwork <CoupledModelMono> copy(model);
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
    MyExecutive <CoupledModelMono> model;
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

//
// The same model but using the std::thread implementation
//

TEST_CASE("main/synchronizer/hierarchy-thread/flat-network", "run")
{
    MyDSDE dsde_engine;
    MyFlatNetwork <CoupledModelThread> model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 36 36 36");
}

TEST_CASE("main/synchronizer/hierarchy-thread/network-of-network", "run")
{
    MyDSDE dsde_engine;
    MyNetworkOfNetwork <CoupledModelThread> model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 2);
    REQUIRE(final_date == 2);

    std::string result = model.observation();
    REQUIRE(result == "4 1 1 4 1 1 4 1 1 4 1 1");
}

TEST_CASE("main/synchronizer/hierarchy-thread/network-of-network2", "run")
{
    MyDSDE dsde_engine;
    MyNetworkOfNetwork <CoupledModelThread> model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 5);
    REQUIRE(final_date == 5);

    std::string result = model.observation();
    REQUIRE(result == "16 4 4 16 4 4 16 4 4 16 4 4");
}

TEST_CASE("main/synchronizer/hierarchy-thread/network-of-network-mono", "run")
{
    MyDSDE dsde_engine;
    MyNetworkOfNetworkMono <CoupledModelThread> model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 9 9 36 9 9 36 9 9 36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy-thread/network-1", "run")
{
    MyDSDE dsde_engine;
    MyNetwork <CoupledModelThread> model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy-thread/bignetwork-1-thread", "run")
{
    auto mstart = std::chrono::system_clock::now();

    MyDSDE dsde_engine;
    MyBigNetwork <CoupledModelThread> model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    auto mend = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = mend - mstart;
    dInfo("MyBigNetworkThread elapsed time: ", elapsed_seconds.count(), "s");
}

TEST_CASE("main/synchronizer/hierarchy-thread/bignetwork-1-mono", "run")
{
    auto mstart = std::chrono::system_clock::now();

    MyDSDE dsde_engine;
    MyBigNetworkMono <CoupledModelThread> model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    auto mend = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = mend - mstart;
    dInfo("MyBigNetworkMono elapsed time: ", elapsed_seconds.count(), "s");
}

TEST_CASE("main/synchronizer/hierarchy-thread/network-2", "run")
{
    MyDSDE dsde_engine;
    MyNetwork <CoupledModelThread> model(1, 2);
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "26 9 4");
}

TEST_CASE("main/synchronizer/hierarchy-thread/recursivenetwork-1", "run")
{
    std::cerr << std::boolalpha;

    MyDSDE dsde_engine;
    MyGlobalNetwork <CoupledModelThread> model(1, 1);
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy-thread/recursivenetwork-2", "run")
{
    MyDSDE dsde_engine;
    MyGlobalNetwork <CoupledModelThread> model(1, 2);
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "26 9 4");
}

SCENARIO("User API can use copy constructor with thread", "run")
{
    GIVEN("A simple simulation")
    {
        MyDSDE dsde_engine;
        MyGlobalNetwork <CoupledModelThread> model(1, 1);

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
            MyGlobalNetwork <CoupledModelThread> copy(model);
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

TEST_CASE("main/synchronizer/hierarchy-thread/executive-network-1", "run")
{
    MyDSDE dsde_engine;
    MyExecutive <CoupledModelThread> model;
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
