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
#include <vle/generic.hpp>
#include <boost/format.hpp>
#include <list>

template <typename T>
struct Infinity
{
    static constexpr T negative = -std::numeric_limits<T>::infinity();
    static constexpr T positive = std::numeric_limits<T>::infinity();
};

typedef vle::Time <double, Infinity<double>> MyTime;
typedef std::string MyValue;
typedef vle::dsde::Engine <MyTime, MyValue> MyDSDE;

using UpdatedPort = vle::dsde::UpdatedPort <MyTime, MyValue>;
using AtomicModel = vle::dsde::AtomicModel <MyTime, MyValue>;
using GenericCoupledModel = vle::dsde::GenericCoupledModel <MyTime, MyValue,
      vle::dsde::TransitionPolicyThread <MyTime, MyValue>>;
using CoupledModelThread = vle::dsde::CoupledModel <MyTime, MyValue,
      vle::dsde::TransitionPolicyThread <MyTime, MyValue>>;
using CoupledModelMono = vle::dsde::CoupledModel <MyTime, MyValue,
      vle::dsde::TransitionPolicyDefault <MyTime, MyValue>>;
using ExecutiveThread = vle::dsde::Executive <MyTime, MyValue,
      vle::dsde::TransitionPolicyThread <MyTime, MyValue>>;
using ExecutiveMono = vle::dsde::Executive <MyTime, MyValue,
      vle::dsde::TransitionPolicyDefault <MyTime, MyValue>>;

struct ModelB : AtomicModel
{
    int i;

    ModelB(const vle::Context& ctx)
        : AtomicModel(ctx, {"in"}, {"out"})
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

    ModelA(const vle::Context& ctx)
        : AtomicModel(ctx)
    {
        x.add_port("in");
        y.add_port("out");
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

    Counter(const vle::Context& ctx)
        : AtomicModel(ctx, {"in"}, {}), i(0)
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
        vle_dbg(ctx, "Counter make a delta with %" PRIuMAX " message(s)"
                    " so, number of received message is %u",
                    (std::uintmax_t)x[0].size(), i);

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

    Generator(const vle::Context& ctx)
        : AtomicModel(ctx, {}, {"out"}),
        timestep(1), i(0), prng(1234), dist(0., 5.)
    {}

    Generator(const vle::Context& ctx, unsigned int timestep)
        : AtomicModel(ctx, {}, {"out"}),
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
        vle_dbg(ctx, "Generator %p delta", this);
        i++;
        return timestep; // return std::abs(dist(prng));
    }

    virtual void lambda() const override final
    {
        vle_dbg(ctx, "Generator lambda %p sends two messages", this);
        y[0] = { std::string("msg"), std::string("msg2") };
    }

    virtual std::string observation() const
    {
        return std::to_string(i);
    }
};

struct MyModel : AtomicModel
{
    MyModel(const vle::Context& ctx) : AtomicModel(ctx) {}
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

    MyNetwork(const vle::Context& ctx) :
        Parent(ctx), gen1(ctx), gen2(ctx), cpt(ctx)
    {}

    MyNetwork(const vle::Context& ctx, unsigned int timestep1,
              unsigned int timestep2)
        : Parent(ctx), gen1(ctx, timestep1), gen2(ctx, timestep2), cpt(ctx)
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

//
//  +--------------+
//  |              |
//  +>-cpt     gen-+>
//  |              |
//  +--------------+
//
template <typename Parent>
struct MyNetworkToCouple : Parent
{
    Generator gen;
    Counter cpt;

    MyNetworkToCouple(const vle::Context& ctx)
        : Parent(ctx, {"in"}, {"out"}), gen(ctx, 1), cpt(ctx)
    {}

    virtual ~MyNetworkToCouple() {}

    virtual typename Parent::children_t children(const vle::Common&) override final
    {
        return {&gen, &cpt};
    }

    virtual std::string observation() const
    {
        std::string ret = cpt.observation() + " " + gen.observation();

        return ret;
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const override final
    {
        (void) out;

        if (!MyNetworkToCouple::x.empty()) {
            vle_dbg(Parent::ctx, "Coupled (%p) send %lu to cpt %lu (%p)",
                    this,
                    MyNetworkToCouple::x[0].size(),
                    cpt.x[0].size(),
                    &cpt);

            cpt.x[0].insert(cpt.x[0].end(),
                            MyNetworkToCouple::x[0].begin(),
                            MyNetworkToCouple::x[0].end());
            in.emplace(&cpt);
        }

        if (!gen.y.empty()) {
            vle_dbg(Parent::ctx, "Gen sends %lu to Coupled %lu",
                    gen.y[0].size(),
                    MyNetworkToCouple::y[0].size());

            MyNetworkToCouple::y[0].insert(MyNetworkToCouple::y[0].end(),
                                           gen.y[0].begin(),
                                           gen.y[0].end());
            in.emplace(this);
        }
    }
};

template <typename Parent>
struct MyRootNetworkToCouple : Parent
{
    std::vector <MyNetworkToCouple <Parent>> m_children;

    MyRootNetworkToCouple(const vle::Context& ctx)
        : Parent(ctx)
    {
        m_children.emplace_back(ctx);
        m_children.emplace_back(ctx);
    }

    virtual ~MyRootNetworkToCouple() {}

    virtual typename Parent::children_t children(const vle::Common&) override final
    {
        typename Parent::children_t ret;

        for (auto& child : m_children)
            ret.emplace_back(&child);

        return std::move(ret);
    }

    virtual std::string observation() const
    {
        std::string ret;

        for (auto& child : m_children)
            ret += child.observation() + " ";

        return std::move(ret);
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const override final
    {
        (void) out;

        if (!m_children[0].y.empty()) {
            vle_dbg(Parent::ctx, "MyRootNetworkToCouple move 0 to 1 (%lu vs %lu)",
                    m_children[1].x[0].size(),
                    m_children[0].y[0].size());
            m_children[1].x[0].insert(m_children[1].x[0].end(),
                                      m_children[0].y[0].begin(),
                                      m_children[0].y[0].end());
            in.emplace(&m_children[1]);
        }

        if (!m_children[1].y.empty()) {
            vle_dbg(Parent::ctx, "MyRootNetworkToCouple move 1 to 0 (%lu vs %lu)",
                    m_children[0].x[0].size(),
                    m_children[1].y[0].size());
            m_children[0].x[0].insert(m_children[0].x[0].end(),
                                      m_children[1].y[0].begin(),
                                      m_children[1].y[0].end());
            in.emplace(&m_children[0]);
        }
    }
};



template <typename Parent>
struct MyFlatNetwork : Parent
{
    std::vector <Generator> gens;
    std::vector <Counter> cpts;

    MyFlatNetwork(const vle::Context& ctx) :
        Parent(ctx)
    {
        for (int i = 0; i < 8; ++i)
            gens.emplace_back(ctx);

        for (int i = 0; i < 4; ++i)
            cpts.emplace_back(ctx);
    }

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

    MyNetworkOfNetwork(const vle::Context& ctx) :
        Parent(ctx)
    {
        models.emplace_back(ctx);
        models.emplace_back(ctx);
        models.emplace_back(ctx);
        models.emplace_back(ctx);
    }

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
        (void)out;

        assert(out.empty());
        assert(Parent::heap.size() == 4);
        assert(out.size() == 0 or out.size() == 4);
    }
};

template <typename Parent>
struct MyNetworkOfNetworkMono : Parent
{
    std::vector <MyNetwork <Parent>> models;

    MyNetworkOfNetworkMono(const vle::Context& ctx) :
        Parent(ctx)
    {
        models.emplace_back(ctx);
        models.emplace_back(ctx);
        models.emplace_back(ctx);
        models.emplace_back(ctx);
    }

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
        (void) out;

        assert(out.empty());
    }
};

template <typename Parent>
struct MyBigNetworkMono : Parent
{
    std::vector <Generator> gens;
    Counter cpt;

    MyBigNetworkMono(const vle::Context& ctx) :
        Parent(ctx), cpt(ctx)
    {
        gens.reserve(1000);

        for (int i = 0, e = 1000; i != e; ++i)
            gens.emplace_back(ctx);
    }

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

    MyBigNetwork(const vle::Context& ctx) :
        Parent(ctx), cpt(ctx)
    {
        gens.reserve(1000);

        for (int i = 0, e = 1000; i != e; ++i)
            gens.emplace_back(ctx);
    }

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

    MyGenNetwork(const vle::Context& ctx,
                 unsigned int timestep)
        : Parent(ctx, {}, {"out"}), gen(ctx, timestep)
    {}

    MyGenNetwork(const vle::Context& ctx,
                 const MyGenNetwork& other)
        : Parent(ctx, other), gen(other.gen)
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

    MyCptNetwork(const vle::Context& ctx)
        : Parent(ctx, {"in"}, {}), cpt(ctx)
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

    MyGlobalNetwork(const vle::Context& ctx,
                    unsigned int timestep1,
                    unsigned int timestep2)
        : Parent(ctx), gen1(ctx, timestep1),
        gen2(ctx, timestep2), cpt(ctx)
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
        vle_dbg(Parent::ctx, "MyGlobalNetwork::post out.is_empty=%s"
                " event: %" PRIuMAX "%" PRIuMAX,
                ((out.empty()) ? "true" : "false"),
                (std::uintmax_t)out.count(&gen1),
                (std::uintmax_t)out.count(&gen2));

        if (out.count(&gen1) + out.count(&gen2) > 0) {
            in.emplace(&cpt);

            cpt.x[0] = gen1.y[0];
            cpt.x[0].insert(cpt.x[0].end(),
                            gen2.y[0].begin(),
                            gen2.y[0].end());

            vle_dbg(Parent::ctx,
                    "MyGlobalNetwork::post message to cpt: %" PRIuMAX,
                    cpt.x[0].size());
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

    MyExecutive(const vle::Context& ctx)
        : ExecutiveMono(ctx), cpt(ctx)
    {}

    virtual ~MyExecutive() {}

    virtual ExecutiveMono::children_t children(const vle::Common& common) override final
    {
        (void)common;

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
            vle_dbg(ExecutiveMono::ctx, "MyExecutive: previous == %f", next);
            if (previous == 5.) {
                vle_dbg(ExecutiveMono::ctx, "MyExecutive: destroy a model");
                erase(&generators.back());
                generators.pop_back();
            } else {
                vle_dbg(ExecutiveMono::ctx, "MyExecutive: build a new model");
                generators.emplace_back(ctx, 1u);
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
        vle_dbg(ctx, "MyExecutive::post out.is_emtpy %s",
                    ((out.empty() ? "true" : "false")));

        if (!out.empty()) {
            for (const auto *child : out) {
                vle_dbg(ctx, "MyExecutive need to copy message from a child");
                cpt.x[0].insert(cpt.x[0].end(),
                                child->y[0].begin(),
                                child->y[0].end());
            }
            in.emplace(&cpt);

            vle_dbg(ctx, "MyExecutive::post `%" PRIuMAX "' message to cpt",
                        cpt.x[0].size());
        }
    }
};

#define CATCH_CONFIG_MAIN
#include "catch.hpp"

TEST_CASE("main/synchronizer/hierarchy/simple_model_api1", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyModel model(ctx);
    vle::Simulation <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0, 10);

    REQUIRE(final_date == 10);
}

TEST_CASE("main/synchronizer/hierarchy/simple_model_api2", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyModel model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0, 10);

    REQUIRE(final_date == 10);
    REQUIRE(sim.nbloop == 10);
}

TEST_CASE("main/synchronizer/hierarchy/flat-network", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyFlatNetwork <CoupledModelMono> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 36 36 36");
}

TEST_CASE("main/synchronizer/hierarchy/myrootnetworktocouple", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyRootNetworkToCouple <CoupledModelMono> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "18 9 18 9 ");
}

TEST_CASE("main/synchronizer/hierarchy/network-of-network", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyNetworkOfNetwork <CoupledModelMono> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 2);
    REQUIRE(final_date == 2);

    std::string result = model.observation();
    REQUIRE(result == "4 1 1 4 1 1 4 1 1 4 1 1");
}

TEST_CASE("main/synchronizer/hierarchy/network-of-network2", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyNetworkOfNetwork <CoupledModelMono> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 5);
    REQUIRE(final_date == 5);

    std::string result = model.observation();
    REQUIRE(result == "16 4 4 16 4 4 16 4 4 16 4 4");
}

TEST_CASE("main/synchronizer/hierarchy/network-of-network-mono", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyNetworkOfNetworkMono <CoupledModelMono> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 9 9 36 9 9 36 9 9 36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy/network-1", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyNetwork <CoupledModelMono> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy/bignetwork-1-thread", "run")
{
    auto mstart = std::chrono::system_clock::now();

    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyBigNetwork <CoupledModelMono> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    auto mend = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = mend - mstart;
    vle_dbg(ctx, "MyBigNetworkThread elapsed time: %f s.",
                elapsed_seconds.count());
}

TEST_CASE("main/synchronizer/hierarchy/bignetwork-1-mono", "run")
{
    auto mstart = std::chrono::system_clock::now();
    vle::Context ctx = std::make_shared <vle::ContextImpl>();

    MyDSDE dsde_engine;
    MyBigNetworkMono <CoupledModelMono> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    auto mend = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = mend - mstart;
    vle_dbg(ctx, "MyBigNetworkMono elapsed time: %f s.",
                elapsed_seconds.count());
}

TEST_CASE("main/synchronizer/hierarchy/network-2", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyNetwork <CoupledModelMono> model(ctx, 1, 2);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "26 9 4");
}

TEST_CASE("main/synchronizer/hierarchy/recursivenetwork-1", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    std::cerr << std::boolalpha;

    MyDSDE dsde_engine;
    MyGlobalNetwork <CoupledModelMono> model(ctx, 1, 1);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy/recursivenetwork-2", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyGlobalNetwork <CoupledModelMono> model(ctx, 1, 2);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "26 9 4");
}

SCENARIO("User API can use copy constructor", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    GIVEN("A simple simulation")
    {
        MyDSDE dsde_engine;
        MyGlobalNetwork <CoupledModelMono> model(ctx, 1, 1);

        WHEN("I run a simulation") {
            vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);
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
                vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, copy);
                double final_date = sim.run(0.0, 10);
                std::string result = copy.observation();

                THEN("The simulation results are the same") {
                    REQUIRE(final_date == 10.0);
                    REQUIRE(result == "36 9 9");
                }
            }
        }

        WHEN("I run a simulation with same object") {
            vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);
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
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyExecutive <CoupledModelMono> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

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
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyFlatNetwork <CoupledModelThread> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 36 36 36");
}

TEST_CASE("main/synchronizer/hierarchy-thread/network-of-network", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyNetworkOfNetwork <CoupledModelThread> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 2);
    REQUIRE(final_date == 2);

    std::string result = model.observation();
    REQUIRE(result == "4 1 1 4 1 1 4 1 1 4 1 1");
}

TEST_CASE("main/synchronizer/hierarchy-thread/network-of-network2", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyNetworkOfNetwork <CoupledModelThread> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 5);
    REQUIRE(final_date == 5);

    std::string result = model.observation();
    REQUIRE(result == "16 4 4 16 4 4 16 4 4 16 4 4");
}

//TEST_CASE("main/synchronizer/hierarchy-thread/generic-coupledmodel-1", "run")
//{
    //typedef vle::dsde::Factory <MyTime, MyValue> factory_t;
    //factory_t factory;
    //factory.functions.emplace("Counter",
                              //[]() -> factory_t::modelptr
                              //{
                                  //return factory_t::modelptr(new Counter());
                              //});

    //factory.functions.emplace("Generator",
                              //[]() -> factory_t::modelptr
                              //{
                                  //return factory_t::modelptr(new Generator());
                              //});

    //{
        //const std::string str = "Unknown\n";
        //std::istringstream is(str);
        //MyDSDE dsde_engine;

        //REQUIRE_THROWS_AS(
            //(vle::dsde::GenericCoupledModel
             //<MyTime, MyValue, vle::dsde::TransitionPolicyThread <
             //MyTime, MyValue>>(is, factory)),
            //vle::dsde::factory_error);
    //}
    //{
        //const std::string str = "Generator\n"
            //"Generator\n"
            //"Generator\n"
            //"Counter\n";
        //std::istringstream is(str);
        //MyDSDE dsde_engine;

        //REQUIRE_THROWS_AS(
            //(vle::dsde::GenericCoupledModel
             //<MyTime, MyValue, vle::dsde::TransitionPolicyThread <
             //MyTime, MyValue>>(is, factory)),
            //vle::dsde::fileformat_error);
    //}
    //{
        //const std::string str = "Generator\n"
            //"Generator\n"
            //"Generator\n"
            //"Counter\n"
            //"#\n"
            //"5 4 0 0";
        //std::istringstream is(str);
        //MyDSDE dsde_engine;

        //REQUIRE_THROWS_AS(
            //(vle::dsde::GenericCoupledModel
             //<MyTime, MyValue, vle::dsde::TransitionPolicyThread <
             //MyTime, MyValue>>(is, factory)),
            //vle::dsde::fileformat_error);
    //}
    //{
        //const std::string str = "Generator\n"
            //"Generator\n"
            //"Generator\n"
            //"Counter\n"
            //"#\n"
            //"1 3 1 0";
        //std::istringstream is(str);
        //MyDSDE dsde_engine;

        //REQUIRE_THROWS_AS(
            //(vle::dsde::GenericCoupledModel
             //<MyTime, MyValue, vle::dsde::TransitionPolicyThread <
             //MyTime, MyValue>>(is, factory)),
            //vle::dsde::fileformat_error);
    //}
    //{
        //const std::string str = "Generator\n"
            //"Generator\n"
            //"Generator\n"
            //"Counter\n"
            //"#\n"
            //"1 4 0 0\n"
            //"2 4 0 0\n"
            //"3 4 0 0\n";

        //std::istringstream is(str);
        //MyDSDE dsde_engine;
        //vle::dsde::GenericCoupledModel
            //<MyTime, MyValue, vle::dsde::TransitionPolicyThread <
                                  //MyTime, MyValue>> model(is, factory);

        //vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

        //double final_date = sim.run(0.0, 10);
        //REQUIRE(final_date == 10.0);
        //REQUIRE(model.m_children.size() == 4u);
        //REQUIRE(model.m_children[3].get());

        //std::string result = dynamic_cast <Counter*>(
            //model.m_children[3].get())->observation();
        //REQUIRE(result == "54");
    //}
    //{
        //const std::string str = "Generator\n"
            //"Generator\n"
            //"Generator\n"
            //"Counter\n"
            //"#\n"
            //"1 4 out in\n"
            //"2 4 out in\n"
            //"3 4 out in\n";

        //std::istringstream is(str);
        //MyDSDE dsde_engine;
        //vle::dsde::GenericCoupledModel
            //<MyTime, MyValue, vle::dsde::TransitionPolicyThread <
            //MyTime, MyValue>> model(is,
                                    //factory,
                                    //GenericCoupledModel::INDEXED_BY_STRING);

        //vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

        //double final_date = sim.run(0.0, 10);
        //REQUIRE(final_date == 10.0);
        //REQUIRE(model.m_children.size() == 4u);
        //REQUIRE(model.m_children[3].get());

        //std::string result = dynamic_cast <Counter*>(
            //model.m_children[3].get())->observation();
        //REQUIRE(result == "54");
    //}
//}

TEST_CASE("main/synchronizer/hierarchy-thread/network-of-network-mono", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyNetworkOfNetworkMono <CoupledModelThread> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 9 9 36 9 9 36 9 9 36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy-thread/network-1", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyNetwork <CoupledModelThread> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy-thread/bignetwork-1-thread", "run")
{
    auto mstart = std::chrono::system_clock::now();

    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyBigNetwork <CoupledModelThread> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    auto mend = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = mend - mstart;
    vle_dbg(ctx, "MyBigNetworkThread elapsed time: %f s.", elapsed_seconds.count());
}

TEST_CASE("main/synchronizer/hierarchy-thread/bignetwork-1-mono", "run")
{
    auto mstart = std::chrono::system_clock::now();

    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyBigNetworkMono <CoupledModelThread> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    auto mend = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = mend - mstart;
    vle_dbg(ctx, "MyBigNetworkMono elapsed time: %f s. ",
                elapsed_seconds.count());
}

TEST_CASE("main/synchronizer/hierarchy-thread/network-2", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyNetwork <CoupledModelThread> model(ctx, 1, 2);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "26 9 4");
}

TEST_CASE("main/synchronizer/hierarchy-thread/recursivenetwork-1", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    std::cerr << std::boolalpha;

    MyDSDE dsde_engine;
    MyGlobalNetwork <CoupledModelThread> model(ctx, 1, 1);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy-thread/recursivenetwork-2", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyGlobalNetwork <CoupledModelThread> model(ctx, 1, 2);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "26 9 4");
}

SCENARIO("User API can use copy constructor with thread", "run")
{
    vle::Context ctx = std::make_shared <vle::ContextImpl>();

    GIVEN("A simple simulation")
    {
        MyDSDE dsde_engine;
        MyGlobalNetwork <CoupledModelThread> model(ctx, 1, 1);

        WHEN("I run a simulation") {
            vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);
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
                vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, copy);
                double final_date = sim.run(0.0, 10);
                std::string result = copy.observation();

                THEN("The simulation results are the same") {
                    REQUIRE(final_date == 10.0);
                    REQUIRE(result == "36 9 9");
                }
            }
        }

        WHEN("I run a simulation with same object") {
            vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);
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
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    MyDSDE dsde_engine;
    MyExecutive <CoupledModelThread> model(ctx);
    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, model);

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
