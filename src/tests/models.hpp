/*
 * Copyright (C) 2013 INRA
 * Copyright (C) 2013 ULCO
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

#ifndef VLE_2_MODELS_HPP
#define VLE_2_MODELS_HPP

#include <limits>
#include <vle/vle.hpp>
#include <vle/dbg.hpp>
#include <boost/format.hpp>
#include <list>

template < typename T >
struct Infinity
{
    static constexpr T negative = -std::numeric_limits<T>::infinity();
    static constexpr T positive = std::numeric_limits<T>::infinity();
};

typedef vle::Time <double, Infinity<double>> MyTime;

struct ModelB : vle::Dynamics <MyTime, std::string>
{
    int i;

    ModelB()
        : vle::Dynamics <MyTime, std::string> ({"in"}, {"out"})
    {}

    virtual ~ModelB() override
    {}

    virtual double start(double) override
    {
        i = 0;
        return 0.;
    }

    virtual double transition(double) override
    {
        i++;
        return .1;
    }

    virtual void output() const override
    {}

    virtual std::string observation() const override
    {
        return std::to_string(i);
    }
};

struct ModelA : vle::Dynamics <MyTime, std::string>
{
    int i;

    ModelA()
        : vle::Dynamics <MyTime, std::string>()
    {
        x.add("in");
        y.add("out");
    }

    virtual ~ModelA() override
    {}

    virtual double start(double) override
    {
        i = 0;
        return 0.;
    }

    virtual double transition(double) override
    {
        i++;
        return 1.;
    }

    virtual void output() const override
    {}

    virtual std::string observation() const override
    {
        return std::to_string(i);
    }
};

struct Counter : vle::Dynamics <MyTime, std::string>
{
    int i;

    Counter()
        : vle::Dynamics <MyTime, std::string> ({"in"}, {}), i(0)
    {}

    Counter(const Counter& other)
        : vle::Dynamics <MyTime, std::string>(other), i(other.i)
    {}

    virtual ~Counter()
    {}

    virtual double start(double) override
    {
        return Infinity<double>::positive;
    }

    virtual double transition(double) override
    {
        dWarning("Counter make a transition with ", x[0].size(), " message(s)",
                 " so, number of received message is ", i);

        i += x[0].size();
        return Infinity<double>::positive;
    }

    virtual void output() const override
    {}

    virtual std::string observation() const override
    {
        return std::to_string(i);
    }
};

struct Generator : vle::Dynamics <MyTime, std::string>
{
    unsigned int timestep;
    int i;
    std::mt19937 prng;
    std::normal_distribution < double > dist;

    Generator(unsigned int timestep = 1)
        : vle::Dynamics <MyTime, std::string>({}, {"out"}),
        timestep(timestep), i(0), prng(1234), dist(0., 5.)
    {}

    Generator(const Generator& other)
        : vle::Dynamics <MyTime, std::string>(other),
          timestep(other.timestep), i(other.i),
          prng(other.prng), dist(other.dist)
    {}

    virtual ~Generator()
    {}

    virtual double start(double) override
    {
        return timestep; // return std::abs(dist(prng));
    }

    virtual double transition(double) override
    {
        dWarning("Generator transition");
        i++;
        return timestep; // return std::abs(dist(prng));
    }

    virtual void output() const override
    {
        dWarning("Generator send output");
        y[0] = { std::string("msg"), std::string("msg2") };
    }

    virtual std::string observation() const override
    {
        return std::to_string(i);
    }
};

struct MyModel : vle::Dynamics <MyTime, std::string>
{
    MyModel() : vle::Dynamics <MyTime, std::string> () {}
    virtual ~MyModel() {}

    virtual double start(double t) override
    {
        return 0.0;
    }

    virtual double transition(double e) override
    {
        return 1;
    }

    virtual void output() const override
    {
    }

    virtual std::string observation() const override
    {
        return std::string();
    }
};

struct MyNetwork : vle::NetworkDynamics <MyTime, std::string>
{
    Generator gen1, gen2;
    Counter cpt;

    MyNetwork() :
        vle::NetworkDynamics <MyTime, std::string>()
    {}

    MyNetwork(unsigned int timestep1, unsigned int timestep2)
        : vle::NetworkDynamics <MyTime, std::string>(), gen1(timestep1),
        gen2(timestep2)
    {}

    virtual ~MyNetwork() {}

    virtual std::vector <Child <MyTime, std::string>*> children() override
    {
        return {&gen1, &gen2, &cpt};
    }

    virtual std::string observation() const override
    {
        std::string ret = cpt.observation() + " " + gen1.observation() + " " +
            gen2.observation();

        return ret;
    }

    virtual void post(const vle::UpdatedPort <MyTime, std::string> &y,
                      vle::UpdatedPort <MyTime, std::string> &x) const override
    {
        if (y.count(&gen1) + y.count(&gen2) > 0) {
            x.emplace(&cpt);
            cpt.x[0] = gen1.y[0];
            cpt.x[0].insert(cpt.x[0].end(),
                            gen2.y[0].begin(),
                            gen2.y[0].end());
        }
    }
};

struct MyGenNetwork : vle::NetworkDynamics <MyTime, std::string>
{
    Generator gen;

    MyGenNetwork(unsigned int timestep)
        : vle::NetworkDynamics <MyTime, std::string>({}, {"out"}), gen(timestep)
    {}

    MyGenNetwork(const MyGenNetwork& other)
        : vle::NetworkDynamics <MyTime, std::string>(other), gen(other.gen)
    {}

    virtual ~MyGenNetwork() {}

    virtual std::vector <Child <MyTime, std::string>*> children() override
    {
        return {&gen};
    }

    virtual std::string observation() const override
    {
        return gen.observation();
    }

    virtual void post(const vle::UpdatedPort <MyTime, std::string> &out,
                      vle::UpdatedPort <MyTime, std::string> &in) const override
    {
        dWarning("MyGenNetwork::post out.is_emtpy=", out.empty());

        if (!out.empty())
            y[0] = gen.y[0];
    }
};

struct MyCptNetwork : vle::NetworkDynamics <MyTime, std::string>
{
    Counter cpt;

    MyCptNetwork()
        : vle::NetworkDynamics <MyTime, std::string>({"in"}, {}), cpt()
    {}

    MyCptNetwork(const MyCptNetwork& other)
        : vle::NetworkDynamics <MyTime, std::string>(other), cpt(other.cpt)
    {}

    virtual ~MyCptNetwork() {}

    virtual std::vector <Child <MyTime, std::string>*> children() override
    {
        return {&cpt};
    }

    virtual std::string observation() const override
    {
        return cpt.observation();
    }

    virtual void post(const vle::UpdatedPort <MyTime, std::string> &out,
                      vle::UpdatedPort <MyTime, std::string> &in) const override
    {
        dWarning("MyCptNetwork::post out.is_emtpy=", out.empty());

        if (!x[0].empty()) {
            cpt.x[0] = x[0];
            in.emplace(&cpt);
        }
    }
};

struct MyGlobalNetwork : vle::NetworkDynamics <MyTime, std::string>
{
    MyGenNetwork gen1;
    MyGenNetwork gen2;
    MyCptNetwork cpt;

    MyGlobalNetwork(unsigned int timestep1, unsigned int timestep2)
        : vle::NetworkDynamics <MyTime, std::string>(), gen1(timestep1),
        gen2(timestep2), cpt()
    {}

    MyGlobalNetwork(const MyGlobalNetwork& other)
        : vle::NetworkDynamics <MyTime, std::string>(other),
          gen1(other.gen1), gen2(other.gen2), cpt(other.cpt)
    {}

    virtual ~MyGlobalNetwork() {}

    virtual std::vector <Child <MyTime, std::string>*> children() override
    {
        return {&cpt, &gen1, &gen2};
    }

    virtual std::string observation() const override
    {
        return (boost::format("%1% %2% %3%") % cpt.observation()
            % gen1.observation() % gen2.observation()).str();
    }

    virtual void post(const vle::UpdatedPort <MyTime, std::string> &out,
                      vle::UpdatedPort <MyTime, std::string> &in) const override
    {
        dWarning("MyGlobalNetwork::post out.is_emtpy=", out.empty(),
                 " event:", out.count(&gen1), ", ", out.count(&gen2));

        if (out.count(&gen1) + out.count(&gen2) > 0) {
            in.emplace(&cpt);
            cpt.x[0] = gen1.y[0];
            cpt.x[0].insert(cpt.x[0].end(),
                            gen2.y[0].begin(),
                            gen2.y[0].end());

            dWarning("MyGlobalNetwork::post message to cpt");
        }
    }
};

struct MyExecutive : vle::Executive <MyTime, std::string>
{
    std::list <MyGenNetwork> generators;

    /* TODO segfault when resize: lost the vle::Child* in vle::HeapElement. */
    // std::vector <MyGenNetwork> generators;
    MyCptNetwork cpt;
    double previous, next;

    MyExecutive()
        : vle::Executive <MyTime, std::string>()
    {}

    virtual ~MyExecutive() {}

    virtual std::vector <Child <MyTime, std::string>*> children() override
    {
        return {&cpt};
    }

    virtual double start(double t) override
    {
        previous = t;
        next = t + 1;

        return next - previous;
    }

    virtual double transition(double e)
    {
        previous += e;

        if (next == previous) {
            dWarning("MyExecutive: previsous == next == ", next);
            if (previous == 5.) {
                dError("MyExecutive: destroy a model");
                destroy(&generators.back());
                generators.pop_back();
            } else {
                dError("MyExecutive: build a new model");
                generators.emplace_back(1u);
                push(&generators.back());
            }
            next++;
        }

        return next - previous;
    }

    virtual void output() const override
    {
    }

    virtual std::string observation() const override
    {
        return cpt.observation();
    }

    virtual void post(const vle::UpdatedPort <MyTime, std::string> &out,
                      vle::UpdatedPort <MyTime, std::string> &in) const override
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

#endif
