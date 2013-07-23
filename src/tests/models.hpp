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

template < typename T >
struct Infinity
{
    static constexpr T negative = -std::numeric_limits<T>::infinity();
    static constexpr T positive = std::numeric_limits<T>::infinity();
};

typedef vle::Time <double, Infinity<double>> MyTime;

struct MyValueNull
{
    bool is_null(const std::string& value) const
    {
        return value.empty();
    }
};

typedef vle::Value < std::string, MyValueNull > MyValue;

struct ModelB : vle::Dynamics <MyTime, MyValue>
{
    int i;

    ModelB()
        : vle::Dynamics <MyTime, MyValue> ({"in"}, {"out"})
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

struct ModelA : vle::Dynamics <MyTime, MyValue>
{
    int i;

    ModelA()
        : vle::Dynamics <MyTime, MyValue>()
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

struct Counter : vle::Dynamics <MyTime, MyValue>
{
    int i;

    Counter()
        : vle::Dynamics <MyTime, MyValue> ({"in"}, {}), i(0)
    {}

    virtual ~Counter()
    {}

    virtual double start(double) override
    {
        return Infinity<double>::positive;
    }

    virtual double transition(double) override
    {
        dWarning("Counter make a transition");

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

struct Generator : vle::Dynamics <MyTime, MyValue>
{
    unsigned int timestep;
    int i;
    std::mt19937 prng;
    std::normal_distribution < double > dist;

    Generator(unsigned int timestep = 1)
        : vle::Dynamics <MyTime, MyValue>({}, {"out"}),
        timestep(timestep), i(0), prng(1234), dist(0., 5.)
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

struct MyModel : vle::Dynamics <MyTime, MyValue>
{
    MyModel() : vle::Dynamics <MyTime, MyValue> () {}
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

struct MyNetwork : vle::NetworkDynamics <MyTime, MyValue>
{
    Generator gen1, gen2;
    Counter cpt;

    MyNetwork() :
        vle::NetworkDynamics <MyTime, MyValue>()
    {}

    MyNetwork(unsigned int timestep1, unsigned int timestep2)
        : vle::NetworkDynamics <MyTime, MyValue>(), gen1(timestep1),
        gen2(timestep2)
    {}

    virtual ~MyNetwork() {}

    virtual std::vector <Child <MyTime, MyValue>*> children() override
    {
        return {&gen1, &gen2, &cpt};
    }

    virtual std::string observation() const override
    {
        std::string ret = cpt.observation() + " " + gen1.observation() + " " +
            gen2.observation();

        return ret;
    }

    virtual void post(const vle::UpdatedPort <MyTime, MyValue> &y,
                      vle::UpdatedPort <MyTime, MyValue> &x) const override
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

struct MyGenNetwork : vle::NetworkDynamics <MyTime, MyValue>
{
    Generator gen;

    MyGenNetwork(unsigned int timestep)
        : vle::NetworkDynamics <MyTime, MyValue>({}, {"out"}), gen(timestep)
    {}

    virtual ~MyGenNetwork() {}

    virtual std::vector <Child <MyTime, MyValue>*> children() override
    {
        return {&gen};
    }

    virtual std::string observation() const override
    {
        return gen.observation();
    }

    virtual void post(const vle::UpdatedPort <MyTime, MyValue> &out,
                      vle::UpdatedPort <MyTime, MyValue> &in) const override
    {
        dWarning("MyGenNetwork::post out.is_emtpy=", out.empty());

        if (!out.empty())
            y[0] = gen.y[0];
    }
};

struct MyCptNetwork : vle::NetworkDynamics <MyTime, MyValue>
{
    Counter cpt;

    MyCptNetwork()
        : vle::NetworkDynamics <MyTime, MyValue>({"in"}, {}), cpt()
    {}

    virtual ~MyCptNetwork() {}

    virtual std::vector <Child <MyTime, MyValue>*> children() override
    {
        return {&cpt};
    }

    virtual std::string observation() const override
    {
        return cpt.observation();
    }

    virtual void post(const vle::UpdatedPort <MyTime, MyValue> &out,
                      vle::UpdatedPort <MyTime, MyValue> &in) const override
    {
        dWarning("MyCptNetwork::post out.is_emtpy=", out.empty());

        if (!x[0].empty()) {
            cpt.x[0] = x[0];
            in.emplace(&cpt);
        }
    }
};

struct MyGlobalNetwork : vle::NetworkDynamics <MyTime, MyValue>
{
    MyGenNetwork gen1;
    MyGenNetwork gen2;
    MyCptNetwork cpt;

    MyGlobalNetwork(unsigned int timestep1, unsigned int timestep2)
        : vle::NetworkDynamics <MyTime, MyValue>(), gen1(timestep1),
        gen2(timestep2), cpt()
    {}

    virtual ~MyGlobalNetwork() {}

    virtual std::vector <Child <MyTime, MyValue>*> children() override
    {
        return {&cpt, &gen1, &gen2};
    }

    virtual std::string observation() const override
    {
        return (boost::format("%1% %2% %3%") % cpt.observation()
            % gen1.observation() % gen2.observation()).str();
    }

    virtual void post(const vle::UpdatedPort <MyTime, MyValue> &out,
                      vle::UpdatedPort <MyTime, MyValue> &in) const override
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


//struct MyExecutive : vle::Executive <MyTime, MyValue>
//{
    //std::shared_ptr <vle::Dynamics <MyTime, MyValue>> mymodel;

    //MyExecutive()
        //: vle::Executive <MyTime, MyValue>({"in"}, {"out"}),
        //mymodel(new MyModel())
    //{}

    //virtual ~MyExecutive()
    //{}

    //virtual double start(double t) override
    //{
        ////std::printf("MyExecutive::stara %pt\n", parent.get());

        ////std::shared_ptr <vle::Simulator <MyTime, MyValue>> (
            ////new vle::Simulator <MyTime, MyValue>(mymodel));

        //////=
            //////std::shared_ptr <vle::Simulator <MyTime, MyValue>>(mymodel);

        ////push <mymodel> (mymodel);

            //////std::make_shared <vle::NetworkElement <MyTime, MyValue>>(mymodel));

        //return MyTime::infinity;
    //}

    //virtual double transition(double e) override
    //{
        //return MyTime::infinity;
    //}

    //virtual void output() const override
    //{
    //}

    //virtual std::string observation() const override
    //{
        //return std::string();
    //}

    //virtual void post(const vle::UpdatedPort <MyTime, MyValue> &y,
                      //vle::UpdatedPort <MyTime, MyValue> &x) override
    //{
    //}
//};

#endif
