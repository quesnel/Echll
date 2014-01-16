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
#include <vle/dsde.hpp>
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
typedef vle::dsde <MyTime, std::string> MyDSDE;

struct ModelB : MyDSDE::AtomicModel
{
    int i;

    ModelB()
        : MyDSDE::AtomicModel({"in"}, {"out"})
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

struct ModelA : MyDSDE::AtomicModel
{
    int i;

    ModelA()
        : MyDSDE::AtomicModel()
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

struct Counter : MyDSDE::AtomicModel
{
    int i;

    Counter()
        : MyDSDE::AtomicModel({"in"}, {}), i(0)
    {}

    Counter(const Counter& other)
        : MyDSDE::AtomicModel(other), i(other.i)
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

struct Generator : MyDSDE::AtomicModel
{
    unsigned int timestep;
    int i;
    std::mt19937 prng;
    std::normal_distribution < double > dist;

    Generator(unsigned int timestep = 1)
        : MyDSDE::AtomicModel({}, {"out"}),
        timestep(timestep), i(0), prng(1234), dist(0., 5.)
    {}

    Generator(const Generator& other)
        : MyDSDE::AtomicModel(other),
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

struct MyModel : MyDSDE::AtomicModel
{
    MyModel() : MyDSDE::AtomicModel() {}
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

struct MyNetwork : MyDSDE::CoupledModel
{
    Generator gen1, gen2;
    Counter cpt;

    MyNetwork() :
        MyDSDE::CoupledModel()
    {}

    MyNetwork(unsigned int timestep1, unsigned int timestep2)
        : MyDSDE::CoupledModel(), gen1(timestep1),
        gen2(timestep2)
    {}

    virtual ~MyNetwork() {}

    virtual MyDSDE::CoupledModel::children_t children() override final
    {
        return {&gen1, &gen2, &cpt};
    }

    virtual std::string observation() const
    {
        std::string ret = cpt.observation() + " " + gen1.observation() + " " +
            gen2.observation();

        return ret;
    }

    virtual void post(const MyDSDE::UpdatedPort &out,
                      MyDSDE::UpdatedPort &in) const override final
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

struct MyGenNetwork : MyDSDE::CoupledModel
{
    Generator gen;

    MyGenNetwork(unsigned int timestep)
        : MyDSDE::CoupledModel({}, {"out"}), gen(timestep)
    {}

    MyGenNetwork(const MyGenNetwork& other)
        : MyDSDE::CoupledModel(other), gen(other.gen)
    {}

    virtual ~MyGenNetwork() {}

    virtual MyDSDE::CoupledModel::children_t children() override final
    {
        return {&gen};
    }

    virtual std::string observation() const
    {
        return gen.observation();
    }

    virtual void post(const MyDSDE::UpdatedPort &out,
                      MyDSDE::UpdatedPort &in) const override final
    {
        if (!out.empty()) {
            y[0] = gen.y[0];
        }
    }
};

struct MyCptNetwork : MyDSDE::CoupledModel
{
    Counter cpt;

    MyCptNetwork()
        : MyDSDE::CoupledModel({"in"}, {}), cpt()
    {}

    MyCptNetwork(const MyCptNetwork& other)
        : MyDSDE::CoupledModel(other), cpt(other.cpt)
    {}

    virtual ~MyCptNetwork() {}

    virtual MyDSDE::CoupledModel::children_t children() override final
    {
        return {&cpt};
    }

    virtual std::string observation() const
    {
        return cpt.observation();
    }

    virtual void post(const MyDSDE::UpdatedPort &out,
                      MyDSDE::UpdatedPort &in) const override final
    {
        if (!out.empty()) {
            cpt.x[0] = x[0];
            in.emplace(&cpt);
        }
    }
};

struct MyGlobalNetwork : MyDSDE::CoupledModel
{
    MyGenNetwork gen1;
    MyGenNetwork gen2;
    MyCptNetwork cpt;

    MyGlobalNetwork(unsigned int timestep1, unsigned int timestep2)
        : MyDSDE::CoupledModel(), gen1(timestep1),
        gen2(timestep2), cpt()
    {}

    MyGlobalNetwork(const MyGlobalNetwork& other)
        : MyDSDE::CoupledModel(other),
          gen1(other.gen1), gen2(other.gen2), cpt(other.cpt)
    {}

    virtual ~MyGlobalNetwork() {}

    virtual MyDSDE::CoupledModel::children_t children() override final
    {
        return {&cpt, &gen1, &gen2};
    }

    virtual std::string observation() const
    {
        return (boost::format("%1% %2% %3%") % cpt.observation()
            % gen1.observation() % gen2.observation()).str();
    }

    virtual void post(const MyDSDE::UpdatedPort &out,
                      MyDSDE::UpdatedPort &in) const override final
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

struct MyExecutive : MyDSDE::Executive
{
    std::list <MyGenNetwork> generators;

    /* TODO segfault when resize: lost the vle::Child* in vle::HeapElement. */
    // std::vector <MyGenNetwork> generators;
    MyCptNetwork cpt;
    double previous, next;

    MyExecutive()
        : MyDSDE::Executive()
    {}

    virtual ~MyExecutive() {}

    virtual MyDSDE::Executive::children_t children() override final
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

    virtual void post(const MyDSDE::UpdatedPort &out,
                      MyDSDE::UpdatedPort &in) const override final
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
