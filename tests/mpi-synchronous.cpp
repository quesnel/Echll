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

#include <vle/mpi.hpp>
#include <vle/vle.hpp>
#include <iostream>
#include <cstdlib>

template <typename T>
struct Infinity
{
    static constexpr T negative = -std::numeric_limits<T>::infinity();
    static constexpr T positive = std::numeric_limits<T>::infinity();
};

typedef vle::Time <double, Infinity<double>> MyTime;
typedef int MyValue;
typedef vle::dsde::Engine <MyTime, MyValue> MyDSDE;

using UpdatedPort = vle::dsde::UpdatedPort <MyTime, MyValue>;
using AtomicModel = vle::dsde::AtomicModel <MyTime, MyValue>;
using CoupledModel = vle::dsde::CoupledModel <MyTime, MyValue>;
using CoupledModelMono = vle::dsde::CoupledModel <MyTime, MyValue, vle::dsde::TransitionPolicyDefault <MyTime, MyValue>>;
using SynchronousProxyModel = vle::dsde::SynchronousProxyModel <MyTime, MyValue>;
using Executive = vle::dsde::Executive <MyTime, MyValue>;
using SynchronousLogicalProcessor = vle::dsde::SynchronousLogicalProcessor <MyTime, MyValue>;

struct Counter : AtomicModel
{
    bool is_rank_0;
    int i;
    double current_time;

    Counter()
        : AtomicModel({"in"}, {}),
        is_rank_0(false),
        i(0)
    {}

    virtual ~Counter()
    {}

    virtual double init(const vle::Common&, const double& time) override final
    {
        if (is_rank_0) {
            current_time = time;
            std::cerr << "Counter init\n";
        }

        i = 0;
        return Infinity<double>::positive;
    }

    virtual double delta(const double& time) override final
    {
        if (is_rank_0) {
            current_time += time;
            std::cerr << "Counter delta t=" << current_time
                << " x[0].size()=" << x[0].size()
                << "\n";
        }

        i += x[0].size();
        return Infinity<double>::positive;
    }

    virtual void lambda() const override final
    {}
};

struct Generator : AtomicModel
{
    unsigned int timestep;

    Generator(unsigned int timestep = 1)
        : AtomicModel({}, {"out"})
        , timestep(timestep)
    {}

    virtual ~Generator()
    {}

    virtual double init(const vle::Common&, const double&) override final
    {
        return timestep;
    }

    virtual double delta(const double&) override final
    {
        return timestep;
    }

    virtual void lambda() const override final
    {
        y[0] = { 1, 2, 3 };
    }
};

struct Network : CoupledModelMono
{
    std::vector <Generator> gens;
    Counter cpt;

    Network() :
        CoupledModelMono(), gens(1)
    {}

    virtual ~Network() {}

    virtual CoupledModelMono::children_t children(const vle::Common&) override final
    {
        CoupledModelMono::children_t cs;

        for (auto& gen : gens)
            cs.push_back(&gen);

        cs.push_back(&cpt);

        return cs;
    }

    virtual void post(const UpdatedPort &out, UpdatedPort &in) const override final
    {
        assert(out.size() == 0 or out.size() == 1);

        if (cpt.is_rank_0)
            std::cerr << "--------------------\nstart loop\n";

        if (cpt.is_rank_0) {
            std::cerr << "Network cpt:" << cpt.x[0].size()
                << " generator: " << gens.size() << "\n";
        }

        for (auto& gen : gens) {
            if (!gen.y[0].empty())
                in.emplace(&cpt);

            if (cpt.is_rank_0) {
                std::cerr << "Network cpt: receives " << gen.y[0].size()
                    << " message(s)\n";
            }

            cpt.x[0].insert(cpt.x[0].end(),
                            gen.y[0].begin(),
                            gen.y[0].end());
        }
        if (cpt.is_rank_0)
            std::cerr << "end loop\n";
    }
};

struct RootNetwork : CoupledModelMono
{
    std::vector <SynchronousProxyModel> pm;
    Network c;

    RootNetwork(boost::mpi::environment* env,
                boost::mpi::communicator* comm)
        : CoupledModelMono(), pm(comm->size() - 1)
    {
        for (int i = 0, e = comm->size() - 1; i != e; ++i) {
            pm[i].environment = env;
            pm[i].communicator = comm;
            pm[i].rank = i + 1;
        }
    }

    virtual ~RootNetwork()
    {}

    virtual CoupledModelMono::children_t children(const vle::Common&) override final
    {
        CoupledModelMono::children_t cs;

        for (auto& p : pm)
            cs.push_back(&p);

        cs.push_back(&c);

        return cs;
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &/*in*/) const override final
    {
        assert(out.empty());
    }
};

int main(int argc, char *argv[])
{
    boost::mpi::environment env(argc, argv);
    boost::mpi::communicator comm;
    int ret = EXIT_FAILURE;

    if (comm.size() == 1) {
        std::cerr << "comm.size() == 1\n";
        goto quit;
    }

    if (comm.rank() == 0) {
        MyDSDE dsde_engine;
        RootNetwork rn(&env, &comm);

        std::cerr << "RootCoordinator " << comm.rank() << "\n";
        vle::SimulationDbg <MyDSDE> sim(dsde_engine, rn);
        rn.c.cpt.is_rank_0 = true;
        double final_date = sim.run(0.0, 1000.0);

        if (final_date != 1000.0) {
            std::cerr << "sim.run(0.0, 1000.0) != 1000.0\n";
            goto quit;
        }

        if (rn.c.cpt.i != 2997) {
            std::cerr << "rn.c.cpt.i (" << rn.c.cpt.i << ") != 2997\n";
            goto quit;
        }

        ret = EXIT_SUCCESS;
    } else {
        SynchronousLogicalProcessor sp;
        sp.environment = &env;
        sp.communicator = &comm;
        sp.parent = 0;

        Network model;

        sp.run(model);

        std::cerr << "LP " << comm.rank() << " counter=" << model.cpt.i << "\n";
        if (model.cpt.i != 2997) {
            std::cerr << "model.cpt.i (" << model.cpt.i << " != 2997\n";
            goto quit;
        }

        ret = EXIT_SUCCESS;
    }

quit:
    return ret;
}
