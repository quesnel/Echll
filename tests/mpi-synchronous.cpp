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

#include <vle/dsde/mpi-synchronous.hpp>
#include <vle/vle.hpp>
#include <iostream>
#include <cstdlib>

typedef vle::DoubleTime MyTime;
typedef vle::PortList <int> MyPort;
typedef vle::dsde::Engine <MyTime> MyDSDE;

using AtomicModel = vle::dsde::AtomicModel <MyTime, MyPort, MyPort>;
using CoupledModelThread = vle::dsde::CoupledModel <MyTime, MyPort, MyPort,
      MyPort, MyPort,
      vle::dsde::TransitionPolicyThread <MyTime>>;
using CoupledModelMono = vle::dsde::CoupledModel <MyTime, MyPort, MyPort,
      MyPort, MyPort,
      vle::dsde::TransitionPolicyDefault <MyTime>>;
using ExecutiveThread = vle::dsde::Executive <MyTime, MyPort, MyPort,
      MyPort, MyPort,
      vle::dsde::TransitionPolicyThread <MyTime>>;
using ExecutiveMono = vle::dsde::Executive <MyTime, MyPort, MyPort,
      MyPort, MyPort,
      vle::dsde::TransitionPolicyDefault <MyTime>>;
using SynchronousLogicalProcessor = vle::dsde::SynchronousLogicalProcessor <MyTime, MyPort, MyPort>;
using SynchronousProxyModel = vle::dsde::SynchronousProxyModel <MyTime, MyPort, MyPort>;

struct Counter : AtomicModel
{
    bool is_rank_0;
    int i;
    double current_time;

    Counter(const vle::Context& ctx)
        : AtomicModel(ctx, 1u, 1u)
        , is_rank_0(false)
        , i(0)
    {}

    virtual double init(const vle::Common&, const double& time) override final
    {
        if (is_rank_0)
            current_time = time;

        i = 0;

        return MyTime::infinity();
    }

    virtual double delta(const double& , const double &,
                         const double &t) override final
    {
        if (is_rank_0)
            current_time = t;

        double ret;

        if (x.empty())
            ret = MyTime::infinity();
        else {
            ret = std::numeric_limits <double>::epsilon();
            i += x[0].size();
        }

        return ret;
    }

    virtual void lambda() const override final
    {
        y[0] = {i};
    }
};

struct Generator : AtomicModel
{
    unsigned int timestep;

    Generator(const vle::Context& ctx,
              unsigned int timestep = 1)
        : AtomicModel(ctx, 0u, 1u)
        , timestep(timestep)
    {}

    virtual double init(const vle::Common&, const double&) override final
    {
        return timestep;
    }

    virtual double delta(const double&, const double&,
                         const double&) override final
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

    Network(const vle::Context& ctx)
        : CoupledModelMono(ctx, 0u, 1u)
        , cpt(ctx)
    {
        gens.emplace_back(ctx);
    }

    virtual CoupledModel::children_t children(const vle::Common&) override final
    {
        CoupledModel::children_t cs;

        for (auto& gen : gens)
            cs.push_back(&gen);

        cs.push_back(&cpt);

        return cs;
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const override final
    {
        (void)out;
        assert(out.size() == 0 or out.size() == 1);

        for (auto& gen : gens) {
            if (!gen.y[0].empty())
                in.emplace(&cpt);

            cpt.x.merge(gen.y, 0, 0);
        }

        if (!cpt.y.empty())
            y.merge(cpt.y, 0, 0);
    }
};

struct RootNetwork : CoupledModelMono
{
    std::vector <SynchronousProxyModel> pm;
    Network c;
    Counter cpt;

    RootNetwork(const vle::Context& ctx)
        : CoupledModelMono(ctx)
        , c(ctx)
        , cpt(ctx)
    {
        boost::mpi::communicator com;

        pm.reserve(boost::numeric_cast <std::size_t>(com.size() - 1));
        for (int i = 0, e = com.size() - 1; i != e; ++i) {
            pm.emplace_back(ctx);
            pm[boost::numeric_cast <std::size_t>(i)].rank = i + 1;
        }
    }

    virtual CoupledModelMono::children_t children(
        const vle::Common&) override final
    {
        CoupledModelMono::children_t ret;

        for (auto& p : pm)
            ret.push_back(&p);

        ret.push_back(&c);
        ret.push_back(&cpt);

        return ret;
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const override final
    {
        if (!out.empty()) {
            in.emplace(&cpt);

            for (auto& o : out)
                if (o != &cpt)
                    cpt.x.merge(o->y, 0, 0);
        }
    }
};

int main(int argc, char *argv[])
{
    boost::mpi::environment env(argc, argv);
    boost::mpi::communicator comm;
    vle::Context ctx = std::make_shared <vle::ContextImpl>();
    int ret = EXIT_FAILURE;

    if (comm.size() == 1) {
        std::cerr << "comm.size() == 1\n";
        goto quit;
    }

    if (comm.rank() == 0) {
        MyDSDE dsde_engine;
        RootNetwork rn(ctx);

        std::cerr << "RootCoordinator " << comm.rank() << "\n";
        vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine);
        rn.c.cpt.is_rank_0 = true;
        double final_date = sim.run(rn, 0.0, 1000.0);

        if (final_date != 1000.0) {
            std::cerr << "sim.run(0.0, 1000.0) != 1000.0\n";
            goto quit;
        }

        if (rn.c.cpt.i != 2997) {
            std::cerr << "rn.c.cpt.i (" << rn.c.cpt.i << ") != 2997\n";
            goto quit;
        }

        if (rn.cpt.i != (999 * comm.size())) {
            std::cerr << "rn.cpt.i (" << rn.cpt.i << ") != "
                      << (999 * comm.size()) << "\n";
            goto quit;
        }

        ret = EXIT_SUCCESS;
    } else {
        SynchronousLogicalProcessor sp;
        sp.parent = 0;

        Network model(ctx);

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
