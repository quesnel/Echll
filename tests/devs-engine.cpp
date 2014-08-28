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
#include <vector>
#include <iostream>
#include <random>
#include <limits>
#include <vle/vle.hpp>
#include <vle/devs.hpp>
#include <vle/dbg.hpp>
#include <boost/format.hpp>
#include <list>

template < typename T > struct Infinity
{
    static constexpr T negative = -std::numeric_limits<T>::infinity();
    static constexpr T positive = std::numeric_limits<T>::infinity();
};

typedef vle::Time <double, Infinity<double>> MyTime;
typedef vle::devs::Engine <MyTime, std::string> MyDEVS;

using UpdatedPort = vle::devs::UpdatedPort <MyTime, std::string>;
using AtomicModel = vle::devs::AtomicModel <MyTime, std::string>;
using CoupledModel = vle::devs::CoupledModel <MyTime, std::string>;

struct ModelA : AtomicModel
{
    ModelA()
        : AtomicModel({"in"}, {"out"})
    {
    }

    virtual ~ModelA()
    {}

    virtual double ta() const override final
    {
        return 0.0;
    }

    virtual void lambda() const override final
    {
    }

    virtual void internal() const override final
    {
    }

    virtual void external(const double& time) const override final
    {
        (void)time;
    }
};

#define CATCH_CONFIG_MAIN
#include "catch.hpp"

TEST_CASE("engine/devs/model_a", "run")
{
    //MyDEVS devs_engine;
    //ModelA model;

    //vle::Simulation <MyDEVS> sim(devs_engine, model);
    //double final_date = sim.run(0.0, 10.0);

    //REQUIRE(final_date == 10.0);
}
