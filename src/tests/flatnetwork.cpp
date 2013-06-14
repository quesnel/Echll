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

#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <random>
#include <vle/vle.hpp>
#include "models.hpp"

#define CATCH_CONFIG_MAIN
#include "catch.hpp"

struct StaticFlat
{
    typedef Dynamics* Model;

    vle::PortList < MyValue > x;
    vle::PortList < MyValue > y;
    ModelA a;
    ModelB b;

    std::vector < Dynamics* > children;

    StaticFlat()
    {
        x.add("input");
        y.add("output");

        children = { &a, &b };
    }

    void put()
    {
        if (not a.y.is_empty())
            b.x[0] = a.y[0];
        if (not b.y.is_empty())
            a.x[0] = b.y[0];

        a.y.clear();
        b.y.clear();
    }
};

struct OnlyOne
{
    typedef ModelA* Model;
    vle::PortList < MyValue > x;
    vle::PortList < MyValue > y;
    ModelA a;

    std::vector < ModelA* > children;

    OnlyOne()
    {
        children = {&a};
    }

    void put()
    {
        a.y.clear();
    }
};

struct StaticFlat2
{
    typedef Dynamics* Model;

    vle::PortList < MyValue > x;
    vle::PortList < MyValue > y;
    Generator g;
    Counter c;

    std::vector < Dynamics* > children;

    StaticFlat2()
        : children({ &g, &c })
    {}

    void put()
    {
        if (not g.y.is_empty())
            c.x[0] = g.y[0];

        g.y.clear();
    }
};

struct StaticFlat3
{
    typedef Dynamics* Model;

    vle::PortList < MyValue > x;
    vle::PortList < MyValue > y;
    Generator g;
    Counter c1;
    Counter c2;

    std::vector < Dynamics* > children;

    StaticFlat3()
        : children({ &g, &c1, &c2 })
    {}

    void put()
    {
        if (not g.y.is_empty()) {
            c1.x[0] = g.y[0];
            c2.x[0] = g.y[0];
        }

        g.y.clear();
    }
};

TEST_CASE("main/synchronizer/NetworkSimulator/OnlyOne-1", "run O")
{
    vle::Synchronizer < MyTime, std::string,
        vle::NetworkSimulator < MyTime, std::string, OnlyOne > > a;

    double final_date = a.run(-1., 1.);

    REQUIRE(a.child.children[0].model->i == 2);

    REQUIRE(final_date == 1.);
}


TEST_CASE("main/synchronizer/NetworkSimulator/OnlyOne-2", "run")
{
    vle::Synchronizer < MyTime, std::string,
        vle::NetworkSimulator < MyTime, std::string, OnlyOne > > a;

    double final_date = a.run(-1., 1000.);

    REQUIRE(a.child.children[0].model->i == 1001);

    REQUIRE(final_date == 1000.);
}

TEST_CASE("main/synchronizer/networksimulator/StaticFlat-1", "run")
{
    vle::Synchronizer < MyTime, std::string,
                        vle::NetworkSimulator < MyTime, std::string,
                                                StaticFlat > > a;

    double final_date = a.run(-1., 1.);

    REQUIRE(a.child.children[0].model->observation() == "2");
    REQUIRE(a.child.children[1].model->observation() == "21");

    REQUIRE(final_date == 1.);
}

TEST_CASE("main/synchronizer/networksimulator/StaticFlat-2", "run")
{
    vle::Synchronizer < MyTime, std::string,
                        vle::NetworkSimulator < MyTime, std::string,
                                                StaticFlat > > a;

    double final_date = a.run(-1., 1000.);

    REQUIRE(a.child.children[0].model->observation() == "1001");
    REQUIRE(a.child.children[1].model->observation() == "10010");

    REQUIRE(final_date == 1000.);
}

TEST_CASE("main/synchronizer/networksimulator/generator-counter", "run")
{
    vle::Synchronizer < MyTime, std::string,
                        vle::NetworkSimulator < MyTime, std::string,
                                                StaticFlat2 > > a;

    double final_date = a.run(-1., 1000.);

    REQUIRE(a.child.children[1].model->observation() == "490");
    REQUIRE(a.child.children[0].model->observation() == "245");

    REQUIRE(final_date >= 1000.);
}

TEST_CASE("main/synchronizer/networksimulator/generator-counters", "run") {
    vle::SynchronizerLoop < MyTime, std::string,
                        vle::NetworkSimulator < MyTime, std::string,
                                                StaticFlat3 > > a(-1.);

    int i = 0;
    while (a.loop(1000.))
        i++;

    REQUIRE(i == 244);
    REQUIRE(a.child.children[2].model->observation() == "490");
    REQUIRE(a.child.children[1].model->observation() == "490");
    REQUIRE(a.child.children[0].model->observation() == "245");
}
