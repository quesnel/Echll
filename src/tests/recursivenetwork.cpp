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

struct StaticHierarchy
{
    typedef Dynamics* Model;

    vle::PortList < MyValue > x;
    vle::PortList < MyValue > y;
    ModelA a;
    ModelB b;

    std::vector < Dynamics* > children;

    StaticHierarchy()
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

TEST_CASE("main/synchronizer/hierarchy/model", "run") {
    vle::Synchronizer < MyTime, std::string,
                        vle::NetworkSimulator < MyTime, std::string,
                                                StaticHierarchy > > a;

    double final_date = a.run(0, 1);

    REQUIRE(final_date == 1.0);
}
