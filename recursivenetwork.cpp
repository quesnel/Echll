/*
 * Copyright (C) 2013 INRA
 * Copyright (C) 2013 ULCO
 *
 * Gauthier Quesnel gauthier.quesnel@users.sourceforge.net
 * Éric Ramat ramat@lisic.univ-littoral.fr
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <random>
#include "vle.hpp"
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

TEST_CASE("main/synchronizer/recursive/model", "run") {
    vle::Synchronizer < MyTime, std::string,
                        vle::NetworkSimulator < MyTime, std::string,
                                                StaticFlat > > a;

    double final_date = a.run(0, 1);

    REQUIRE(final_date == 1.0);
}
