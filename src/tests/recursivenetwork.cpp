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
#include <boost/format.hpp>
#include "models.hpp"

#define CATCH_CONFIG_MAIN
#include "catch.hpp"

TEST_CASE("main/synchronizer/hierarchy/simple_model_api1", "run")
{
    MyModel model;
    vle::Synchronizer <MyTime, std::string> a(&model);

    double final_date = a.run(0, 10);

    REQUIRE(final_date == 10);
}

TEST_CASE("main/synchronizer/hierarchy/simple_model_api2", "run")
{
    MyModel model;
    vle::SynchronizerBagCounter <MyTime, std::string> a(&model);

    double final_date = a.run(0, 10);

    REQUIRE(final_date == 10);
    REQUIRE(a.bag == 10);
}

TEST_CASE("main/synchronizer/hierarchy/network-1", "run")
{
    MyNetwork small;
    vle::Synchronizer <MyTime, std::string> a(&small);

    double final_date = a.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = small.observation();
    REQUIRE(result == "36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy/network-2", "run")
{
    MyNetwork small(1, 2);
    vle::Synchronizer <MyTime, std::string> a(&small);

    double final_date = a.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = small.observation();
    REQUIRE(result == "26 9 4");
}

TEST_CASE("main/synchronizer/hierarchy/recursivenetwork-1", "run")
{
    MyGlobalNetwork small(1, 1);
    vle::Synchronizer <MyTime, std::string> a(&small);

    double final_date = a.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = small.observation();
    REQUIRE(result == "36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy/recursivenetwork-2", "run")
{
    MyGlobalNetwork small(1, 2);
    vle::Synchronizer <MyTime, std::string> a(&small);

    double final_date = a.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = small.observation();
    REQUIRE(result == "26 9 4");
}

SCENARIO("User API can use copy constructor", "run")
{
    GIVEN("A simple simulation")
    {
        MyGlobalNetwork small(1, 1);

        WHEN("I run a simulation") {
            vle::Synchronizer <MyTime, std::string> a(&small);
            double final_date = a.run(0.0, 10);
            std::string result = small.observation();

            THEN("The simulation results are correct") {
                REQUIRE(final_date == 10.0);
                REQUIRE(result == "36 9 9");
            }
        }

        WHEN("I copy the NetworkDynamics") {
            MyGlobalNetwork copy(small);
            WHEN("I run a simulation") {
                vle::Synchronizer <MyTime, std::string> a(&copy);
                double final_date = a.run(0.0, 10);
                std::string result = copy.observation();

                THEN("The simulation results are the same") {
                    REQUIRE(final_date == 10.0);
                    REQUIRE(result == "36 9 9");
                    REQUIRE(copy.element != small.element);
                }
            }
        }
    }
}


// TEST_CASE("main/synchronizer/hierarchy/executive-network-1", "run")
// {
//     MyExecutive exe;
//     vle::Synchronizer <MyTime, std::string> a(&exe);

//     double final_date = a.run(0.0, 10);
//     REQUIRE(final_date == 10.0);

//     REQUIRE(exe.observation() == "26");
// }
