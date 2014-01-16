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
#include <vle/dbg.hpp>
#include <boost/format.hpp>
#include "models.hpp"

#define CATCH_CONFIG_MAIN
#include "catch.hpp"

TEST_CASE("main/synchronizer/hierarchy/simple_model_api1", "run")
{
    MyDSDE dsde_engine;
    MyModel model;
    vle::Simulation <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0, 10);

    REQUIRE(final_date == 10);
}

TEST_CASE("main/synchronizer/hierarchy/simple_model_api2", "run")
{
    MyDSDE dsde_engine;
    MyModel model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0, 10);

    REQUIRE(final_date == 10);
    REQUIRE(sim.bag == 10);
}

TEST_CASE("main/synchronizer/hierarchy/network-1", "run")
{
    MyDSDE dsde_engine;
    MyNetwork model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy/network-2", "run")
{
    MyDSDE dsde_engine;
    MyNetwork model(1, 2);
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "26 9 4");
}

TEST_CASE("main/synchronizer/hierarchy/recursivenetwork-1", "run")
{
    std::cerr << std::boolalpha;

    MyDSDE dsde_engine;
    MyGlobalNetwork model(1, 1);
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "36 9 9");
}

TEST_CASE("main/synchronizer/hierarchy/recursivenetwork-2", "run")
{
    MyDSDE dsde_engine;
    MyGlobalNetwork model(1, 2);
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    std::string result = model.observation();
    REQUIRE(result == "26 9 4");
}

SCENARIO("User API can use copy constructor", "run")
{
    GIVEN("A simple simulation")
    {
        MyDSDE dsde_engine;
        MyGlobalNetwork model(1, 1);

        WHEN("I run a simulation") {
            vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);
            double final_date = sim.run(0.0, 10);
            std::string result = model.observation();

            THEN("The simulation results are correct") {
                REQUIRE(final_date == 10.0);
                REQUIRE(result == "36 9 9");
            }
        }

        WHEN("I copy the NetworkDynamics") {
            MyGlobalNetwork copy(model);
            WHEN("I run a simulation") {
                vle::SimulationDbg <MyDSDE> sim(dsde_engine, copy);
                double final_date = sim.run(0.0, 10);
                std::string result = copy.observation();

                THEN("The simulation results are the same") {
                    REQUIRE(final_date == 10.0);
                    REQUIRE(result == "36 9 9");
                }
            }
        }

        WHEN("I run a simulation with same object") {
            vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);
            double final_date = sim.run(0.0, 10);
            std::string result = model.observation();

            THEN("The simulation results are correct") {
                REQUIRE(final_date == 10.0);
                REQUIRE(result == "36 9 9");
            }
        }
    }
}

TEST_CASE("main/synchronizer/hierarchy/executive-network-1", "run")
{
    MyDSDE dsde_engine;
    MyExecutive model;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);

    double final_date = sim.run(0.0, 10);
    REQUIRE(final_date == 10.0);

    /*
     56 messages must be observed:
     - at time 0, no generator model
     - at time 1, 2, 3, 4, 6, 7, 8, 9, new model build
     - at time 5, a model is destroyed
     - generator send two message by output
     - generator send output after 1 time unit

         time | nb model | nb message send
            0      0          0
            1      1          0
            2      2          2
            3      3          4
            4      4          6
            5      3          8
            6      4          6
            7      5          8
            8      6          10
            9      7          12
       --------------------------
                              56
       */

    REQUIRE(model.observation() == "56");
}
