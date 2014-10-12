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

#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>
#include <vle/vle.hpp>
#include <vle/dsde.hpp>
#include <cmath>
#include <ctime>

template <typename T>
struct Infinity
{
    static constexpr T negative = -std::numeric_limits<T>::infinity();
    static constexpr T positive = std::numeric_limits<T>::infinity();
};

typedef vle::Time <double, Infinity<double>> DoubleTime;
typedef boost::any Data;
typedef vle::dsde::Engine <DoubleTime, Data> MyDSDE;
typedef vle::dsde::Factory <DoubleTime, Data> Factory;

using AtomicModel = vle::dsde::AtomicModel <DoubleTime, Data>;

using GenericCoupledModel = vle::dsde::GenericCoupledModel <DoubleTime, Data,
      vle::dsde::TransitionPolicyThread <DoubleTime, Data>>;

struct Counter : AtomicModel
{
    unsigned int i;

    Counter()
        : AtomicModel({"in"}, {})
        , i(0u)
    {}

    virtual ~Counter()
    {}

    virtual double init(const vle::Common&, const double&) override final
    {
        i = 0;
        return Infinity<double>::positive;
    }

    virtual double delta(const double&) override final
    {
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

struct Generator : AtomicModel
{
    unsigned int timestep;
    unsigned int emitted;

    Generator()
        : AtomicModel({}, {"out"})
        , timestep(1u)
        , emitted(0u)
    {}

    virtual ~Generator()
    {}

    virtual double init(const vle::Common&, const double&) override final
    {
        timestep = 1.0;
        emitted = 0u;

        return timestep;
    }

    virtual double delta(const double&) override final
    {
        ++emitted;

        return timestep;
    }

    virtual void lambda() const override final
    {
        y[0] = { std::string("beep") };
    }

    virtual std::string observation() const
    {
        return std::to_string(emitted);
    }
};

int main(int argc, char *argv[])
{
    if (argc <= 1) {
        std::cerr << argv[0] << " [files...]" << std::endl;
        return EXIT_SUCCESS;
    }

    Factory factory;

    factory.functions.emplace("Counter",
                              []() -> Factory::modelptr
                              {
                                  return Factory::modelptr(new Counter());
                              });

    factory.functions.emplace("Generator",
                              []() -> Factory::modelptr
                              {
                                  return Factory::modelptr(new Generator());
                              });

    for (int i = 1; i < argc; ++i) {
        std::ifstream ifs(argv[i]);

        if (!ifs.is_open()) {
            std::cerr << "Failed to open " << argv[i] << "\n";
            continue;
        }

        try {
            MyDSDE dsde_engine;
            GenericCoupledModel model(ifs, factory);
            vle::SimulationDbg <MyDSDE> sim(dsde_engine, model);
        } catch (const std::exception& e) {
            std::cerr << "Simulation failed: " << e.what() << "\n";
        }
    }

    return EXIT_SUCCESS;
}
