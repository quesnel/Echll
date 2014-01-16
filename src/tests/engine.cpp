/*
 * Copyright (C) 2013 INRA
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

#include <vle/dsde.hpp>


namespace test {

    template < typename T > struct Infinity
    {
        static constexpr T negative = -std::numeric_limits<T>::infinity();
        static constexpr T positive = std::numeric_limits<T>::infinity();
    };

    typedef vle::Time <double, Infinity<double>> MyTime;

    typedef vle::Engine <vle::dsde_engine <MyTime, std::string>> myengine;

    struct AtomicModel : myengine::AtomicModel <MyTime, std::string>
    {
        int i;

        AtomicModel()
            : myengine::AtomicModel <MyTime, std::string>({"in"}, {"out"})
            {}

        virtual double init(const double time) override
        {
            (void)time;
            i = 0;

            return 0.;
        }

        virtual double delta(const double time) override
        {
            i++;
            return .1;
        }

        virtual void output() const override
        {
        }
    };
}

TEST_CASE("engince/dsde_atomicmodel", "run")
{
    test::myengine eng;
    test::AtomicModel mdl;

    vle::Simulation(eng, mdl);
}
