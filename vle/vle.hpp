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

#ifndef __VLE_KERNEL_VLE_HPP__
#define __VLE_KERNEL_VLE_HPP__

#include <vle/time.hpp>
#include <vle/dbg.hpp>

namespace vle {

template <typename Engine>
struct Simulation
{
    typedef Engine engine_type;
    typedef typename Engine::time_type time_type;
    typedef typename Engine::value_type value_type;
    typedef typename Engine::model_type model_type;

    engine_type& engine;
    model_type& model;

    Simulation(engine_type& engine, model_type& model)
        : engine(engine), model(model)
    {}

    time_type run(const time_type &begin, const time_type &end)
    {
        time_type i;
        for (i = engine.pre(model, begin);
             i < end;
             i = engine.run(model, i));

        engine.post(model, i);

        return i;
    }
};

template <typename Engine>
struct SimulationDbg
{
    typedef Engine engine_type;
    typedef typename Engine::time_type time_type;
    typedef typename Engine::value_type value_type;
    typedef typename Engine::model_type model_type;

    engine_type& engine;
    model_type& model;
    std::uintmax_t nbloop;

    SimulationDbg(engine_type& engine, model_type& model)
        : engine(engine), model(model)
    {}

    time_type run(const time_type &begin, const time_type &end)
    {
        time_type i = engine.pre(model, begin);
        time_type prev = i;
        std::uintmax_t localbag = 1;
        nbloop = 0;

        vle::debugf("- - - - - - - - - - - - - start date %f", i);

        for (; i < end; i = engine.run(model, i)) {
            if (prev < i) {
                prev = i;
                localbag = 1;
                vle::debugf("- - - - - - - - - - - - - next date %f", i);
            } else {
                localbag++;
                vle::debugf("- - - - - - - - - - - - - bag %" PRIuMAX, localbag);
            }
            nbloop++;
        }

        engine.post(model, i);

        return i;
    }
};

}

#endif
