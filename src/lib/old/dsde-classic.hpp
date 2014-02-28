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

#ifndef __VLE_KERNEL_DSDSE_CLASSIC_HPP__
#define __VLE_KERNEL_DSDSE_CLASSIC_HPP__

#include <vle/vle.hpp>

namespace vle {

    template <typename Time, typename Value, typename ModelPtr >
        struct StaticFlatSimulator
        {
            ModelPtr model;
            typename Time::type tl, tn;

            StaticFlatSimulator(ModelPtr model)
                : model(model), tl(Time::negative_infinity), tn(Time::infinity)
            {
                model->simulator = reinterpret_cast <void*>(this);
            }

            void start(typename Time::type t)
            {
                tl = t;
                tn = t + model->start(t);
            }

            void transition(typename Time::type time)
            {
                assert(tl <= time && time <= tn);

                if (time < tn and model->x.is_empty())
                    return;

                tn = time + model->transition(time - tl);
                tl = time;
            }

            void output(typename Time::type time)
            {
                if (time == tn)
                    model->output();
            }
        };

    template <typename Time, typename Value, typename GraphManager >
        struct StaticFlatNetworkSimulator
        {
            typedef StaticFlatSimulator <Time, Value, typename GraphManager::Model> Child;

            GraphManager gm;

            /* All the children of the NetworkSimulator. */
            std::vector <Child> children;

            /* Use to optimize the GraphManager::put() function. */
            std::vector <typename GraphManager::Model> last_output_list;

            typename Time::type tl;
            typename Time::type tn;

            StaticFlatNetworkSimulator()
                : tl(Time::negative_infinity), tn(Time::infinity)
            {
                last_output_list.reserve(gm.children.size());
                children.reserve(gm.children.size());

                for (auto & model : gm.children)
                    children.push_back(Child(model));
            }

            void start(typename Time::type t)
            {
                tl = t;
                tn = Time::infinity;

                for (auto & child : children) {
                    child.start(t);

                    if (child.tn < tn)
                        tn = child.tn;
                }
            }

            void transition(typename Time::type time)
            {
                assert(tl <= time && time <= tn);

                typename Time::type next = Time::infinity;

                for (auto & child : children) {
                    if (not child.model->x.is_empty() or time == child.tn)
                        child.transition(time);

                    if (child.tn < next)
                        next = child.tn;
                }

                tn = next;
            }

            void output(typename Time::type time)
            {
                last_output_list.clear();

                if (time == tn) {
                    std::vector <typename GraphManager::Model> lst;

                    for (auto & child : children) {
                        child.output(time);

                        if (!child.model->y.is_empty())
                            lst.push_back(child.model);
                    };

                    gm.put(lst, last_output_list);
                }
            }
        };
}

#endif
