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

#ifndef __VLE_KERNEL_DSDE_HEAP_HPP__
#define __VLE_KERNEL_DSDE_HEAP_HPP__

#include <vle/vle.hpp>
#include <boost/heap/fibonacci_heap.hpp>
#include <set>

namespace vle {

    template < typename Simulator >
        struct StaticFlatHeapSimulatorCompare
        : std::binary_function < Simulator, Simulator, bool >
        {
            bool operator()(const Simulator &lhs, const Simulator &rhs) const
            {
                return lhs.tn > rhs.tn;
            }
        };

    template < typename HandleType >
        struct StaticFlatHeapHandleTypeCompare
        : std::binary_function < HandleType, HandleType, bool >
        {
            bool operator()(const HandleType &lhs, const HandleType &rhs) const
            {
                return (*lhs).model < (*rhs).model;
            }
        };

    template < typename Simulator >
        struct StaticFlatHeapScheduler
        {
            typedef boost::heap::fibonacci_heap <
                Simulator, boost::heap::compare <
                StaticFlatHeapSimulatorCompare < Simulator > > > Type;

            typedef typename boost::heap::fibonacci_heap <
                Simulator, boost::heap::compare <
                StaticFlatHeapSimulatorCompare < Simulator > > >::handle_type Handle;
        };

    template < typename Time, typename Value, typename ModelPtr >
        struct StaticFlatHeapSimulator
        {
            ModelPtr model;
            typename Heap < StaticFlatHeapSimulator < Time, Value, ModelPtr >>::Handle heapid;
            typename Time::type tl, tn;

            StaticFlatHeapSimulator(ModelPtr model)
                : model(model), tl(Time::negative_infinity),
                tn(Time::infinity)
            {
                model->simulator = reinterpret_cast <void*>(this);
            }

            StaticFlatHeapSimulator(const StaticFlatHeapSimulator &other)
                : model(other.model), tl(other.tn), tn(other.tn)
            {
                model->simulator = reinterpret_cast <void*>(this);
            }

            void start(typename Time::type t)
            {
                tl = t;
                tn = t + model->start(t);
            }

            void transition(typename Time::type t)
            {
                assert(tl <= t && t <= tn);

                if (t < tn and model->x.is_empty())
                    return;

                tn = t + model->transition(t - tl);
                tl = t;
            }

            void output(typename Time::type t) const
            {
                if (t == tn)
                    model->output();
            }
        };

    template < typename Time, typename Value, typename GraphManager >
        struct StaticFlatHeapNetworkSimulator
        {
            typedef typename Heap < StaticFlatHeapSimulator < Time, Value, typename
                GraphManager::Model > > ::Type Scheduler;

            typedef StaticFlatHeapSimulator < Time, Value, typename
                GraphManager::Model > Child;

            /* The scheduler stores all children of the NetworkSimulator. */
            Scheduler heap;

            /* Use to optimize the GraphManager::put() function. */
            std::vector <typename GraphManager::Model> last_output_list;

            mutable GraphManager gm; /* Very bad use of mutable. But we need to
                                        call the put() function into output
                                        function. */
            typename Time::type tl, tn;

            StaticFlatHeapNetworkSimulator()
                : tl(Time::negative_infinity), tn(Time::infinity)
            {
                last_output_list.reserve(gm.children.size());

                for (auto & child : gm.children) {
                    auto id = heap.push(StaticFlatHeapSimulator < Time, Value,
                                        typename GraphManager::Model >(child));
                    (*id).heapid = id;
                }
            }

            void start(typename Time::type t)
            {
                tl = t;
                tn = Time::infinity;

                for (auto & child : gm.children) {

                    (reinterpret_cast <Child*>(child->simulator))->start(t);
                    heap.update(
                        (reinterpret_cast <Child*>(child->simulator))->heapid);
                }

                tn = heap.top().tn;
            }

            void transition(typename Time::type t)
            {
                assert(tl <= t && t <= tn);
                assert(tn == heap.top().tn);

                if (t < tn && gm.x.is_empty())
                    return;

                if (not heap.empty()) {
                    /*
                     * We use the std::set class to make sure that only one
                     * model appears in the bag.
                     */
                    std::set <typename Heap <StaticFlatHeapSimulator <Time,
                        Value, typename GraphManager::Model>>::Handle,
                        StaticFlatHeapHandleTypeCompare <typename Heap
                            <StaticFlatHeapSimulator <Time, Value, typename
                            GraphManager::Model>>::Handle>> bag;

                    {
                        auto it = heap.ordered_begin();
                        auto et = heap.ordered_end();

                        for (; it != et && (*it).tn == t; ++it)
                            bag.insert((*it).heapid);
                    }

                    {
                        for (auto & child : last_output_list)
                            bag.insert((reinterpret_cast <Child*>(
                                        child->simulator))->heapid);
                    }

                    last_output_list.clear();

                    for (auto & element : bag) {
                        (*element).transition(t);
                        heap.decrease(element);
                    }

                    tn = heap.top().tn;
                } else {
                    tn = Time::infinity;
                }
            }

            void output(typename Time::type t)
            {
                assert(t == heap.top().tn);
                assert(tn == heap.top().tn);

                last_output_list.clear();

                if (t == tn && not heap.empty()) {
                    std::vector <typename GraphManager::Model> lst;

                    typename Scheduler::ordered_iterator it = heap.ordered_begin();
                    typename Scheduler::ordered_iterator et = heap.ordered_end();
                    typename Scheduler::value_type simulator = *it;

                    do {
                        simulator.output(t);

                        if (!simulator.model->y.is_empty())
                            lst.push_back(simulator.model);

                        ++it;
                    } while (it != et && it->tn == tn);

                    gm.put(lst, last_output_list);
                }
            }
        };

}

#endif
