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

#ifndef ORG_VLEPROJECT_KERNEL_DSDE_MPI_TIMEWARP_HPP
#define ORG_VLEPROJECT_KERNEL_DSDE_MPI_TIMEWARP_HPP

// #include <vle/common.hpp>
// #include <vle/time.hpp>
// #include <vle/port.hpp>
// #include <vle/utils.hpp>
// #include <vle/heap.hpp>
// #include <vle/dsde/dsde.hpp>
// #include <boost/mpi/environment.hpp>
// #include <boost/mpi/communicator.hpp>
// #include <stdexcept>
// #include <numeric>

// namespace vle {
// namespace dsde {

// /*
//  *  == Time warp or optimistic parallelization ==
//  *
//  *
//  *  We need to use several root coordinator to remove the current next
//  *  time (tn) ride. So a root coordinator manages one or more children
//  *  with or without connection between them and with different parent.
//  *
//  *  An RC in optimistic mode needs:
//  *
//  *
//  *  Classic mode:
//  *
//  *          C1 - {MC11... MC1n}
//  *        /
//  *     RC - C2 - {MC21... MC2n}
//  *        \
//  *          C3 - {MC31... MC3n}
//  *
//  *  Optimistic mode:
//  *
//  *  - Example 1 (with 3 processors):
//  *
//  *     RC1 - C1 - {MC11... MC1n}
//  *     RC2 - C2 - {MC21... MC2n}
//  *     RC3 - C3 - {MC31... MC3n}
//  *
//  *  - Example 2 (with 4 processors):
//  *
//  *     RC1 - C1 - {MC11... MC1n}
//  *     RC2 - C2 - {MC21... MC25}
//  *     RC3 - C3 - {MC31... MC3n}
//  *     RC4 - {MC25..MCn}
//  */

// enum TimeWarpTag {
//     anti_message_tag,
//     proxy_send_ended_simulation_tag,
//     proxy_send_start_tag,
//     proxy_recv_tn_tag,
//     proxy_send_transition_tag,
//     proxy_send_output_tag,
//     proxy_recv_output_tag
// };

// /*
//  * @e DistantChild: represents communication between a coupled model
//  * and a distant child. In optimistic mode, the send and receive
//  * commands between parent and child are asynchronous.
//  */
// template <typename Time, typename Value>
// struct DistantChild : Model <Time, Value> {
//     typedef typename Time::type time_type;
//     typedef Value value_type;

//     boost::mpi::environment *environment;
//     boost::mpi::communicator *communicator;
//     std::uintmax_t id;
//     int rank; // rank of distant @e TimeWarpSynchroniser.

//     DistantChild()
//         : Model <Time, Value>()
//         , environment(nullptr)
//         , communicator(nullptr)
//         , rank(-1)
//     {}

//     DistantChild(std::initializer_list <std::string> lst_x,
//                  std::initializer_list <std::string> lst_y)
//         : Model <Time, Value>(lst_x, lst_y)
//         , environment(nullptr)
//         , communicator(nullptr)
//         , rank(-1)
//     {}

//     virtual ~DistantChild()
//     {
//     }

//     virtual void start(const Common &common, const time_type &time) override
//     {
//         (void) common;
//         Model <Time, Value>::tl = time;
//         Model <Time, Value>::tn = Time::infinity;
//         Model <Time, Value>::x.clear();
//     }

//     virtual void transition(const time_type &time)
//     {
//         if (!Model <Time, Value>::x.empty()) {
//             communicator->isend(rank, proxy_send_transition_tag,
//                                 time);
//             communicator->isend(rank, proxy_send_transition_tag,
//                                 Model <Time, Value>::x);
//         }

//         // TODO necessary to update tl and tn ?
//         Model <Time, Value>::tl = time;
//         Model <Time, Value>::tn = Time::infinity;
//         Model <Time, Value>::x.clear();
//     }

//     virtual void output(const time_type &time)
//     {
//         (void) time;
//     }
// };

// template <typename Time, typename T>
// struct ordered_data {
//     typedef typename Time::type time_type;

//     struct State {
//         time_type time;
//         T state;
//     };

//     struct StateCompare {
//         bool operator()(const State &lhs, const State &rhs) const
//         {
//             return lhs.time < rhs.time;
//         }
//     };

//     std::set <State, StateCompare> state;

//     void store(time_type time, const T &t)
//     {
//         state.emplace(time, t);
//     }

//     void cleanup(time_type time)
//     {
//         auto it = state.lower_bound(time);

//         if (it != state.end())
//             state.erase(state.begin(), it);
//     }
// };

// template <typename Time, typename Value>
// struct TimeWarpChild {
//     typedef typename Time::type time_type;
//     typedef Value value_type;
//     typedef Model <Time, Value> model_type;

//     ordered_data <Time, model_type> models;
//     ordered_data <Time, PortList <Value>> ins;
//     ordered_data <Time, PortList <Value>> outs;

//     boost::mpi::environment *environment;
//     boost::mpi::communicator *communicator;

//     float processor;
//     model_type model;
//     int rank;

//     TimeWarpChild()
//         : environment(nullptr)
//         , communicator(nullptr)
//         , processor(.5f)
//         , model(nullptr)
//         , rank(-1)
//     {}

//     void start(const Common &common, const time_type &time)
//     {
//         (void) common;
//         (void) time;
//     }

//     void transition(const time_type &time)
//     {
//         (void) time;
//     }

//     void output(const time_type &time)
//     {
//         (void) time;
//     }

//     void roolback(const time_type &time)
//     {
//         if (model.tn >= time) {
//             models.cleanup(time);
//             auto antimessage = outs.from(time);

//             for (auto it : antimessage)
//                 communicator->send(rank, anti_message_tag,
//                                    it->time);

//             outs.cleanup(time);
//         }
//     }

//     time_type min_tn() const
//     {
//         if (!model.state.empty())
//             return std::min(*model.state.begin());

//         return Time::positive;
//     }
// };

// template <typename Time, typename Value>
// struct TimeWarpSynchroniser {
//     typedef typename Time::type time_type;
//     typedef Value value_type;

//     Context ctx;
//     std::vector <TimeWarpChild <Time, Value>> children;

//     boost::mpi::environment *environment;
//     boost::mpi::communicator *communicator;

//     time_type minimal_time;

//     std::uintmax_t id;
//     int rank;

//     TimeWarpSynchroniser(const Context &ctx)
//         : ctx(ctx)
//         , environment(nullptr)
//         , communicator(nullptr)
//         , rank(-1)
//     {
//     }

//     constexpr bool is_initialized() const
//     {
//         return environment && communicator;
//     }

//     ~TimeWarpSynchroniser()
//     {}

//     std::vector <int>  initialize_child_processor()
//     {
//         float sum = std::accumulate(children.cbegin(), children.cend(), 0.0f,
//         [](const auto & child) {
//             return child.processor;
//         });
//         std::vector <float> reduce(children.size(), 0.0f);
//         std::transform(children.cbegin(), children.cend(),
//                        reduce.begin(),
//         [&sum](const auto & child) {
//             return child.processor / sum;
//         });
//         float min = std::min_element(children.begin(), children.end(),
//         [](const auto & rhs, const auto & lhs) {
//             return rhs.processor  < lhs.processor;
//         }) / sum;
//         std::vector <int> ret;
//         ret.reserve(children.size() * 100u);

//         for (std::size_t i = 0, e = children.size(); i != e; ++i) {
//             children[i].processor = children[i].processor / min;

//             if (children[i].processor < 1)
//                 children[i].processor = 1.0f;
//             else if (children[i].processor > children[i].size() * 100.0f)
//                 children[i].processor = children[i].size() * 100.0f;

//             ret.insert(ret.end(),
//                        static_cast <std::size_t>(children[i].processor),
//                        i);
//             ctx->dbg() << "TimeWarpSynchronizer: " << i << "priority: "
//                        << children.processor << '\n';
//         }

//         std::random_shuffle(ret.begin(), ret.end());
//         return std::move(ret);
//     }

//     void run_global_virtual_time()
//     {
//         if (!is_initialized())
//             return;

//         if (communicator->rank() == 0) {
//             // TODO sleep ?
//             //boost::mpi::all_reduce(*world,
//             //minimal_time,
//             //boost::mpi::minimum <time_type>());
//             //boost::mpi::broadcast(*world,
//             //minimal_time,
//             //0);
//         }
//     }

//     void run_simulation()
//     {
//         if (!is_initialized())
//             return;

//         time_type time;
//         auto process = initialize_child_processor();

//         for (auto it : children)
//             it->start(vle::Common(), time);

//         for (;;) {
//             for (std::size_t i = 0, e = process.size(); i != e; ++i) {
//                 children[process[i]]->output(time);
//                 children[process[i]]->transition(time);
//             }
//         }
//     }
// };

// }
// }

#endif
