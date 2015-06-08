/*
 * Copyright (C) 2014 INRA
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

#include <vle/vle.hpp>
#include <vle/dsde/generic.hpp>
#include <sstream>
#include <numeric>

// static const std::string root_string("coupled_s0\n"
//                                      "coupled_s1\n"
//                                      "#\n"
//                                      "1 2 3 3\n"
//                                      "1 2 4 4\n"
//                                      "1 2 5 5\n"
//                                      "2 1 0 0\n"
//                                      "2 1 1 1\n"
//                                      "2 1 2 2\n"
//                                      "2 1 6 6\n"
//                                      "2 1 7 7\n");

// static const std::string s0_string("normal\n"
//                                    "top\n"
//                                    "top\n"
//                                    "normal\n"
//                                    "normal\n"
//                                    "#\n"
//                                    "1 5 0 0\n"
//                                    "2 5 0 0\n"
//                                    "3 4 0 0\n"
//                                    "4 5 0 0\n"
//                                    "0 1 0 0\n"
//                                    "0 4 1 0\n"
//                                    "0 4 2 0\n"
//                                    "0 5 2 0\n"
//                                    "0 4 6 0\n"
//                                    "0 5 6 0\n"
//                                    "0 4 7 0\n"
//                                    "0 5 7 0\n"
//                                    "1 0 0 3\n"
//                                    "2 0 0 4\n"
//                                    "3 0 0 5\n");

// static const std::string s1_string("top\n"
//                                    "normal\n"
//                                    "normal\n"
//                                    "normal\n"
//                                    "normal\n"
//                                    "#\n"
//                                    "1 3 0 0\n"
//                                    "1 2 0 0\n"
//                                    "2 4 0 0\n"
//                                    "2 5 0 0\n"
//                                    "3 4 0 0\n"
//                                    "4 5 0 0\n"
//                                    "0 4 3 0\n"
//                                    "0 5 4 0\n"
//                                    "0 5 5 0\n"
//                                    "1 0 0 0\n"
//                                    "2 0 0 1\n"
//                                    "3 0 0 2\n"
//                                    "4 0 0 6\n"
//                                    "5 0 0 7\n");

// void Assert(bool is_true)
// {
//     if (!is_true)
//         throw std::logic_error("Assertion failed");
// }

// typedef vle::DoubleTime MyTime;
// typedef vle::PortList <double > MyPort;
// typedef vle::dsde::Engine <MyTime> MyDSDE;

// using Factory = vle::dsde::Factory <MyTime>;
// using GenericCoupledModelThread = vle::dsde::GenericCoupledModel
//                                   <MyTime, MyPort, MyPort, MyPort, MyPort,
//                                   vle::dsde::TransitionPolicyDefault <MyTime>>;
// using AtomicModel = vle::dsde::AtomicModel <MyTime, MyPort, MyPort>;
// using CoupledModelThread = vle::dsde::CoupledModel
//                            <MyTime, MyPort, MyPort, MyPort, MyPort,
//                            vle::dsde::TransitionPolicyThread <MyTime>>;
// using CoupledModelMono = vle::dsde::CoupledModel
//                          <MyTime, MyPort, MyPort, MyPort, MyPort,
//                          vle::dsde::TransitionPolicyDefault <MyTime>>;
// using ExecutiveThread = vle::dsde::Executive
//                         <MyTime, MyPort, MyPort, MyPort, MyPort,
//                         vle::dsde::TransitionPolicyThread <MyTime>>;
// using ExecutiveMono = vle::dsde::Executive
//                       <MyTime, MyPort, MyPort, MyPort, MyPort,
//                       vle::dsde::TransitionPolicyDefault <MyTime>>;


// struct TopPixel : AtomicModel {
//     std::string  m_name;
//     double value;

//     TopPixel(const vle::Context &ctx)
//         : AtomicModel(ctx, 0u, 1u)
//         , value(0)
//     {}

//     virtual ~TopPixel()
//     {}

//     virtual double init(const vle::Common &common, const double &) override final
//     {
//         m_name = boost::any_cast <std::string>(common.at("name"));
//         return 0.0;
//     }

//     virtual double delta(const double &, const double &,
//                          const double &) override final
//     {
//         return 1.0;
//     }

//     virtual void lambda() const override final
//     {
//         y[0] = {value};
//     }
// };

// struct NormalPixel : AtomicModel {
//     enum Phase { WAIT, SEND };

//     std::string  m_name;
//     double       m_value;
//     double       m_current_time;
//     double       m_last_time;
//     unsigned int m_neighbour_number;
//     unsigned int m_received;
//     unsigned int m_last_received;
//     Phase        m_phase;

//     NormalPixel(const vle::Context &ctx)
//         : AtomicModel(ctx, 1u, 1u)
//         , m_value(0.0)
//         , m_current_time(MyTime::negative_infinity())
//         , m_last_time(MyTime::infinity())
//         , m_neighbour_number(0)
//         , m_received(0)
//         , m_last_received(0)
//         , m_phase(WAIT)
//     {}

//     virtual ~NormalPixel()
//     {}

//     virtual double init(const vle::Common &common,
//                         const double &t) override final
//     {
//         m_value = 0.0;
//         m_current_time = t;
//         m_last_time = MyTime::negative_infinity();
//         m_name = boost::any_cast <std::string>(common.at("name"));
//         m_neighbour_number = boost::any_cast <unsigned int>
//                              (common.at("neighbour_number"));
//         m_received = 0;
//         m_last_received = 0;
//         m_phase = WAIT;
//         return MyTime::infinity();
//     }

//     virtual double delta(const double &e, const double &remaining,
//                          const double &time) override final
//     {
//         (void)e;
//         (void)remaining;
//         m_current_time = time;

//         if (x.empty())
//             dint(m_current_time);
//         else
//             dext(m_current_time);

//         if (m_phase == WAIT)
//             return MyTime::infinity();

//         return 0.0;
//     }

//     void dint(const double &time)
//     {
//         if (m_phase == SEND) {
//             m_phase = WAIT;
//             m_last_received += m_received;
//             m_received = 0;
//             m_last_time = time;
//         }
//     }

//     void dext(const double &time)
//     {
//         assert(m_last_time != time);
//         m_received += x[0].size();

//         if (m_received == m_neighbour_number) {
//             m_phase = SEND;
//         }
//     }

//     virtual void lambda() const override final
//     {
//         if (m_phase == SEND)
//             y[0] = {m_value};
//     }
// };

// struct MyGenericCoupledModel_s0 : GenericCoupledModelThread {
//     MyGenericCoupledModel_s0(const vle::Context &ctx)
//         : GenericCoupledModelThread(ctx,
//     {"0", "1", "2", "6", "7"},
//     {"3", "4", "5"})
//     {}

//     virtual ~MyGenericCoupledModel_s0()
//     {}

//     virtual vle::Common update_common(
//         const vle::Common &common,
//         const MyGenericCoupledModel_s0::vertices &v,
//         const MyGenericCoupledModel_s0::edges &e,
//         int child)
//     {
//         auto mdl = v[boost::numeric_cast<std::size_t>(child)].get();
//         typedef MyGenericCoupledModel_s0::edges::value_type edge_type;
//         unsigned int nb = std::accumulate(
//                               e.cbegin(), e.cend(),
//         0u, [&mdl](unsigned int x, const edge_type & edge) {
//             return edge.second.first == mdl ? x + 1u : x;
//         });
//         vle::Common ret(common);
//         ret["name"] = std::string("s0-") + std::to_string(child);
//         ret["neighbour_number"] = nb;
//         return std::move(ret);
//     }
// };

// struct MyGenericCoupledModel_s1 : GenericCoupledModelThread {
//     MyGenericCoupledModel_s1(const vle::Context &ctx)
//         : GenericCoupledModelThread(ctx,
//     {"3", "4", "5"},
//     {"0", "1", "2", "6", "7"})
//     {}

//     virtual ~MyGenericCoupledModel_s1()
//     {}

//     virtual vle::Common update_common(
//         const vle::Common &common,
//         const MyGenericCoupledModel_s1::vertices &v,
//         const MyGenericCoupledModel_s1::edges &e,
//         int child)
//     {
//         auto mdl = v[boost::numeric_cast<std::size_t>(child)].get();
//         typedef MyGenericCoupledModel_s1::edges::value_type edge_type;
//         unsigned int nb = std::accumulate(
//                               e.cbegin(), e.cend(),
//         0u, [&mdl](unsigned int x, const edge_type & edge) {
//             return edge.second.first == mdl ? x + 1 : x;
//         });
//         vle::Common ret(common);
//         ret["name"] = std::string("s1-") + std::to_string(child);
//         ret["neighbour_number"] = nb;
//         return std::move(ret);
//     }
// };

// struct RootNetwork : GenericCoupledModelThread {
//     RootNetwork(const vle::Context &ctx)
//         : GenericCoupledModelThread(ctx, {}, {})
//     {}

//     virtual ~RootNetwork()
//     {}

//     virtual vle::Common update_common(
//         const vle::Common &common,
//         const MyGenericCoupledModel_s1::vertices &v,
//         const MyGenericCoupledModel_s1::edges &e,
//         int child)
//     {
//         (void) v;
//         (void) e;
//         vle::Common ret(common);

//         if (child == 0)
//             ret.at("tgf-stringsource") = s0_string;
//         else if (child == 1)
//             ret.at("tgf-stringsource") = s1_string;
//         else
//             throw std::invalid_argument("Too many child!");

//         return std::move(ret);
//     }
// };

// typedef vle::dsde::Factory <MyTime, MyValue> MainFactory;

// void check_f1(const vle::Context &ctx, const vle::CommonPtr &common)
// {
//     common->at("tgf-stringsource") = root_string;
//     MyDSDE dsde_engine(common);
//     RootNetwork root(ctx);
//     vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, root);
//     double final_date = sim.run(0.0, 1.0);
//     Assert(final_date == 1.0);
//     MyGenericCoupledModel_s0 *model_s0 =
//         dynamic_cast <MyGenericCoupledModel_s0 *>(root.m_children[0].get());
//     Assert(model_s0);
//     NormalPixel *model_s0_4 =
//         dynamic_cast <NormalPixel *>(model_s0->m_children[4].get());
//     Assert(model_s0_4);
//     Assert(model_s0_4->m_last_received == 6);
// }

// void check_f2(const vle::Context &ctx, const vle::CommonPtr &common)
// {
//     common->at("tgf-stringsource") = root_string;
//     MyDSDE dsde_engine(common);
//     RootNetwork root(ctx);
//     vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine, root);
//     double final_date = sim.run(0.0, 10.0);
//     Assert(final_date == 10.0);
//     MyGenericCoupledModel_s0 *model_s0 =
//         dynamic_cast <MyGenericCoupledModel_s0 *>(root.m_children[0].get());
//     Assert(model_s0);
//     NormalPixel *model_s0_4 =
//         dynamic_cast <NormalPixel *>(model_s0->m_children[4].get());
//     Assert(model_s0_4);
//     Assert(model_s0_4->m_last_received == (6 * final_date));
// }

int main()
{
//     vle::Context ctx = std::make_shared <vle::ContextImpl>();
//     std::shared_ptr <MainFactory> factory = std::make_shared <MainFactory>();
//     typedef MainFactory::modelptr modelptr;
//     factory->functions.emplace("normal",
//     [&ctx]() -> modelptr {
//         return modelptr(new NormalPixel(ctx));
//     });
//     factory->functions.emplace("top",
//     [&ctx]() -> modelptr {
//         return modelptr(new TopPixel(ctx));
//     });
//     factory->functions.emplace("coupled_s0",
//     [&ctx]() -> modelptr {
//         return modelptr(
//             new MyGenericCoupledModel_s0(ctx));
//     });
//     factory->functions.emplace("coupled_s1",
//     [&ctx]() -> modelptr {
//         return modelptr(
//             new MyGenericCoupledModel_s1(ctx));
//     });
//     vle::CommonPtr common = std::make_shared <vle::Common>();
//     int source = 1;
//     int format = 1;
//     common->emplace("tgf-factory", factory);
//     common->emplace("tgf-source", source);
//     common->emplace("tgf-format", format);
//     common->emplace("tgf-stringsource", root_string);
//     check_f1(ctx, common);
//     check_f2(ctx, common);
//     return EXIT_SUCCESS;
}
