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
#include <vle/generic.hpp>
#include <vle/dbg.hpp>
#include <sstream>
#include <numeric>

static const std::string root_string("coupled_s0\n"
                                     "coupled_s1\n"
                                     "#\n"
                                     "1 2 3 3\n"
                                     "1 2 4 4\n"
                                     "1 2 5 5\n"
                                     "2 1 0 0\n"
                                     "2 1 1 1\n"
                                     "2 1 2 2\n"
                                     "2 1 6 6\n"
                                     "2 1 7 7\n");

static const std::string s0_string("normal\n"
                                   "top\n"
                                   "top\n"
                                   "normal\n"
                                   "normal\n"
                                   "#\n"
                                   "1 5 0 0\n"
                                   "2 5 0 0\n"
                                   "3 4 0 0\n"
                                   "4 5 0 0\n"
                                   "0 1 0 0\n"
                                   "0 4 1 0\n"
                                   "0 4 2 0\n"
                                   "0 5 2 0\n"
                                   "0 4 6 0\n"
                                   "0 5 6 0\n"
                                   "0 4 7 0\n"
                                   "0 5 7 0\n"
                                   "1 0 0 3\n"
                                   "2 0 0 4\n"
                                   "3 0 0 5\n");

static const std::string s1_string("top\n"
                                   "normal\n"
                                   "normal\n"
                                   "normal\n"
                                   "normal\n"
                                   "#\n"
                                   "1 3 0 0\n"
                                   "1 2 0 0\n"
                                   "2 4 0 0\n"
                                   "2 5 0 0\n"
                                   "3 4 0 0\n"
                                   "4 5 0 0\n"
                                   "0 4 3 0\n"
                                   "0 5 4 0\n"
                                   "0 5 5 0\n"
                                   "1 0 0 0\n"
                                   "2 0 0 1\n"
                                   "3 0 0 2\n"
                                   "4 0 0 6\n"
                                   "5 0 0 7\n");


template <typename T>
struct Infinity
{
    static constexpr T negative = -std::numeric_limits<T>::infinity();
    static constexpr T positive = std::numeric_limits<T>::infinity();
};

typedef vle::Time <double, Infinity<double>> MyTime;
typedef double MyValue;
typedef vle::dsde::Engine <MyTime, MyValue> MyDSDE;

using UpdatedPort = vle::dsde::UpdatedPort <MyTime, MyValue>;
using AtomicModel = vle::dsde::AtomicModel <MyTime, MyValue>;
using Factory = vle::dsde::Factory <MyTime, MyValue>;
using GenericCoupledModelThread = vle::dsde::GenericCoupledModel <MyTime, MyValue,
      vle::dsde::TransitionPolicyDefault <MyTime, MyValue>>;
using CoupledModelThread = vle::dsde::CoupledModel <MyTime, MyValue,
      vle::dsde::TransitionPolicyThread <MyTime, MyValue>>;
using CoupledModelMono = vle::dsde::CoupledModel <MyTime, MyValue,
      vle::dsde::TransitionPolicyDefault <MyTime, MyValue>>;
using ExecutiveThread = vle::dsde::Executive <MyTime, MyValue,
      vle::dsde::TransitionPolicyThread <MyTime, MyValue>>;
using ExecutiveMono = vle::dsde::Executive <MyTime, MyValue,
      vle::dsde::TransitionPolicyDefault <MyTime, MyValue>>;

struct TopPixel : AtomicModel
{
    std::string  m_name;
    double value;

    TopPixel()
        : AtomicModel({}, {"0"})
        , value(0)
    {}

    virtual ~TopPixel()
    {}

    virtual double init(const vle::Common& common, const double&) override final
    {
        vle::debugf("TopPixel %s: init", m_name.c_str());

        m_name = boost::any_cast <std::string>(common.at("name"));

        return 0.0;
    }

    virtual double delta(const double&) override final
    {
        vle::debugf("TopPixel %s: delta", m_name.c_str());

        return 1.0;
    }

    virtual void lambda() const override final
    {
        vle::debugf("TopPixel %s: lambda", m_name.c_str());

        y[0] = {value};
    }
};

struct NormalPixel : AtomicModel
{
    enum Phase { WAIT, SEND };

    std::string  m_name;
    double       m_value;
    double       m_current_time;
    double       m_last_time;
    unsigned int m_neighbour_number;
    unsigned int m_received;
    unsigned int m_last_received;
    Phase        m_phase;

    NormalPixel()
        : AtomicModel({"0"}, {"0"})
        , m_value(0.0)
        , m_current_time(Infinity <double>::negative)
        , m_last_time(Infinity <double>::negative)
        , m_neighbour_number(0)
        , m_received(0)
        , m_last_received(0)
        , m_phase(WAIT)
    {}

    virtual ~NormalPixel()
    {}

    virtual double init(const vle::Common& common,
                        const double& t) override final
    {
        vle::debugf("NormalPixel %s: init", m_name.c_str());

        m_value = 0.0;
        m_current_time = t;
        m_last_time = Infinity <double>::negative;
        m_name = boost::any_cast <std::string>(common.at("name"));
        m_neighbour_number = boost::any_cast <unsigned int>(common.at("neighbour_number"));
        m_received = 0;
        m_last_received = 0;
        m_phase = WAIT;

        return Infinity <double>::positive;
    }

    virtual double delta(const double& time) override final
    {
        m_current_time += time;
        vle::debugf("NormalPixel %s: delta (time=%f)", m_name.c_str(), time);

        if (x.empty())
            dint(m_current_time);
        else
            dext(m_current_time);

        if (m_phase == WAIT)
            return Infinity <double>::positive;

        return 0.0;
    }

    void dint(const double& time)
    {
        vle::debugf("%s dint: m_received=%u size=%lu",
                    m_name.c_str(), m_received,
                    x[0].size());

        if (m_phase == SEND) {
            m_phase = WAIT;
            m_last_received += m_received;
            m_received = 0;
            m_last_time = time;
        }
    }

    void dext(const double& time)
    {
        if (m_last_time == time)
            vle::debugf("oups !");

        vle::debugf("%s dext: m_received=%u size=%lu (needs=%u)",
                    m_name.c_str(), m_received,
                    x[0].size(),
                    m_neighbour_number);

        m_received += x[0].size();

        if (m_received == m_neighbour_number) {
            m_phase = SEND;
        }
    }

    virtual void lambda() const override final
    {
        vle::debugf("NormalPixel %s: lambda", m_name.c_str());

        if (m_phase == SEND)
            y[0] = {m_value};
    }
};

struct MyGenericCoupledModel_s0 : GenericCoupledModelThread
{
    MyGenericCoupledModel_s0()
        : GenericCoupledModelThread({"0", "1", "2", "6", "7"}, {"3", "4", "5"})
    {}

    virtual ~MyGenericCoupledModel_s0()
    {}

    virtual vle::Common update_common(
        const vle::Common& common,
        const MyGenericCoupledModel_s0::vertices& v,
        const MyGenericCoupledModel_s0::edges& e,
        int child)
    {
        auto mdl = v[child].get();

        unsigned int nb = std::accumulate(
            e.cbegin(), e.cend(),
            0u, [&mdl](unsigned int x, const auto& edge)
            {
                return edge.second.first == mdl ? x + 1u : x;
            });

        vle::Common ret(common);

        ret["name"] = std::string("s0-") + std::to_string(child);
        ret["neighbour_number"] = nb;

        vle::debugf("model %d, neighbour_number %" PRIuMAX,
                    child, (std::uintmax_t)nb);

        return std::move(ret);
    }
};

struct MyGenericCoupledModel_s1 : GenericCoupledModelThread
{
    MyGenericCoupledModel_s1()
        : GenericCoupledModelThread({"3", "4", "5"}, {"0", "1", "2", "6", "7"})
    {}

    virtual ~MyGenericCoupledModel_s1()
    {}

    virtual vle::Common update_common(
        const vle::Common& common,
        const MyGenericCoupledModel_s1::vertices& v,
        const MyGenericCoupledModel_s1::edges& e,
        int child)
    {
        auto mdl = v[child].get();

        unsigned int nb = std::accumulate(
            e.cbegin(), e.cend(),
            0u, [&mdl](unsigned int x, const auto& edge)
            {
                return edge.second.first == mdl ? x + 1 : x;
            });

        vle::Common ret(common);

        ret["name"] = std::string("s1-") + std::to_string(child);
        ret["neighbour_number"] = nb;

        vle::debugf("model %d, neighbour_number %" PRIuMAX,
                    child, (std::uintmax_t)nb);

        return std::move(ret);
    }
};

struct RootNetwork : GenericCoupledModelThread
{
    RootNetwork()
        : GenericCoupledModelThread({}, {})
    {}

    virtual ~RootNetwork()
    {}

    virtual vle::Common update_common(
        const vle::Common& common,
        const MyGenericCoupledModel_s1::vertices& v,
        const MyGenericCoupledModel_s1::edges& e,
        int child)
    {
        (void) v;
        (void) e;

        vle::Common ret(common);
        if (child == 0)
            ret.at("tgf-stringsource") = s0_string;
        else if (child == 1)
            ret.at("tgf-stringsource") = s1_string;
        else
            throw std::invalid_argument("Too many child!");

        return std::move(ret);
    }
};

typedef vle::dsde::Factory <MyTime, MyValue> MainFactory;

void check_f1(vle::CommonPtr common)
{
    common->at("tgf-stringsource") = root_string;

    MyDSDE dsde_engine(common);
    RootNetwork root;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, root);

    double final_date = sim.run(0.0, 1.0);
    assert(final_date == 1.0);

    MyGenericCoupledModel_s0* model_s0 =
        dynamic_cast <MyGenericCoupledModel_s0*>(root.m_children[0].get());

    assert(model_s0);

    NormalPixel* model_s0_4 =
        dynamic_cast <NormalPixel*>(model_s0->m_children[4].get());

    assert(model_s0_4);
    assert(model_s0_4->m_last_received == 6);
}

void check_f2(vle::CommonPtr common)
{
    common->at("tgf-stringsource") = root_string;

    MyDSDE dsde_engine(common);
    RootNetwork root;
    vle::SimulationDbg <MyDSDE> sim(dsde_engine, root);

    double final_date = sim.run(0.0, 10.0);
    assert(final_date == 10.0);

    MyGenericCoupledModel_s0* model_s0 =
        dynamic_cast <MyGenericCoupledModel_s0*>(root.m_children[0].get());

    assert(model_s0);

    NormalPixel* model_s0_4 =
        dynamic_cast <NormalPixel*>(model_s0->m_children[4].get());

    assert(model_s0_4);
    assert(model_s0_4->m_last_received == (6 * final_date));
}

int main()
{
    std::shared_ptr <MainFactory> factory = std::make_shared <MainFactory>();

    typedef MainFactory::modelptr modelptr;

    factory->functions.emplace("normal",
                               []() -> modelptr
                               {
                                   return modelptr(new NormalPixel);
                               });

    factory->functions.emplace("top",
                               []() -> modelptr
                               {
                                   return modelptr(new TopPixel);
                               });

    factory->functions.emplace("coupled_s0",
                               []() -> modelptr
                               {
                                   return modelptr(new MyGenericCoupledModel_s0);
                               });

    factory->functions.emplace("coupled_s1",
                               []() -> modelptr
                               {
                                   return modelptr(new MyGenericCoupledModel_s1);
                               });

    vle::CommonPtr common = std::make_shared <vle::Common>();
    int source = 1;
    int format = 1;
    common->emplace("tgf-factory", factory);
    common->emplace("tgf-source", source);
    common->emplace("tgf-format", format);
    common->emplace("tgf-stringsource", root_string);

    check_f1(common);
    check_f2(common);

    return EXIT_SUCCESS;
}
