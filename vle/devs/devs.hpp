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

#ifndef __VLE_KERNEL_DEVS_HPP__
#define __VLE_KERNEL_DEVS_HPP__

#include <vle/context.hpp>
#include <vle/time.hpp>
#include <vle/port.hpp>
#include <vle/heap.hpp>
#include <memory>
#include <set>

namespace vle { namespace devs {

struct devs_internal_error : std::logic_error
{
    explicit devs_internal_error(const std::string& msg);

    virtual ~devs_internal_error();
};

template <typename Time, typename Value>
struct Model
{
    typedef Time time_format;
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    Model(const Context &ctx);

    Model(const Context &ctx,
          std::size_t input_port_number,
          std::size_t output_port_number);

    Model(const Context &ctx,
          std::initializer_list <std::string> lst_x,
          std::initializer_list <std::string> lst_y);

    virtual ~Model();

    Context ctx;
    mutable vle::PortList <Value> x, y;
    time_type tl, tn, e;
    Model *parent;
    typename HeapType <Time, Value>::handle_type heapid;

    inline constexpr const Context& context() const;

    virtual void i_msg(const time_type& time) = 0;
    virtual void s_msg(const time_type& time) = 0;
    virtual void x_msg(const time_type& time) = 0;
    virtual void y_msg(Model <Time, Value>& model,
                       const time_type& time) = 0;
};

template <typename Time, typename Value>
struct AtomicModel : Model <Time, Value>
{
    typedef Time time_format;
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    virtual time_type ta() const = 0;
    virtual void lambda() const = 0;
    virtual void internal() = 0;
    virtual void external(const time_type& time) = 0;

    AtomicModel(const Context &ctx);

    AtomicModel(const Context &ctx,
                std::size_t input_port_number,
                std::size_t output_port_number);

    AtomicModel(const Context &ctx,
                std::initializer_list <std::string> lst_x,
                std::initializer_list <std::string> lst_y);

    virtual ~AtomicModel();

    virtual void i_msg(const time_type& time) override final;

    virtual void s_msg(const time_type& time) override final;

    virtual void x_msg(const time_type& time) override final;

    virtual void y_msg(Model <Time, Value>& model,
                       const time_type& time) override final;
};

template <typename Time, typename Value>
using UpdatedPort = std::set <const Model <Time, Value>*>;

template <typename Time, typename Value>
struct CoupledModel : Model <Time, Value>
{
    typedef Time time_format;
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    HeapType <Time, Value> heap;
    typedef std::vector <Model <Time, Value>*> children_t;

    virtual children_t children() = 0;
    virtual void post(const Model <Time, Value> &out, UpdatedPort <Time, Value> &in) const = 0;
    virtual std::size_t select(const std::vector <Model <Time, Value>*>& models) const = 0;

    CoupledModel(const Context &ctx);

    CoupledModel(const Context &ctx,
                 std::size_t input_port_number,
                 std::size_t output_port_number);

    CoupledModel(const Context &ctx,
                 std::initializer_list <std::string> lst_x,
                 std::initializer_list <std::string> lst_y);

    virtual ~CoupledModel();

    virtual void i_msg(const time_type& time) override final;

    virtual void s_msg(const time_type& time) override final;

    virtual void x_msg(const time_type& time) override final;

    virtual void y_msg(Model <Time, Value>& model,
                       const time_type& time) override final;
};

template <typename Time, typename Value>
struct Engine
{
    typedef Time time_format;
    typedef typename Time::time_type time_type;
    typedef Value value_type;
    typedef Model <Time, Value> model_type;

    time_type pre(model_type& model, const time_type& time);

    time_type run(model_type& model, const time_type& time);

    void post(model_type& model, const time_type& time);
};

}}

#include <vle/devs/detail/devs-implementation.hpp>

#endif
