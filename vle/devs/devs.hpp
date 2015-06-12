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

#ifndef ORG_VLEPROJECT_KERNEL_DEVS_HPP
#define ORG_VLEPROJECT_KERNEL_DEVS_HPP

#include <vle/context.hpp>
#include <vle/time.hpp>
#include <vle/port.hpp>
#include <vle/heap.hpp>
#include <memory>
#include <set>

namespace vle {
namespace devs {

class devs_internal_error : public std::logic_error
{
public:
    devs_internal_error(const std::string &msg);

    devs_internal_error(const devs_internal_error &) = default;

    virtual ~devs_internal_error() noexcept;
};

template <typename Time, typename InputPort, typename OutputPort>
class Model
{
public:
    using time_format = Time;
    using time_type = typename Time::time_type;
    using inputport_type = InputPort;
    using outputport_type = OutputPort;

    Model(const Model &) = default;
    Model(Model &&) = default;
    Model &operator=(const Model &) = default;
    Model &operator=(Model &&) = default;

    Model(const Context &ctx);

    template <typename InputPortInit, typename OutputPortInit>
    Model(const Context &ctx,
          const InputPortInit &inputport_init,
          const OutputPortInit &outputport_init);

    virtual ~Model();

    Context ctx;
    mutable inputport_type x;
    mutable outputport_type y;
    time_type tl;
    time_type tn;
    time_type e;
    Model *parent;
    typename HeapType <Time>::handle_type heapid;

    inline constexpr const Context &context() const;

    virtual void i_msg(const time_type &time) = 0;
    virtual void s_msg(const time_type &time) = 0;
    virtual void x_msg(const time_type &time) = 0;
    virtual void y_msg(Model <Time, InputPort, OutputPort> &model,
                       const time_type &time) = 0;
};

template <typename Time, typename InputPort, typename OutputPort>
class AtomicModel : public Model <Time, InputPort, OutputPort>
{
public:
    using parent_type = Model <Time, InputPort, OutputPort>;
    using time_format = Time;
    using time_type = typename Time::time_type;
    using inputport_type = InputPort;
    using outputport_type = OutputPort;

    AtomicModel(const AtomicModel &) = default;
    AtomicModel(AtomicModel &&) = default;
    AtomicModel &operator=(const AtomicModel &) = default;
    AtomicModel &operator=(AtomicModel &&) = default;

    virtual time_type ta() const = 0;
    virtual void lambda() const = 0;
    virtual void internal() = 0;
    virtual void external(const time_type &time) = 0;

    AtomicModel(const Context &ctx);

    template <typename InputPortInit, typename OutputPortInit>
    AtomicModel(const Context &ctx,
                const InputPortInit &inputport_init,
                const OutputPortInit &outputport_init);

    virtual ~AtomicModel();

    virtual void i_msg(const time_type &time) override final;

    virtual void s_msg(const time_type &time) override final;

    virtual void x_msg(const time_type &time) override final;

    virtual void y_msg(Model <Time, InputPort, OutputPort> &model,
                       const time_type &time) override final;
};

template <typename Time, typename InputPort, typename OutputPort,
          typename ChildInputPort, typename ChildOutputPort>
class CoupledModel : public Model <Time, InputPort, OutputPort>
{
public:
    using parent_type = Model <Time, InputPort, OutputPort>;
    using child_type = Model <Time, ChildInputPort, ChildOutputPort>;
    using time_format = Time;
    using time_type = typename Time::time_type;
    using inputport_type = InputPort;
    using outputport_type = OutputPort;
    using childinputport_type = ChildInputPort;
    using childoutputport_type = ChildOutputPort;
    using children_t = std::vector <child_type*>;

    using UpdatedPort = std::set <const child_type*>;

    HeapType <Time> heap;

    virtual children_t children() = 0;
    virtual void post(const child_type *out, UpdatedPort &in) const = 0;
    virtual std::size_t select(const std::vector <child_type*>& models) const = 0;

    CoupledModel(const Context &ctx);

    template <typename InputPortInit, typename OutputPortInit>
    CoupledModel(const Context &ctx,
                 const InputPortInit &inputport_init,
                 const OutputPortInit &outputport_init);

    virtual ~CoupledModel();

    virtual void i_msg(const time_type &time) override final;

    virtual void s_msg(const time_type &time) override final;

    virtual void x_msg(const time_type &time) override final;

    virtual void y_msg(Model <Time, InputPort, OutputPort> &model,
                       const time_type &time) override final;
};

template <typename Time>
class Engine
{
public:
    using time_format = Time;
    using time_type = typename Time::time_type;

    template <typename InputPort, typename OutputPort>
        time_type pre(Model <Time, InputPort, OutputPort> &model,
                      const time_type &time);

    template <typename InputPort, typename OutputPort>
        time_type run(Model <Time, InputPort, OutputPort> &model,
                      const time_type &time);

    template <typename InputPort, typename OutputPort>
        void post(Model <Time, InputPort, OutputPort> &model,
                  const time_type &time);
};

}
}

#include <vle/devs/detail/devs-implementation.hpp>

#endif
