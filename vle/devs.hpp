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

namespace vle {

struct devs_internal_error : std::logic_error
{
    explicit devs_internal_error(const std::string& msg)
        : std::logic_error(msg)
    {}
};

namespace devs {

template <typename Time, typename Value>
struct Model
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    Model(const Context &ctx)
        : ctx(ctx)
          , tl(-Time::infinity)
          , tn(Time::infinity)
          , parent(nullptr)
    {}

    Model(const Context &ctx,
          std::initializer_list <std::string> lst_x,
          std::initializer_list <std::string> lst_y)
        : ctx(ctx)
          , x(lst_x)
          , y(lst_y)
          , tl(-Time::infinity)
          , tn(Time::infinity)
          , e(0)
          , parent(nullptr)
    {}

    virtual ~Model()
    {}

    Context ctx;
    mutable vle::PortList <Value> x, y;
    time_type tl, tn, e;
    Model *parent;
    typename HeapType <Time, Value>::handle_type heapid;

    inline constexpr const Context& context() const { return ctx; }

    virtual void i_msg(const time_type& time) = 0;
    virtual void s_msg(const time_type& time) = 0;
    virtual void x_msg(const time_type& time) = 0;
    virtual void y_msg(Model <Time, Value>& model,
                       const time_type& time) = 0;
};

template <typename Time, typename Value>
struct AtomicModel : Model <Time, Value>
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    virtual time_type ta() const = 0;
    virtual void lambda() const = 0;
    virtual void internal() const = 0;
    virtual void external(const time_type& time) const = 0;

    AtomicModel(const Context &ctx)
        : Model <Time, Value>(ctx)
    {}

    AtomicModel(const Context &ctx,
                std::initializer_list <std::string> lst_x,
                std::initializer_list <std::string> lst_y)
        : Model <Time, Value>(ctx, lst_x, lst_y)
    {}

    virtual ~AtomicModel()
    {}

    virtual void i_msg(const time_type& time) override final
    {
        Model <Time, Value>::tl = time - Model <Time, Value>::e;
        Model <Time, Value>::tn = Model <Time, Value>::tl + ta();
    }

    virtual void s_msg(const time_type& time) override final
    {
#ifndef VLE_OPTIMIZE
        if (time != Model <Time, Value>::tn)
            throw devs_internal_error("Synchronization error");
#endif

        lambda();
        Model <Time, Value>::parent->y_msg(*this, time);
        internal();
        Model <Time, Value>::tl = time;
        Model <Time, Value>::tn = time + ta();
    }

    virtual void x_msg(const time_type& time) override final
    {
#ifndef VLE_OPTIMIZE
        if (!((Model <Time, Value>::tl <= time) &&
              (time <= Model <Time, Value>::tn)))
            throw devs_internal_error("Synchronization error");
#endif

        Model <Time, Value>::e = time - Model <Time, Value>::tl;
        external(Model <Time, Value>::e);
        Model <Time, Value>::tl = time;
        Model <Time, Value>::tn = time + ta();
    }

    virtual void y_msg(Model <Time, Value>& model,
                       const time_type& time) override final
    {
        (void)model;
        (void)time;
    }
};

template <typename Time, typename Value>
using UpdatedPort = std::set <const Model <Time, Value>*>;

template <typename Time, typename Value>
struct CoupledModel : Model <Time, Value>
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    HeapType <Time, Value> heap;
    typedef std::vector <Model <Time, Value>*> children_t;

    virtual children_t children() = 0;
    virtual void post(const Model <Time, Value> &out,
                      UpdatedPort <Time, Value> &in) const = 0;

    CoupledModel(const Context &ctx)
        : Model <Time, Value>(ctx)
    {}

    CoupledModel(const Context &ctx,
                 std::initializer_list <std::string> lst_x,
                 std::initializer_list <std::string> lst_y)
        : Model <Time, Value>(ctx, lst_x, lst_y)
    {}

    virtual ~CoupledModel()
    {}

    virtual void i_msg(const time_type& time) override final
    {
        auto cs = children();
        std::for_each(cs.begin(), cs.end(),
                      [=](Model <Time, Value> *child)
                      {
                          child->parent = this;
                          child->i(time);

                          auto id = heap.emplace(child, child->tn);
                          child->heapid = id;
                          (*id).heapid = id;
                      });

        Model <Time, Value>::tl = time;
        Model <Time, Value>::tn = heap.top().tn;
    }

    virtual void s_msg(const time_type& time) override final
    {
#ifndef VLE_OPTIMIZE
        if (time != Model <Time, Value>::tn)
            throw devs_internal_error("Synchronization error");
#endif
        auto d = heap.top();
        reinterpret_cast <Model <Time, Value>*>(d.element)->s_msg(time);
        d.tn = reinterpret_cast <Model <Time, Value>*>(d.element)->tn;
        heap.update(d.heapid);

        Model <Time, Value>::tl = time;
        Model <Time, Value>::tn = heap.top().tn;
    }

    virtual void x_msg(const time_type& time) override final
    {
#ifndef VLE_OPTIMIZE
        if (!((Model <Time, Value>::tl <= time) &&
              (time <= Model <Time, Value>::tn)))
            throw devs_internal_error("Synchronization error");
#endif
        UpdatedPort <Time, Value> receivers;
        post(*this, receivers);

        std::for_each(receivers.begin(), receivers.end(),
                      [=](const Model <Time, Value> *model)
                      {
                          model->x_msg(time);

                          (*model->heapid)->tn = model->tn;
                          heap.update(model->heapid);
                      });

        Model <Time, Value>::tl = time;
        Model <Time, Value>::tn = heap.top().tn;
    }

    virtual void y_msg(Model <Time, Value>& model,
                       const time_type& time) override final
    {
        UpdatedPort <Time, Value> receivers;
        post(model, receivers);

        std::for_each(receivers.begin(), receivers.end(),
                      [=](const Model <Time, Value> *model)
                      {
                          if (model == this) {
                              Model <Time, Value>::parent->y_msg(*this,
                                                                 time);
                          } else {
                              model->x_msg(time);

                              (*model->heapid)->tn = model->tn;
                              heap.update(model->heapid);
                          }
                      });
    }
};

template <typename Time, typename Value>
struct Engine
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;
    typedef Model <Time, Value> model_type;

    time_type pre(model_type& model, const time_type& time)
    {
        model.i_msg(time);

        return model.tn;
    }

    time_type run(model_type& model, const time_type& time)
    {
        model.s_msg(time);

        return model.tn;
    }

    void post(model_type& model, const time_type& time)
    {
        (void)model;
        (void)time;
    }
};
};

};

#endif
