/*
 * Copyright (C) 2015  INRA
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

#ifndef ORG_VLEPROJECT_KERNEL_DEVS_DETAIL_DEVS_IMPLEMENTATION_HPP
#define ORG_VLEPROJECT_KERNEL_DEVS_DETAIL_DEVS_IMPLEMENTATION_HPP

namespace vle { namespace devs {

devs_internal_error::devs_internal_error(const std::string& msg)
    : std::logic_error(msg)
{
}

devs_internal_error::~devs_internal_error() noexcept
{
}

template <typename Time, typename Value>
Model <Time, Value>::Model(const Context &ctx_)
    : ctx(ctx_)
    , tl(Time::negative_infinity())
    , tn(Time::infinity())
    , e(Time::null())
    , parent(nullptr)
{
}

template <typename Time, typename Value>
Model <Time, Value>::Model(const Context &ctx_,
                           std::size_t input_port_number,
                           std::size_t output_port_number)
    : ctx(ctx_)
    , x(input_port_number)
    , y(output_port_number)
    , tl(Time::negative_infinity())
    , tn(Time::infinity())
    , e(Time::null())
    , parent(nullptr)
{
}

template <typename Time, typename Value>
Model <Time, Value>::Model(const Context &ctx_,
                           std::initializer_list <std::string> lst_x,
                           std::initializer_list <std::string> lst_y)
    : ctx(ctx_)
    , x(lst_x)
    , y(lst_y)
    , tl(Time::negative_infinity())
    , tn(Time::infinity())
    , e(Time::null())
    , parent(nullptr)
{
}

template <typename Time, typename Value>
Model <Time, Value>::~Model()
{
}

template <typename Time, typename Value>
constexpr const Context& Model <Time, Value>::context() const
{
    return ctx;
}

template <typename Time, typename Value>
AtomicModel <Time, Value>::AtomicModel(const Context &ctx)
    : Model <Time, Value>(ctx)
{
}

template <typename Time, typename Value>
AtomicModel <Time, Value>::AtomicModel(const Context &ctx,
                                       std::size_t input_port_number,
                                       std::size_t output_port_number)
    : Model <Time, Value>(ctx, input_port_number, output_port_number)
{
}

template <typename Time, typename Value>
AtomicModel <Time, Value>::AtomicModel(const Context &ctx,
                                       std::initializer_list <std::string> lst_x,
                                       std::initializer_list <std::string> lst_y)
    : Model <Time, Value>(ctx, lst_x, lst_y)
{
}

template <typename Time, typename Value>
AtomicModel <Time, Value>::~AtomicModel()
{
}

template <typename Time, typename Value>
void AtomicModel <Time, Value>::i_msg(const time_type& time)
{
    Model <Time, Value>::tl = time;
    Model <Time, Value>::tn = Model <Time, Value>::tl + ta();
}

template <typename Time, typename Value>
void AtomicModel <Time, Value>::s_msg(const time_type& time)
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

template <typename Time, typename Value>
void AtomicModel <Time, Value>::x_msg(const time_type& time)
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

template <typename Time, typename Value>
void AtomicModel <Time, Value>::y_msg(Model <Time, Value>& model,
                                      const time_type& time)
{
    (void)model;
    (void)time;
}

template <typename Time, typename Value>
CoupledModel <Time, Value>::CoupledModel(const Context &ctx)
    : Model <Time, Value>(ctx)
{
}

template <typename Time, typename Value>
CoupledModel <Time, Value>::CoupledModel(const Context &ctx,
                                         std::size_t input_port_number,
                                         std::size_t output_port_number)
    : Model <Time, Value>(ctx, input_port_number, output_port_number)
{
}

template <typename Time, typename Value>
CoupledModel <Time, Value>::CoupledModel(const Context &ctx,
                                         std::initializer_list <std::string> lst_x,
                                         std::initializer_list <std::string> lst_y)
    : Model <Time, Value>(ctx, lst_x, lst_y)
{
}

template <typename Time, typename Value>
CoupledModel <Time, Value>::~CoupledModel()
{
}

template <typename Time, typename Value>
void CoupledModel <Time, Value>::i_msg(const time_type& time)
{
    auto cs = children();
    std::for_each(cs.begin(), cs.end(),
                  [=](Model <Time, Value> *child)
                  {
                      child->parent = this;
                      child->i_msg(time);

                      auto id = heap.emplace(child, child->tn);
                      child->heapid = id;
                      (*id).heapid = id;
                  });

    Model <Time, Value>::tl = time;
    Model <Time, Value>::tn = heap.top().tn;
}

template <typename Time, typename Value>
void CoupledModel <Time, Value>::s_msg(const time_type& time)
{
#ifndef VLE_OPTIMIZE
    if (time != Model <Time, Value>::tn)
        throw devs_internal_error("Synchronization error");
#endif

    typename HeapElement <Time, Value>::handle_t imm;

    {
        std::vector <typename HeapElement <Time, Value>::handle_t> heap_element_imm;
        std::vector <Model <Time, Value>*> model_element_imm;

        auto it = heap.ordered_begin();
        auto et = heap.ordered_end();

        for (; it != et and (*it).tn == time; ++it) {
            heap_element_imm.push_back((*it).heapid);
            model_element_imm.push_back(reinterpret_cast <Model <Time, Value>*>((*it).element));
        }

        if (model_element_imm.empty())
            throw devs_internal_error("empty scheduller");

        if (model_element_imm.size() == 1u)
            imm = heap_element_imm.front();
        else {
            std::size_t i = select(model_element_imm);
            if (i > model_element_imm.size())
                throw devs_internal_error("select error");

            imm = heap_element_imm[i];
        }
    }

    reinterpret_cast <Model <Time, Value>*>((*imm).element)->s_msg(time);
    (*imm).tn = reinterpret_cast <Model <Time, Value>*>((*imm).element)->tn;
    heap.update((*imm).heapid);

    Model <Time, Value>::tl = time;
    Model <Time, Value>::tn = heap.top().tn;
}

template <typename Time, typename Value>
void CoupledModel <Time, Value>::x_msg(const time_type& time)
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
                      Model <Time, Value>* mdl = const_cast <Model <Time, Value>*>(model);
                      mdl->x_msg(time);

                      (*mdl->heapid).tn = mdl->tn;
                      heap.update(mdl->heapid);

                      mdl->x.clear();
                  });

    Model <Time, Value>::x.clear();

    Model <Time, Value>::tl = time;
    Model <Time, Value>::tn = heap.top().tn;
}

template <typename Time, typename Value>
void CoupledModel <Time, Value>::y_msg(Model <Time, Value>& model,
                                       const time_type& time)
{
    // Check external coupling to see if there is an external output event
    // or internal coupling.

    UpdatedPort <Time, Value> receivers;
    post(model, receivers);

    std::for_each(receivers.begin(), receivers.end(),
                  [=](const Model <Time, Value> *r)
                  {
                      Model <Time, Value>* mdl = const_cast <Model <Time, Value>*>(r);

                      if (mdl == this) {
                          Model <Time, Value>::parent->y_msg(*this, time);
                      } else {
                          mdl->x_msg(time);

                          (*mdl->heapid).tn = mdl->tn;
                          heap.update(mdl->heapid);

                          mdl->x.clear();
                      }
                  });

    model.y.clear();
}

template <typename Time, typename Value>
typename Time::time_type
Engine <Time, Value>::pre(model_type& model, const time_type& time)
{
    model.i_msg(time);

    return model.tn;
}

template <typename Time, typename Value>
typename Time::time_type
Engine <Time, Value>::run(model_type& model, const time_type& time)
{
    model.s_msg(time);

    return model.tn;
}

template <typename Time, typename Value>
void Engine <Time, Value>::post(model_type& model, const time_type& time)
{
    (void)model;
    (void)time;
}

}}

#endif
