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

namespace vle {
namespace devs {

devs_internal_error::devs_internal_error(const std::string &msg)
    : std::logic_error(msg)
{
}

devs_internal_error::~devs_internal_error() noexcept
{
}

template <typename Time, typename InputPort, typename OutputPort>
Model <Time, InputPort, OutputPort>::Model(const Context &ctx_)
    : ctx(ctx_)
    , tl(Time::negative_infinity())
    , tn(Time::infinity())
    , e(Time::null())
    , parent(nullptr)
{
}

template <typename Time, typename InputPort, typename OutputPort>
template <typename InputPortInit, typename OutputPortInit>
Model <Time, InputPort, OutputPort>::Model(const Context &ctx_,
        const InputPortInit &inputport_init,
        const OutputPortInit &outputport_init)
    : ctx(ctx_)
    , x(inputport_init)
    , y(outputport_init)
    , tl(Time::negative_infinity())
    , tn(Time::infinity())
    , e(Time::null())
    , parent(nullptr)
{
}

template <typename Time, typename InputPort, typename OutputPort>
Model <Time, InputPort, OutputPort>::~Model()
{
}

template <typename Time, typename InputPort, typename OutputPort>
constexpr const Context &Model <Time, InputPort, OutputPort>::context() const
{
    return ctx;
}

template <typename Time, typename InputPort, typename OutputPort>
AtomicModel <Time, InputPort, OutputPort>::AtomicModel(const Context &ctx)
    : Model <Time, InputPort, OutputPort>(ctx)
{
}

template <typename Time, typename InputPort, typename OutputPort>
template <typename InputPortInit, typename OutputPortInit>
AtomicModel <Time, InputPort, OutputPort>::AtomicModel(const Context &ctx,
        const InputPortInit &inputport_init,
        const OutputPortInit &outputport_init)
    : Model <Time, InputPort, OutputPort>(ctx, inputport_init, outputport_init)
{
}

template <typename Time, typename InputPort, typename OutputPort>
AtomicModel <Time, InputPort, OutputPort>::~AtomicModel()
{
}

template <typename Time, typename InputPort, typename OutputPort>
void AtomicModel <Time, InputPort, OutputPort>::i_msg(const time_type &time)
{
    Model <Time, InputPort, OutputPort>::tl = time;
    Model <Time, InputPort, OutputPort>::tn = Model
            <Time, InputPort, OutputPort>::tl + ta();
}

template <typename Time, typename InputPort, typename OutputPort>
void AtomicModel <Time, InputPort, OutputPort>::s_msg(const time_type &time)
{
#ifndef VLE_OPTIMIZE

    if (time != Model <Time, InputPort, OutputPort>::tn)
        throw devs_internal_error("Synchronization error");

#endif
    lambda();
    Model <Time, InputPort, OutputPort>::parent->y_msg(*this, time);
    internal();
    Model <Time, InputPort, OutputPort>::tl = time;
    Model <Time, InputPort, OutputPort>::tn = time + ta();
}

template <typename Time, typename InputPort, typename OutputPort>
void AtomicModel <Time, InputPort, OutputPort>::x_msg(const time_type &time)
{
#ifndef VLE_OPTIMIZE

    if (!((Model <Time, InputPort, OutputPort>::tl <= time) &&
          (time <= Model <Time, InputPort, OutputPort>::tn)))
        throw devs_internal_error("Synchronization error");

#endif
    Model <Time, InputPort, OutputPort>::e = time - Model
            <Time, InputPort, OutputPort>::tl;
    external(Model <Time, InputPort, OutputPort>::e);
    Model <Time, InputPort, OutputPort>::tl = time;
    Model <Time, InputPort, OutputPort>::tn = time + ta();
}

template <typename Time, typename InputPort, typename OutputPort>
void AtomicModel <Time, InputPort, OutputPort>::y_msg(Model
        <Time, InputPort, OutputPort> &model,
        const time_type &time)
{
    (void)model;
    (void)time;
}

template <typename Time, typename InputPort, typename OutputPort,
          typename ChildInputPort, typename ChildOutputPort>
CoupledModel <Time, InputPort, OutputPort, ChildInputPort,
              ChildOutputPort>::CoupledModel(const Context &ctx)
    : Model <Time, InputPort, OutputPort>(ctx)
{
}

template <typename Time, typename InputPort, typename OutputPort,
          typename ChildInputPort, typename ChildOutputPort>
template <typename InputPortInit, typename OutputPortInit>
CoupledModel <Time, InputPort, OutputPort, ChildInputPort,
             ChildOutputPort>::CoupledModel(const Context &ctx,
        const InputPortInit &inputport_init,
        const OutputPortInit &outputport_init)
    : Model <Time, InputPort, OutputPort>(ctx, inputport_init, outputport_init)
{
}

template <typename Time, typename InputPort, typename OutputPort,
          typename ChildInputPort, typename ChildOutputPort>
CoupledModel <Time, InputPort, OutputPort, ChildInputPort,
             ChildOutputPort>::~CoupledModel()
{
}

template <typename Time, typename InputPort, typename OutputPort,
          typename ChildInputPort, typename ChildOutputPort>
void CoupledModel <Time, InputPort, OutputPort, ChildInputPort,
                   ChildOutputPort>::i_msg(const time_type &time)
{
    auto cs = children();
    std::for_each(cs.begin(), cs.end(),
    [ = ](Model <Time, InputPort, OutputPort> *child) {
        child->parent = this;
        child->i_msg(time);
        auto id = heap.emplace(child, child->tn);
        child->heapid = id;
        (*id).heapid = id;
    });
    Model <Time, InputPort, OutputPort>::tl = time;
    Model <Time, InputPort, OutputPort>::tn = heap.top().tn;
}

template <typename Time, typename InputPort, typename OutputPort,
          typename ChildInputPort, typename ChildOutputPort>
void CoupledModel <Time, InputPort, OutputPort, ChildInputPort,
                   ChildOutputPort>::s_msg(const time_type &time)
{
#ifndef VLE_OPTIMIZE

    if (time != Model <Time, InputPort, OutputPort>::tn)
        throw devs_internal_error("Synchronization error");

#endif
    typename HeapElement <Time>::handle_t imm;
    {
        std::vector <typename HeapElement <Time>::handle_t> heap_element_imm;
        std::vector <Model <Time, InputPort, OutputPort>*> model_element_imm;
        auto it = heap.ordered_begin();
        auto et = heap.ordered_end();

        for (; it != et and (*it).tn == time; ++it) {
            heap_element_imm.push_back((*it).heapid);
            model_element_imm.push_back(reinterpret_cast
                                        <Model <Time, InputPort, OutputPort>*>((*it).element));
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
    reinterpret_cast <Model <Time, InputPort, OutputPort>*>((*imm).element)->s_msg(
        time);
    (*imm).tn = reinterpret_cast <Model <Time, InputPort, OutputPort>*>((
                    *imm).element)->tn;
    heap.update((*imm).heapid);
    Model <Time, InputPort, OutputPort>::tl = time;
    Model <Time, InputPort, OutputPort>::tn = heap.top().tn;
}

template <typename Time, typename InputPort, typename OutputPort,
          typename ChildInputPort, typename ChildOutputPort>
void CoupledModel <Time, InputPort, OutputPort, ChildInputPort,
                   ChildOutputPort>::x_msg(const time_type &time)
{
#ifndef VLE_OPTIMIZE

    if (!((Model <Time, InputPort, OutputPort>::tl <= time) &&
          (time <= Model <Time, InputPort, OutputPort>::tn)))
        throw devs_internal_error("Synchronization error");

#endif
    UpdatedPort receivers;
    post(nullptr, receivers);
    std::for_each(receivers.begin(), receivers.end(),
    [ = ](const Model <Time, InputPort, OutputPort> *model) {
        Model <Time, InputPort, OutputPort> *mdl = const_cast
                <Model <Time, InputPort, OutputPort>*>(model);
        mdl->x_msg(time);
        (*mdl->heapid).tn = mdl->tn;
        heap.update(mdl->heapid);
        mdl->x.clear();
    });
    Model <Time, InputPort, OutputPort>::x.clear();
    Model <Time, InputPort, OutputPort>::tl = time;
    Model <Time, InputPort, OutputPort>::tn = heap.top().tn;
}

template <typename Time, typename InputPort, typename OutputPort,
          typename ChildInputPort, typename ChildOutputPort>
void CoupledModel <Time, InputPort, OutputPort, ChildInputPort,
                   ChildOutputPort>::y_msg(Model
                                           <Time, InputPort, OutputPort> &model,
        const time_type &time)
{
    // Check external coupling to see if there is an external output event
    // or internal coupling.
    UpdatedPort receivers;
    post(&model, receivers);
    std::for_each(receivers.begin(), receivers.end(),
    [ = ](const Model <Time, InputPort, OutputPort> *r) {
        Model <Time, InputPort, OutputPort> *mdl = const_cast
                <Model <Time, InputPort, OutputPort>*>(r);

        if (mdl == this) {
            Model <Time, InputPort, OutputPort>::parent->y_msg(*this, time);
        } else {
            mdl->x_msg(time);
            (*mdl->heapid).tn = mdl->tn;
            heap.update(mdl->heapid);
            mdl->x.clear();
        }
    });
    model.y.clear();
}

template <typename Time>
template <typename InputPort, typename OutputPort>
typename Time::time_type
Engine <Time>::pre(Model <Time, InputPort, OutputPort> &model,
                   const time_type &time)
{
    model.i_msg(time);
    return model.tn;
}

template <typename Time>
template <typename InputPort, typename OutputPort>
typename Time::time_type
Engine <Time>::run(Model <Time, InputPort, OutputPort> &model,
                   const time_type &time)
{
    model.s_msg(time);
    return model.tn;
}

template <typename Time>
template <typename InputPort, typename OutputPort>
void
Engine <Time>::post(Model <Time, InputPort, OutputPort> &model,
                    const time_type &time)
{
    (void)model;
    (void)time;
}

}
}

#endif
