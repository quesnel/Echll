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

#ifndef ORG_VLEPROJECT_KERNEL_DSDE_DSDE_HPP
#define ORG_VLEPROJECT_KERNEL_DSDE_DSDE_HPP

#include <vle/context.hpp>
#include <vle/common.hpp>
#include <vle/time.hpp>
#include <vle/port.hpp>
#include <vle/heap.hpp>
#include <memory>
#include <set>
#include <thread>

namespace vle {
namespace dsde {

class dsde_internal_error : std::logic_error
{
public:
    dsde_internal_error(const std::string &msg);

    dsde_internal_error(const dsde_internal_error &) = default;

    virtual ~dsde_internal_error() noexcept;
};

template <typename Time>
inline void check_transition_synchronization(typename Time::time_type tl,
        typename Time::time_type time,
        typename Time::time_type tn);

template <typename Time>
inline void check_output_synchronization(typename Time::time_type tn,
        typename Time::time_type time);

template <typename Time, typename InputPort, typename OutputPort,
          typename ChildInputPort, typename ChildOutputPort>
class ComposedModel;

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

    Model(const Context &ctx_)
        : tl(Time::negative_infinity())
        , tn(Time::infinity())
        , parent(nullptr)
        , ctx(ctx_)
    {}

    template <typename InputPortInit, typename OutputPortInit>
    Model(const Context &ctx_,
          const InputPortInit &inputport_init,
          const OutputPortInit &outputport_init)
        : x(inputport_init)
        , y(outputport_init)
        , tl(Time::negative_infinity())
        , tn(Time::infinity())
        , parent(nullptr)
        , ctx(ctx_)
    {}

    virtual ~Model() noexcept {}

    mutable inputport_type x;
    mutable outputport_type y;
    time_type tl, tn;
    void *parent;
    typename HeapType <Time>::handle_type heapid;
    Context ctx;

    inline constexpr const Context &context() const { return ctx; }

    virtual void start(const Common &common, const time_type &time) = 0;
    virtual void transition(const time_type &time) = 0;
    virtual void output(const time_type &time) = 0;
};

template <typename Time, typename InputPort, typename OutputPort,
          typename ChildInputPort, typename ChildOutputPort>
class ComposedModel : public Model <Time, InputPort, OutputPort>
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

    using UpdatedPort = std::set <const child_type*>;
    using Bag = std::set <child_type*>;

    ComposedModel(const ComposedModel &) = default;
    ComposedModel(ComposedModel &&) = default;
    ComposedModel &operator=(const ComposedModel &) = default;
    ComposedModel &operator=(ComposedModel &&) = default;

    ComposedModel(const Context &ctx_)
        : Model <Time, InputPort, OutputPort>(ctx_)
    {}

    template <typename InputPortInit, typename OutputPortInit>
    ComposedModel(const Context &ctx_,
                  const InputPortInit &inputport_init,
                  const OutputPortInit &outputport_init)
        : Model <Time, InputPort, OutputPort>(
            ctx_, inputport_init, outputport_init)
    {}

    UpdatedPort last_output_list;
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

    virtual time_type init(const vle::Common &common,
                           const time_type &time) = 0;
    virtual time_type delta(const time_type &elapsed,
                            const time_type &remaining,
                            const time_type &time) = 0;
    virtual void lambda() const = 0;

    AtomicModel(const Context &ctx_)
        : Model <Time, InputPort, OutputPort>(ctx_)
    {}

    template <typename InputPortInit, typename OutputPortInit>
    AtomicModel(const Context &ctx_,
                const InputPortInit &inputport_init,
                const OutputPortInit &outputport_init)
        : Model <Time, InputPort, OutputPort>(
            ctx_, inputport_init, outputport_init)
    {}

    virtual void start(const vle::Common &common, const time_type &time) override
    {
        parent_type::tl = time;
        parent_type::tn = time + init(common, time);
    }

    virtual void transition(const time_type &time) override
    {
        check_transition_synchronization <Time>(parent_type::tl,
                                                time,
                                                parent_type::tn);

        if (time < parent_type::tn and parent_type::x.empty())
            return;

        parent_type::tn = time + delta(time - parent_type::tl,
                                       parent_type::tn - time,
                                       time);
        parent_type::tl = time;
        parent_type::x.clear();
    }

    virtual void output(const time_type &time) override
    {
        if (time == parent_type::tn)
            lambda();
    }
};

template <typename Time>
struct TransitionPolicyDefault {
    using time_format = Time;
    using time_type = typename Time::time_type;

    TransitionPolicyDefault()
    {}

    TransitionPolicyDefault(unsigned)
    {}

    void resize(unsigned)
    {}

    template <typename Bag>
    void operator()(Bag &bag, const time_type &time,
        HeapType <time_format> &heap)
    {
        for (auto *child : bag) {
            child->transition(time);
            child->x.clear();
            (*child->heapid).tn = child->tn;
            heap.update(child->heapid);
        }
    }
};

template <typename Time>
struct TransitionPolicyThread {
    using time_format = Time;
    using time_type = typename Time::time_type;

    TransitionPolicyThread()
        : pool(std::max(1u, std::thread::hardware_concurrency()))
    {}

    TransitionPolicyThread(unsigned thread_number)
        : pool(thread_number)
    {
        if (thread_number == 0)
            pool.resize(1);
    }

    TransitionPolicyThread(const TransitionPolicyThread &other)
        : pool(other.pool.size())
    {
        if (pool.size() == 0)
            pool.resize(1);
    }

    void resize(unsigned i)
    {
        if (i > 0)
            pool.resize(i);
    }

    template <typename Bag>
    void work(Bag &bag, const time_type &time, const std::size_t idx)
    {
        if (idx >= bag.size())
            return;

        std::size_t current_job_id = idx;
        auto it = bag.begin();
        std::advance(it,
                     boost::numeric_cast<std::ptrdiff_t>(
                         idx));

        for (;;) {
            (*it)->transition(time);
            current_job_id += pool.size();

            if (current_job_id >= bag.size())
                break;

            std::advance(it,
                         boost::numeric_cast<std::ptrdiff_t>(
                             pool.size()));
        }
    }

    template <typename Bag>
    void operator()(Bag &bag, const time_type &time,
                    HeapType <time_format> &heap)
    {
        if (bag.size() == 1) {
            auto *child = (*bag.begin());
            child->transition(time);
            child->x.clear();
            (*child->heapid).tn = child->tn;
            heap.update(child->heapid);
        } else {
            for (auto i = 0ul; i < pool.size(); ++i) {
                pool[i] = std::thread(&TransitionPolicyThread<Time>::work<Bag>,
                                      this, std::ref(bag), time, i);
            }

            for (auto &worker : pool)
                if (worker.get_id() != std::thread::id())
                    worker.join();

            for (auto *child : bag) {
                (*child->heapid).tn = child->tn;
                heap.update(child->heapid);
                child->x.clear();
            }
        }
    }

private:
    std::vector <std::thread> pool;
};

template <typename Time, typename InputPort, typename OutputPort,
          typename ChildInputPort, typename ChildOutputPort,
          typename Policy = TransitionPolicyThread <Time>>
class CoupledModel : public ComposedModel <Time, InputPort, OutputPort,
                                           ChildInputPort, ChildOutputPort>
{
public:
    using parent_type = ComposedModel <Time, InputPort, OutputPort,
                                       ChildInputPort, ChildOutputPort>;
    using child_type = typename parent_type::child_type;
    using time_format = Time;
    using time_type = typename Time::time_type;
    using inputport_type = InputPort;
    using outputport_type = OutputPort;
    using childinputport_type = ChildInputPort;
    using childoutputport_type = ChildOutputPort;
    using transition_policy = Policy;

    using UpdatedPort = typename parent_type::UpdatedPort;
    using Bag = typename parent_type::Bag;
    using children_t = std::vector <child_type*>;

    CoupledModel(const CoupledModel &) = default;
    CoupledModel(CoupledModel &&) = default;
    CoupledModel &operator=(const CoupledModel &) = default;
    CoupledModel &operator=(CoupledModel &&) = default;

    HeapType <Time> heap;
    transition_policy policy;

    /**
     * @brief Get the children of the @e CoupledModel.
     * @details The @e children function is called only once by the simulation
     * layer after the constructor.
     *
     * @return A list of pointer of child models.
     */
    virtual children_t children(const vle::Common &common) = 0;

    /**
     * @brief Move or copy event from output port to input port.
     * @details The post function take into account the move message between
     * models. The post function must take into account the input ports of
     * the coupled model that is not in @e UpdatedPort @e out. Be sure to
     * not push this into the @e UpdatedPort @e in.
     *
     * @param out The children list which emits a output message.
     * @param in The children list which receives the message.
     */
    virtual void post(const UpdatedPort &out, UpdatedPort &in) const = 0;

    CoupledModel(const Context &ctx)
        : parent_type(ctx)
        , policy(ctx->get_thread_number())
    {}

    template <typename InputPortInit, typename OutputPortInit>
    CoupledModel(const Context &ctx_,
                 const InputPortInit &inputport_init,
                 const OutputPortInit &outputport_init)
        : parent_type(ctx_, inputport_init, outputport_init)
        , policy(ctx_->get_thread_number())
    {}

    virtual void start(const vle::Common &common,
        const time_type &time) override
    {
        auto cs = children(common);

        for (auto child : cs) {
            child->parent = this;
            child->start(common, time);
            auto id = heap.emplace(child, child->tn);
            child->heapid = id;
            (*id).heapid = id;
        };

        parent_type::tl = time;
        parent_type::tn = heap.top().tn;
    }

    virtual void transition(const time_type &time) override
    {
        check_transition_synchronization <Time>(parent_type::tl,
                                                time,
                                                parent_type::tn);

        if (time  < parent_type::tn && parent_type::x.empty())
            return;

        // The bag stores models in internal (where time equal child.tn),
        // external  (where child.input port is not empty.
        Bag bag;
        {
            auto it = heap.ordered_begin();
            auto et = heap.ordered_end();

            for (; it != et && (*it).tn == time; ++it)
                bag.insert(reinterpret_cast <child_type*>((*it).element));
        }

        if (not parent_type::x.empty()) {
            post(UpdatedPort(), parent_type::last_output_list);
            parent_type::x.clear();
        }

        for (auto &child : parent_type::last_output_list)
            bag.insert(const_cast <child_type*>(child));

        parent_type::last_output_list.clear();
        policy(bag, time, heap);
        parent_type::tl = time;
        parent_type::tn = heap.top().tn;
    }

    virtual void output(const time_type &time) override
    {
        check_output_synchronization <Time>(heap.top().tn, time);

        if (time == parent_type::tn && not heap.empty()) {
            UpdatedPort lst;
            auto it = heap.ordered_begin();
            auto et = heap.ordered_end();

            assert(it != et);
            assert(boost::numeric_cast<std::size_t>(std::distance(it, et))
                   == heap.size());

            do {
                auto id = (*it).heapid;
                child_type *mdl = reinterpret_cast<child_type*>((*id).element);
                mdl->output(time);

                if (!(mdl->y.empty()))
                    lst.emplace(mdl);

                ++it;
            } while (it != et && it->tn == parent_type::tn);

            post(lst, parent_type::last_output_list);

            // TODO How to make test? If user adds this into the
            // last_output_list, we need to remove it from before clearing
            // output ports. parent_type::last_output_list.erase(this);

            for (auto *child : lst)
                child->y.clear();
        }
    }
};

template <typename Time, typename InputPort, typename OutputPort,
          typename ChildInputPort, typename ChildOutputPort,
          typename Policy = TransitionPolicyThread <Time>>
class Executive : public ComposedModel <Time, InputPort, OutputPort,
                                        ChildInputPort, ChildOutputPort>
{
public:
    using parent_type = ComposedModel <Time, InputPort, OutputPort,
                                       ChildInputPort, ChildOutputPort>;
    using child_type = typename parent_type::child_type;
    using time_format = Time;
    using time_type = typename Time::time_type;
    using inputport_type = InputPort;
    using outputport_type = OutputPort;
    using childinputport_type = ChildInputPort;
    using childoutputport_type = ChildOutputPort;
    using transition_policy = Policy;

    using UpdatedPort = typename parent_type::UpdatedPort;
    using Bag = typename parent_type::Bag;
    using children_t = std::vector <child_type*>;

    Executive(const Executive &) = default;
    Executive(Executive &&) = default;
    Executive &operator=(const Executive &) = default;
    Executive &operator=(Executive &&) = default;

    HeapType <Time> heap;

    time_type chi_tl;
    time_type chi_tn;
    typename HeapType <Time>::handle_type chi_heapid;
    mutable childinputport_type chi_x;
    mutable childoutputport_type chi_y;
    transition_policy policy;
    vle::Common localcommon;

    virtual children_t children(const vle::Common &common) = 0;
    virtual time_type init(const time_type &time) = 0;
    virtual time_type delta(const time_type &elapsed,
                            const time_type &remaining,
                            const time_type &time) = 0;
    virtual void lambda() const = 0;
    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const = 0;

    Executive(const vle::Context &ctx)
        : parent_type(ctx)
        , chi_tl(Time::negative_infinity())
        , chi_tn(Time::infinity())
    {}

    template <typename InputPortInit, typename OutputPortInit,
              typename ChiInputPortInit, typename ChiOutputPortInit>
    Executive(const Context &ctx_,
              const InputPortInit &inputport_init,
              const OutputPortInit &outputport_init,
              const ChiInputPortInit &chiinputport_init,
              const ChiOutputPortInit &chioutputport_init)
        : parent_type(ctx_, inputport_init, outputport_init)
        , chi_tl(Time::negative_infinity())
        , chi_tn(Time::infinity())
        , chi_x(chiinputport_init)
        , chi_y(chioutputport_init)
    {}

    void insert(child_type *mdl)
    {
        mdl->parent = this;
        mdl->start(localcommon, Executive::chi_tl);
        auto id = heap.emplace(mdl, mdl->tn);
        mdl->heapid = id;
        (*id).heapid = id;
    }

    void erase(child_type *mdl)
    {
        parent_type::last_output_list.erase(mdl);
        heap.erase(mdl->heapid);
        mdl->parent = nullptr;
    }

    virtual void start(const vle::Common &common,
        const time_type &time) override
    {
        localcommon = common;
        chi_tl = time;
        chi_tn = time + init(time);
        auto id = heap.emplace(this, chi_tn);
        chi_heapid = id;
        (*id).heapid = id;
        auto cs = children(localcommon);

        for (auto child : cs) {
            child->parent = this;
            child->start(localcommon, time);
            auto child_id = heap.emplace(child, child->tn);
            child->heapid = child_id;
            (*child_id).heapid = child_id;
        }

        parent_type::tl = time;
        parent_type::tn = heap.top().tn;
    }

    virtual void transition(const time_type &time) override
    {
        check_transition_synchronization <Time>(parent_type::tl,
                                                time,
                                                parent_type::tn);

        if (time < parent_type::tn && parent_type::x.empty())
            return;

        Bag bag;
        bool have_chi = false;
        {
            auto it = heap.ordered_begin();
            auto et = heap.ordered_end();

            for (; it != et && (*it).tn == time; ++it) {
                if ((*it).element == this)
                    have_chi = true;
                else
                    bag.insert(reinterpret_cast<child_type*>((*it).element));
            }
        }

        if (not parent_type::x.empty()) {
            post({this}, parent_type::last_output_list);
            parent_type::x.clear();
        }

        for (auto &child : parent_type::last_output_list) {
            if (child == this)
                have_chi = true;
            else
                bag.insert(const_cast <child_type*>(child));
        }

        parent_type::last_output_list.clear();
        policy(bag, time, heap);

        if (have_chi) {
            time_type e = time - chi_tl;
            time_type r = chi_tn - time;
            chi_tl = time;
            chi_tn = time + delta(e, r, time);
            parent_type::x.clear();
            (*chi_heapid).tn = chi_tn;
            heap.update(chi_heapid);
        }

        parent_type::tl = time;
        parent_type::tn = heap.top().tn;
    }

    virtual void output(const time_type &time) override
    {
        check_output_synchronization <Time>(heap.top().tn, time);

        if (time == parent_type::tn && not heap.empty()) {
            UpdatedPort lst;
            auto it = heap.ordered_begin();
            auto et = heap.ordered_end();

            do {
                auto id = (*it).heapid;
                child_type *mdl = reinterpret_cast<child_type*>((*id).element);

                if (mdl == this) {
                    lambda();
                } else {
                    mdl->output(time);
                }

                if (!(mdl->y.empty()))
                    lst.emplace(mdl);

                ++it;
            } while (it != et && it->tn == parent_type::tn);

            post(lst, parent_type::last_output_list);

            // If user adds this into the last_output_list, we need to remove
            // it from before clearing output ports.
            parent_type::last_output_list.erase(this);

            for (auto *child : lst)
                child->y.clear();
        }
    }
};

template <typename Time>
class Engine
{
public:
    using time_format = Time;
    using time_type = typename Time::time_type;
    vle::CommonPtr common;

    Engine()
        : common(std::make_shared <Common>())
    {}

    Engine(vle::CommonPtr common_)
        : common(common_)
    {}

    template <typename InputPort, typename OutputPort>
    time_type pre(Model <Time, InputPort, OutputPort> &model,
                  const time_type &time)
    {
        model.start(*common, time);
        return model.tn;
    }

    template <typename InputPort, typename OutputPort>
    time_type run(Model <Time, InputPort, OutputPort> &model,
                  const time_type &time)
    {
        model.output(time);
        model.transition(time);
        model.x.clear();
        return model.tn;
    }

    template <typename InputPort, typename OutputPort>
    void post(Model <Time, InputPort, OutputPort> &model,
              const time_type &time)
    {
        (void)model;
        (void)time;
    }
};

}
}

#include <vle/dsde/detail/dsde-implementation.hpp>

#endif
