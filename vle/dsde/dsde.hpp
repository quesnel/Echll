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

#ifndef __VLE_KERNEL_DSDE_DSDE_HPP__
#define __VLE_KERNEL_DSDE_DSDE_HPP__

#include <vle/context.hpp>
#include <vle/common.hpp>
#include <vle/time.hpp>
#include <vle/port.hpp>
#include <vle/heap.hpp>
#include <memory>
#include <set>
#include <thread>

namespace vle {

struct dsde_internal_error : std::logic_error
{
    explicit dsde_internal_error(const std::string& msg)
        : std::logic_error(msg)
    {}
};

namespace dsde {

template <typename Time>
inline void check_transition_synchronization(typename Time::time_type tl,
                                             typename Time::time_type time,
                                             typename Time::time_type tn);

template <typename Time>
inline void check_output_synchronization(typename Time::time_type tn,
                                         typename Time::time_type time);

template <typename Time, typename Value>
struct ComposedModel;

template <typename Time, typename Value>
struct Model
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    Model(const Context &ctx)
        : tl(Time::negative_infinity())
        , tn(Time::infinity())
        , parent(nullptr)
        , ctx(ctx)
    {}

    Model(const Context &ctx, std::size_t input_size, std::size_t output_size)
        : x(input_size)
        , y(output_size)
        , tl(Time::negative_infinity())
        , tn(Time::infinity())
        , parent(nullptr)
        , ctx(ctx)
    {}

    Model(const Context &ctx,
          std::initializer_list <std::string> lst_x,
          std::initializer_list <std::string> lst_y)
        : x(lst_x)
        , y(lst_y)
        , tl(Time::negative_infinity())
        , tn(Time::infinity())
        , parent(nullptr)
        , ctx(ctx)
    {}

    virtual ~Model()
    {}

    mutable vle::PortList <Value> x, y;
    time_type tl, tn;
    ComposedModel <Time, Value> *parent;
    typename HeapType <Time, Value>::handle_type heapid;
    Context ctx;

    inline constexpr const Context& context() const { return ctx; }

    virtual void start(const Common& common, const time_type& time) = 0;
    virtual void transition(const time_type& time) = 0;
    virtual void output(const time_type& time) = 0;
};

template <typename Time, typename Value>
using Bag = std::set <Model <Time, Value>*>;

template <typename Time, typename Value>
using UpdatedPort = std::set <const Model <Time, Value>*>;

template <typename Time, typename Value>
struct ComposedModel : Model <Time, Value>
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    ComposedModel(const Context &ctx)
        : Model <Time, Value>(ctx)
    {}

    ComposedModel(const Context& ctx, std::size_t input_size, std::size_t output_size)
        : Model <Time, Value>(ctx, input_size, output_size)
    {}

    ComposedModel(const Context &ctx,
                  std::initializer_list <std::string> lst_x,
                  std::initializer_list <std::string> lst_y)
        : Model <Time, Value>(ctx, lst_x, lst_y)
    {}

    virtual ~ComposedModel()
    {}

    virtual void post(const UpdatedPort <Time, Value> &out,
                      UpdatedPort <Time, Value> &in) const = 0;

    UpdatedPort <Time, Value> last_output_list;
};

template <typename Time, typename Value>
struct AtomicModel : Model <Time, Value>
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    virtual time_type init(const vle::Common& common,
                           const time_type& time) = 0;
    virtual time_type delta(const time_type &elapsed,
                            const time_type &remaining,
                            const time_type &time) = 0;
    virtual void lambda() const = 0;

    AtomicModel(const Context& ctx)
        : Model <Time, Value>(ctx)
    {}

    AtomicModel(const Context& ctx, std::size_t input_size, std::size_t output_size)
        : Model <Time, Value>(ctx, input_size, output_size)
    {}

    AtomicModel(const Context& ctx,
                std::initializer_list <std::string> lst_x,
                std::initializer_list <std::string> lst_y)
        : Model <Time, Value>(ctx, lst_x, lst_y)
    {}

    virtual ~AtomicModel()
    {}

    virtual void start(const vle::Common& common, const time_type& time) override
    {
        Model <Time, Value>::tl = time;
        Model <Time, Value>::tn = time + init(common, time);
    }

    virtual void transition(const time_type& time) override
    {
        check_transition_synchronization <Time>(Model <Time, Value>::tl,
                                                time,
                                                Model <Time, Value>::tn);

        if (time < Model <Time, Value>::tn and Model <Time, Value>::x.empty())
            return;

        Model <Time, Value>::tn = time + delta(time - Model <Time, Value>::tl,
                                               Model <Time, Value>::tn - time,
                                               time);
        Model <Time, Value>::tl = time;
        Model <Time, Value>::x.clear();
    }

    virtual void output(const time_type& time) override
    {
        if (time == Model <Time, Value>::tn)
            lambda();
    }
};

template <typename Time, typename Value>
struct TransitionPolicyDefault
{
    typedef typename Time::time_type time_type;

    TransitionPolicyDefault()
    {}

    TransitionPolicyDefault(unsigned)
    {}

    void resize(unsigned)
    {}

    void operator()(Bag <Time, Value>& bag, const time_type& time,
                    HeapType <Time, Value> &heap)
    {
        for (auto *child: bag) {
            child->transition(time);
            child->x.clear();

            (*child->heapid).tn = child->tn;
            heap.update(child->heapid);
        }
    }
};

template <typename Time, typename Value>
struct TransitionPolicyThread
{
    typedef typename Time::time_type time_type;

    TransitionPolicyThread()
        : pool(std::max(1u, std::thread::hardware_concurrency()))
    {}

    TransitionPolicyThread(unsigned thread_number)
        : pool(thread_number)
    {
        if (thread_number == 0)
            pool.resize(1);
    }

    TransitionPolicyThread(const TransitionPolicyThread& other)
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

    void work(Bag <Time, Value>& bag, const time_type& time,
              const std::size_t idx)
    {
        if (idx >= bag.size())
            return;

        std::size_t current_job_id = idx;
        auto it = bag.begin();
        std::advance(it, idx);

        for (;;) {
            (*it)->transition(time);
            current_job_id += pool.size();

            if (current_job_id >= bag.size())
                break;

            std::advance(it, pool.size());
        }
    }

    void operator()(Bag <Time,Value>& bag, const time_type& time,
                    HeapType <Time, Value> &heap)
    {
        if (bag.size() == 1) {
            Model <Time, Value>* child = (*bag.begin());

            child->transition(time);
            child->x.clear();
            (*child->heapid).tn = child->tn;
            heap.update(child->heapid);
        } else {
            for (size_t i = 0; i < pool.size(); ++i) {
                pool[i] = std::thread(&TransitionPolicyThread::work, this,
                                      std::ref(bag), time, i);
            }

            for (auto& worker : pool)
                if (worker.get_id() != std::thread::id())
                    worker.join();

            for (auto* child : bag) {
                (*child->heapid).tn = child->tn;
                heap.update(child->heapid);
                child->x.clear();
            }
        }
    }

private:
    std::vector <std::thread> pool;
};

template <typename Time, typename Value,
          typename Policy = TransitionPolicyThread <Time, Value>>
struct CoupledModel : ComposedModel <Time, Value>
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;
    typedef Policy transition_policy;

    HeapType <Time, Value> heap;
    transition_policy policy;

    typedef std::vector <Model <Time, Value>*> children_t;

    /**
     * @brief Get the children of the @e CoupledModel.
     *
     * The @e children function is called only once by the simulation layer
     * after the constructor.
     *
     * @return
     */
    virtual children_t children(const vle::Common& common) = 0;

    virtual void post(const UpdatedPort <Time, Value> &out,
                      UpdatedPort <Time, Value> &in) const override = 0;

    CoupledModel(const Context& ctx)
        : ComposedModel <Time, Value>(ctx)
        , policy(ctx->get_thread_number())
    {}

    CoupledModel(const Context& ctx, std::size_t input_size, std::size_t output_size)
        : ComposedModel <Time, Value>(ctx, input_size, output_size)
        , policy(ctx->get_thread_number())
    {}

    CoupledModel(const Context& ctx, unsigned thread_number)
        : ComposedModel <Time, Value>(ctx)
        , policy(thread_number)
    {}

    CoupledModel(const Context& ctx,
                 std::initializer_list <std::string> lst_x,
                 std::initializer_list <std::string> lst_y)
        : ComposedModel <Time, Value>(ctx, lst_x, lst_y)
        , policy(ctx->get_thread_number())
    {}

    CoupledModel(const Context& ctx,
                 unsigned thread_number,
                 std::initializer_list <std::string> lst_x,
                 std::initializer_list <std::string> lst_y)
        : ComposedModel <Time, Value>(ctx, lst_x, lst_y)
        , policy(thread_number)
    {}

    virtual ~CoupledModel()
    {}

    virtual void start(const vle::Common& common, const time_type& time) override
    {
        auto cs = children(common);

        for (auto child : cs) {
            child->parent = this;
            child->start(common, time);

            auto id = heap.emplace(child, child->tn);
            child->heapid = id;
            (*id).heapid = id;
        };

        Model <Time, Value>::tl = time;
        Model <Time, Value>::tn = heap.top().tn;
    }

    virtual void transition(const time_type& time) override
    {
        check_transition_synchronization <Time>(Model <Time, Value>::tl,
                                                time,
                                                Model <Time, Value>::tn);

        if (time < Model <Time, Value>::tn && Model <Time, Value>::x.empty())
            return;

        Bag <Time, Value> bag; /* The bag stores models in internal (where
                                * time equal child.tn), external (where
                                * child.input port is not empty. */

        {
            auto it = heap.ordered_begin();
            auto et = heap.ordered_end();

            for (; it != et && (*it).tn == time; ++it)
                bag.insert(
                    reinterpret_cast <Model <Time, Value>*>(
                        (*it).element));
        }

        if (not Model <Time, Value>::x.empty()) {
            post({this}, ComposedModel <Time, Value>::last_output_list);
            Model <Time, Value>::x.clear();
        }

        for (auto &child : ComposedModel <Time, Value>::last_output_list)
            bag.insert(const_cast <Model <Time, Value>*>(child));

        ComposedModel <Time, Value>::last_output_list.clear();

        policy(bag, time, heap);

        Model <Time, Value>::tl = time;
        Model <Time, Value>::tn = heap.top().tn;
    }

    virtual void output(const time_type& time) override
    {
        check_output_synchronization <Time>(heap.top().tn, time);

        if (time == Model <Time, Value>::tn && not heap.empty()) {
            UpdatedPort <Time, Value> lst;
            auto it = heap.ordered_begin();
            auto et = heap.ordered_end();

            assert(it != et);
            assert((std::size_t)std::distance(it, et) == heap.size());

            do {
                auto id = (*it).heapid;
                Model <Time, Value> *mdl =
                    reinterpret_cast <Model <Time, Value>*>(
                        (*id).element);

                mdl->output(time);
                if (!(mdl->y.empty()))
                    lst.emplace(mdl);
                ++it;
            } while (it != et && it->tn == Model <Time, Value>::tn);

            post(lst, ComposedModel <Time, Value>::last_output_list);

            /* If user adds this into the last_output_list, we need to remove it
             * from before clearing output ports. */
            ComposedModel <Time, Value>::last_output_list.erase(this);

            for (auto *child : lst)
                child->y.clear();
        }
    }
};

template <typename Time, typename Value,
          typename Policy = TransitionPolicyThread <Time, Value>>
struct Executive : ComposedModel <Time, Value>
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;
    typedef Policy transition_policy;

    HeapType <Time, Value> heap;

    typedef std::vector <Model <Time, Value>*> children_t;
    time_type chi_tl, chi_tn;
    typename HeapType <Time, Value>::handle_type chi_heapid;
    mutable vle::PortList <Value> chi_x, chi_y;
    transition_policy policy;
    vle::Common localcommon;

    virtual children_t children(const vle::Common& common) = 0;
    virtual time_type init(const time_type& time) = 0;
    virtual time_type delta(const time_type &elapsed,
                            const time_type &remaining,
                            const time_type &time) = 0;
    virtual void lambda() const = 0;
    virtual void post(const UpdatedPort <Time, Value> &y,
                      UpdatedPort <Time, Value> &x) const = 0;

    Executive(const vle::Context& ctx)
        : ComposedModel <Time, Value>(ctx)
        , chi_tl(Time::negative_infinity())
        , chi_tn(Time::infinity())
    {}

    Executive(const vle::Context& ctx,
              std::initializer_list <std::string> lst_x,
              std::initializer_list <std::string> lst_y)
        : ComposedModel <Time, Value>(ctx, lst_x, lst_y)
        , chi_tl(Time::negative_infinity())
        , chi_tn(Time::infinity())
    {}

    Executive(const vle::Context& ctx,
              std::initializer_list <std::string> lst_x,
              std::initializer_list <std::string> lst_y,
              std::initializer_list <std::string> chi_lst_x,
              std::initializer_list <std::string> chi_lst_y)
        : ComposedModel <Time, Value>(ctx, lst_x, lst_y)
        , chi_tl(Time::negative_infinity())
        , chi_tn(Time::infinity())
        , chi_x(chi_lst_x)
        , chi_y(chi_lst_y)
    {}

    virtual ~Executive()
    {}

    void insert(Model <Time, Value> *mdl)
    {
        mdl->parent = this;
        mdl->start(localcommon, Executive::chi_tl);

        auto id = heap.emplace(mdl, mdl->tn);
        mdl->heapid = id;
        (*id).heapid = id;
    }

    void erase(Model <Time, Value >*mdl)
    {
        ComposedModel <Time, Value>::last_output_list.erase(mdl);
        heap.erase(mdl->heapid);
        mdl->parent = nullptr;
    }

    virtual void start(const vle::Common& common, const time_type& time) override
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

            auto id = heap.emplace(child, child->tn);
            child->heapid = id;
            (*id).heapid = id;
        }

        Model <Time, Value>::tl = time;
        Model <Time, Value>::tn = heap.top().tn;
    }

    virtual void transition(const time_type &time) override
    {
        check_transition_synchronization <Time>(Model <Time, Value>::tl,
                                                time,
                                                Model <Time, Value>::tn);

        if (time < Model <Time, Value>::tn && Model <Time, Value>::x.empty())
            return;

        Bag <Time, Value> bag;
        bool have_chi = false;

        {
            auto it = heap.ordered_begin();
            auto et = heap.ordered_end();

            for (; it != et && (*it).tn == time; ++it) {
                if ((*it).element == this)
                    have_chi = true;
                else
                    bag.insert(
                        reinterpret_cast <Model <Time, Value>*>(
                            (*it).element));
            }
        }

        if (not Model <Time, Value>::x.empty()) {
            post({this}, ComposedModel <Time, Value>::last_output_list);
            Model <Time, Value>::x.clear();
        }

        for (auto &child : ComposedModel <Time, Value>::last_output_list) {
            if (child == this)
                have_chi = true;
            else
                bag.insert(const_cast <Model <Time, Value>*>(child));
        }

        ComposedModel <Time, Value>::last_output_list.clear();

        policy(bag, time, heap);

        if (have_chi) {
            time_type e = time - chi_tl;
            time_type r = chi_tn - time;
            chi_tl = time;
            chi_tn = time + delta(e, r, time);
            Model <Time, Value>::x.clear();
            (*chi_heapid).tn = chi_tn;
            heap.update(chi_heapid);
        }

        Model <Time, Value>::tl = time;
        Model <Time, Value>::tn = heap.top().tn;
    }

    virtual void output(const time_type &time) override
    {
        check_output_synchronization <Time>(heap.top().tn, time);

        if (time == Model <Time, Value>::tn && not heap.empty()) {
            UpdatedPort <Time, Value> lst;
            auto it = heap.ordered_begin();
            auto et = heap.ordered_end();

            do {
                auto id = (*it).heapid;
                Model <Time, Value> *mdl =
                    reinterpret_cast <Model <Time, Value>*>((*id).element);

                if (mdl == this) {
                    lambda();
                } else {
                    mdl->output(time);
                }
                if (!(mdl->y.empty()))
                    lst.emplace(mdl);
                ++it;
            } while (it != et && it->tn == Model <Time, Value>::tn);

            post(lst, ComposedModel <Time, Value>::last_output_list);

            /* If user adds this into the last_output_list, we need to remove it
             * from before clearing output ports. */
            ComposedModel <Time, Value>::last_output_list.erase(this);

            for (auto *child : lst)
                child->y.clear();
        }
    }
};

template <typename Time, typename Value>
struct Engine
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;
    typedef Model <Time, Value> model_type;
    vle::CommonPtr common;

    Engine()
        : common(std::make_shared <Common>())
    {}

    Engine(vle::CommonPtr common)
        : common(common)
    {}

    time_type pre(model_type& model, const time_type& time)
    {
        model.start(*common, time);

        return model.tn;
    }

    time_type run(model_type& model, const time_type& time)
    {
        model.output(time);
        model.transition(time);
        model.x.clear();

        return model.tn;
    }

    void post(model_type& model, const time_type& time)
    {
        (void)model;
        (void)time;
    }
};

}}

#include <vle/dsde/detail/dsde-implementation.hpp>

#endif
