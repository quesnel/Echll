/*
 * Copyright (C) 2013 INRA
 * Copyright (C) 2013 ULCO
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

#ifndef __VLE_KERNEL_VLE_HPP__
#define __VLE_KERNEL_VLE_HPP__

#include <vle/value.hpp>
#include <vle/time.hpp>
#include <vle/port.hpp>
#include <vle/heap.hpp>
#include <memory>
#include <set>
#include <vle/dbg.hpp>

namespace vle {

struct dsde_internal_error : std::runtime_error
{
    explicit dsde_internal_error(const std::string& msg)
        : std::runtime_error(msg)
    {}
};

template <typename Time, typename Value>
    struct Dynamics;

template <typename Time, typename Value>
    struct Executive;

template <typename Time, typename Value>
    struct NetworkDynamics;

template <typename Time, typename Value>
    struct Simulator;

template <typename Time, typename Value>
    struct ExecutiveSimulator;

template <typename Time, typename Value>
    struct NetworkSimulator;

template <typename Time, typename Value>
    struct NetworkElement;

template <typename Time, typename Value>
    struct Child
    {
        mutable vle::PortList <Value> x, y;
        NetworkElement <Time, Value>* element;

        Child(NetworkElement <Time, Value>* element)
            : element(element)
        {}

        Child(std::initializer_list <std::string> lst_x,
              std::initializer_list <std::string> lst_y,
              NetworkElement <Time, Value>* element)
            : x(lst_x), y(lst_y), element(element)
        {}

        virtual ~Child()
        {
            delete element;
        }
    };

template <typename Time, typename Value>
    using UpdatedPort = std::set <const Child <Time, Value>*>;

template <typename Time, typename Value>
    using Bag = std::set <NetworkElement <Time, Value>*>;

template <typename Time, typename Value>
    struct NetworkElement
    {
        typename HeapType <Time, Value>::handle_type heapid;
        typename Time::type tl;
        typename Time::type tn;
        struct NetworkElement < Time, Value> *parent;

        NetworkElement()
            : tl(-Time::infinity), tn(Time::infinity), parent(nullptr)
        {}

        virtual ~NetworkElement()
        {}

        virtual void start(typename Time::type t) = 0;

        virtual void transition(typename Time::type t) = 0;

        virtual void output(typename Time::type t) = 0;

        virtual const vle::PortList <Value>& x() const = 0;

        virtual const vle::PortList <Value>& y() const = 0;

        virtual Child <Time, Value>* model() const = 0;
    };

template <typename Time, typename Value>
std::string XS(NetworkElement <Time, Value> *a)
{
    size_t i = 0;

    for (; a; a = a->parent)
        i++;

    return std::string(i, ' ');
}

template <typename Time, typename Value>
    struct Simulator : NetworkElement <Time, Value>
{
    Dynamics <Time, Value> *model_;

    Simulator(Dynamics <Time, Value> *model)
        : model_(model)
    {
        dAssert(model);
    }

    Simulator(const Simulator&) = delete;
    Simulator& operator=(const Simulator&) = delete;

    virtual ~Simulator() {}

    virtual void start(typename Time::type t) override
    {
        dPrint(XS(this), "Simulator::start at: ", t);
        NetworkElement <Time, Value>::tl = t;
        NetworkElement <Time, Value>::tn = t + model_->start(t);
    }

    virtual void transition(typename Time::type t) override
    {
#ifndef VLE_OPTIMIZE
        if (!(NetworkElement <Time, Value>::tl <= t && t <=
              NetworkElement <Time, Value>::tn))
            throw dsde_internal_error("Simulator::transition");
#endif
        dPrint(XS(this), "Simulator::transition at: ", t);

        if (t < NetworkElement <Time, Value>::tn and
            model_->x.is_empty())
            return;

        NetworkElement <Time, Value>::tn =
            t + model_->transition(t - NetworkElement <Time, Value>::tl);
        NetworkElement <Time, Value>::tl = t;
    }

    virtual void output(typename Time::type t) override
    {
        if (t == NetworkElement <Time, Value>::tn)
            model_->output();
    }

    virtual const vle::PortList <Value>& x() const override
    {
        return model_->x;
    }

    virtual const vle::PortList <Value>& y() const override
    {
        return model_->y;
    }

    virtual Child <Time, Value>* model() const override
    {
        return model_;
    }
};

template <typename Time, typename Value>
struct ExecutiveSimulator : NetworkElement <Time, Value>
{
    Executive <Time, Value> *model_;

    ExecutiveSimulator(Executive <Time, Value> *model)
        : model_(model)
    {
        dAssert(model);
    }

    ExecutiveSimulator(const ExecutiveSimulator&) = delete;
    ExecutiveSimulator& operator=(const ExecutiveSimulator&) = delete;

    virtual ~ExecutiveSimulator()
    {}

    virtual void start(typename Time::type t) override
    {
        NetworkElement <Time, Value>::tl = t;
        NetworkElement <Time, Value>::tn = t + model_->start(t);
    }

    virtual void transition(typename Time::type t) override
    {
#ifndef VLE_OPTIMIZE
        if (!(NetworkElement <Time, Value>::tl <= t && t <=
              NetworkElement <Time, Value>::tn))
            throw dsde_internal_error("ExecutiveSimulator::transition");
#endif

        if (t < NetworkElement <Time, Value>::tn and
            model_->x.is_empty())
            return;

        NetworkElement <Time, Value>::tn =
            t + model_->transition(t - NetworkElement <Time, Value>::tl);
        NetworkElement <Time, Value>::tl = t;
    }

    virtual void output(typename Time::type t) override
    {
        if (t == NetworkElement <Time, Value>::tn)
            model_->output();
    }

    virtual const vle::PortList <Value>& x() const override
    {
        return model_->xe;
    }

    virtual const vle::PortList <Value>& y() const override
    {
        return model_->ye;
    }

    virtual Child <Time, Value>* model() const override
    {
        return model_;
    }
};

template <typename Time, typename Value>
struct NetworkSimulator : NetworkElement <Time, Value>
{
    HeapType <Time, Value> heap;
    UpdatedPort <Time, Value> last_output_list;
    NetworkDynamics <Time, Value> *model_;

    NetworkSimulator(NetworkDynamics <Time, Value>* model)
        : model_(model)
    {
        dAssert(model);
    }

    NetworkSimulator(const NetworkSimulator&) = delete;
    NetworkSimulator& operator=(const NetworkSimulator&) = delete;

    virtual void start(typename Time::type t) override
    {
        dPrint(XS(this), "NS::begin start at time=", t, " heap_size=",
               heap.size());
        std::vector <Child <Time, Value>*> children(model_->children());

        for (Child <Time, Value> *child : children) {
            if (!child)
                throw dsde_internal_error("NetworkDynamics::empty child");

            dPrint(XS(this), "NS::start and start a child: ", child->element);
            child->element->parent = this;
            child->element->start(t);

            auto id = heap.emplace(child->element, child->element->tn);
            (*id).heapid = id;
            child->element->heapid = id;
        }

        NetworkElement <Time, Value>::tl = t;
        NetworkElement <Time, Value>::tn = heap.top().tn;

        dPrint(XS(this), "NS::start finish at time=", t, " heap_size=",
               heap.size(), " tn=", NetworkElement <Time, Value>::tn);
    }

    virtual void transition(typename Time::type t) override
    {
#ifndef VLE_OPTIMIZE
        dPrint(XS(this), "NS::transition at ", t, " size = ", heap.size());

        if (!(NetworkElement <Time, Value>::tl <= t && t <=
              NetworkElement <Time, Value>::tn))
            throw dsde_internal_error("NetworkSimulator::transition");

        if (!NetworkElement <Time, Value>::tn ==
            heap.top().tn)
            throw dsde_internal_error("NetworkSimulator::transition");
#endif

        if (t < NetworkElement <Time, Value>::tn && model_->x.is_empty())
            return;

        Bag <Time, Value> bag;

        {
            auto it = heap.ordered_begin();
            auto et = heap.ordered_end();

            for (; it != et && (*it).tn == t; ++it)
                bag.insert((*it).element);

            dPrint(XS(this), "NS::transition build bag (with t ==", t, ") : ",
                   bag.size());
        }

        {
            /* If NS received message from parent, we add the message into the
             * last_output_list variable.
             */
            if (not x().is_empty())
                model_->post({this->model_}, last_output_list);

            /* Merge all output message from imminent bag. */
            for (auto & child : last_output_list)
                bag.insert(child->element);

            last_output_list.clear();

            dPrint(XS(this), "NS::transition build bag (for list message =",
                   last_output_list.size(), ") : ", bag.size());
        }

        for (auto &element : bag) {
            element->transition(t);
            (*(element->heapid)).tn = element->tn;
            heap.update(element->heapid);
        }

        NetworkElement <Time, Value>::tn = heap.top().tn;
    }

    virtual void output(typename Time::type t) override
    {
#ifndef VLE_OPTIMIZE
        dPrint(XS(this), "NS::output at ", t, " size=", heap.size());

        if (!t == heap.top().tn)
            throw dsde_internal_error("NetworkSimulator::transition");

        if (!NetworkElement <Time, Value>::tn ==
            heap.top().tn)
            throw dsde_internal_error("NetworkSimulator::transition");
#endif
        dAssert(last_output_list.empty());

        if (t == NetworkElement <Time, Value>::tn && not heap.empty()) {
            UpdatedPort <Time, Value> lst;

            dPrint(XS(this), "NS::output need to call output for imminent "
                   "children");

            typename HeapType <Time, Value>::ordered_iterator it = heap.ordered_begin();
            typename HeapType <Time, Value>::ordered_iterator et = heap.ordered_end();

            do {
                auto id = (*it).heapid;

                (*id).element->output(t);
                if (!(*id).element->y().is_empty()) {
                    lst.emplace((*id).element->model());
                    dPrint(XS(this), "NS::output child send message");
                } else {
                    dPrint(XS(this), "NS::output children dont send message");
                }
                ++it;
            } while (it != et && it->tn == NetworkElement <Time, Value>::tn);

            model_->post(lst, last_output_list);

            std::for_each(lst.begin(), lst.end(),
                          [](const Child <Time, Value>* child)
                          {
                              child->y.clear();
                          });


            dPrint(XS(this), "NS::output, need to wake up : ",
                   last_output_list.size(), " model");
        }
    }

    virtual const vle::PortList <Value>& x() const override
    {
        return model_->x;
    }

    virtual const vle::PortList <Value>& y() const override
    {
        return model_->y;
    }

    virtual Child <Time, Value>* model() const override
    {
        return model_;
    }
};

template <typename Time, typename Value>
struct Dynamics : public Child <Time, Value>
{
    std::unique_ptr <Simulator <Time, Value>> simulator_;

    Dynamics()
        : Child <Time, Value>(new Simulator <Time, Value>(this))
    {}

    Dynamics(std::initializer_list <std::string> lst_x,
             std::initializer_list <std::string> lst_y) :
        Child <Time, Value>(lst_x, lst_y, new Simulator <Time, Value>(this))
    {}

    Dynamics(const Dynamics&) = delete;
    Dynamics& operator=(const Dynamics&) = delete;

    virtual ~Dynamics() {}

    virtual typename Time::type start(typename Time::type t) = 0;

    virtual typename Time::type transition(typename Time::type e) = 0;

    virtual void output() const = 0;

    virtual std::string observation() const = 0;
};

template <typename Time, typename Value>
struct NetworkDynamics : public Child <Time, Value>
{
    NetworkDynamics()
        : Child <Time, Value>(new NetworkSimulator <Time, Value>(this))
    {}

    NetworkDynamics(std::initializer_list <std::string> lst_x,
                    std::initializer_list <std::string> lst_y) :
        Child <Time, Value>(lst_x, lst_y,
                            new NetworkSimulator <Time, Value>(this))
    {}

    NetworkDynamics(const NetworkDynamics&) = delete;
    NetworkDynamics& operator=(const NetworkDynamics&) = delete;

    virtual ~NetworkDynamics() {}

    virtual std::vector <Child <Time, Value>*> children() = 0;

    virtual std::string observation() const = 0;

    virtual void post(const UpdatedPort <Time, Value> &y,
                      UpdatedPort <Time, Value> &x) const = 0;
};

template < typename Time, typename Value >
struct Executive : public Child <Time, Value>
{
    vle::PortList <Value> xe, ye; /* Input and Output ports of the
                                     Executive. */

    Executive() :
        Child <Time, Value>(new ExecutiveSimulator <Time, Value>())
    {}

    Executive(std::initializer_list <std::string> lst_x,
              std::initializer_list <std::string> lst_y) :
        Child <Time, Value>(lst_x, lst_y,
                            new ExecutiveSimulator <Time, Value>())
    {}

    Executive(std::initializer_list <std::string> lst_x,
              std::initializer_list <std::string> lst_y,
              std::initializer_list <std::string> lst_xe,
              std::initializer_list <std::string> lst_ye) :
        Child <Time, Value>(lst_x, lst_y,
                            new ExecutiveSimulator <Time, Value>(),
                            xe(lst_xe), ye(lst_ye))
    {}

    Executive(const Executive&) = delete;
    Executive& operator=(const Executive&) = delete;

    virtual ~Executive() {}

    virtual typename Time::type start(typename Time::type t) = 0;

    virtual typename Time::type transition(typename Time::type e) = 0;

    virtual void output() const = 0;

    virtual std::string observation() const = 0;

    virtual void post(const UpdatedPort <Time, Value> &y,
                      UpdatedPort <Time, Value> &x) = 0;

    //void push(std::shared_ptr <vle::Executive <Time, Value>> child)
    //{
    //simulator->push(child);
    //}

    //void push(std::shared_ptr <vle::Dynamics <Time, Value>> child)
    //{
    //simulator->push(child);
    //}
};

template <typename Time, typename Value>
struct Synchronizer
{
    vle::Child <Time, Value> *child;

    Synchronizer(vle::Child <Time, Value> *child)
        : child(child)
    {}

    typename Time::type run(typename Time::type begin,
                            typename Time::type end)
    {
        child->element->start(begin);
        begin = child->element->tn;

        while (begin < end) {
            child->element->output(begin);
            child->element->transition(begin);
            begin = child->element->tn;
        }

        return begin;
    }

    Synchronizer(const Synchronizer &other) = delete;
    Synchronizer& operator=(const Synchronizer &other) = delete;
};

template <typename Time, typename Value>
struct SynchronizerBagCounter
{
    vle::Child <Time, Value> *child;
    unsigned long int bag;

    SynchronizerBagCounter(vle::Child <Time, Value> *child)
        : child(child), bag(0)
    {}

    ~SynchronizerBagCounter()
    {}

    typename Time::type run(typename Time::type begin,
                            typename Time::type end)
    {
        child->element->start(begin);
        begin = child->element->tn;

        while (begin < end) {
            child->element->output(begin);
            child->element->transition(begin);
            begin = child->element->tn;

            ++bag;
        }

        return begin;
    }

    SynchronizerBagCounter(const SynchronizerBagCounter&) = delete;
    SynchronizerBagCounter& operator=(const SynchronizerBagCounter&) = delete;
};

}

#endif
