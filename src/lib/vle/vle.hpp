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

        Child(NetworkElement <Time, Value> *element) noexcept
            : element(element)
        {}

        Child(std::initializer_list <std::string> lst_x,
              std::initializer_list <std::string> lst_y,
              NetworkElement <Time, Value> *element) noexcept
            : x(lst_x), y(lst_y), element(element)
        {}

        Child(Child&& other) = delete;
        //     : x(other.x), y(other.y), element(other.element)
        // {
        //     if (other.element->parent)
        //         other.element->parent->move(other.element, this);

        //     // TODO we need to replace &other by this in the
        //     // last_output_list of NetworkSimulator or
        //     // ExecutiveSimulator.
        //     other.element = nullptr;
        //     other.x.clear(); // TODO normally, not necessary.
        //     other.y.clear();
        // }

        Child(const Child& other) noexcept
            : x(other.x), y(other.y), element(nullptr)
        {}

        Child& operator=(const Child& other) = delete;
        Child& operator=(Child&& other) = delete;
        // {
        //     // TODO we need to replace &other by this in the last_output_list of
        //     // NetworkSimulator or ExecutiveSimulator.

        //     x = std::move(other.x);
        //     y = std::move(other.y);
        //     element = std::move(other.element);

        //     return *this;
        // }

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

        NetworkElement(const NetworkElement& other) = delete;
        NetworkElement(NetworkElement&& other) = delete;
        NetworkElement& operator=(const NetworkElement& other) = delete;
        NetworkElement& operator=(NetworkElement&& other) = delete;

        virtual ~NetworkElement() {}

        virtual void push(Child <Time, Value> *new_child) = 0;

        virtual void destroy(Child <Time, Value> *to_delete) = 0;

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

    virtual ~Simulator()
    {
        if (NetworkElement <Time, Value>::parent)
            NetworkElement <Time, Value>::parent->destroy(model_);
    }

    virtual void push(Child <Time, Value> *new_child) override
    {
        throw dsde_internal_error("Simulator can not push new model");
    }

    virtual void destroy(Child <Time, Value> *to_delete) override
    {
        throw dsde_internal_error("Simulator can not destroy model");
    }

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
    HeapType <Time, Value> heap;
    UpdatedPort <Time, Value> last_output_list;
    Executive <Time, Value> *model_;
    typename Time::type chi_tl;
    typename Time::type chi_tn;
    typename HeapType <Time, Value>::handle_type chi_heapid;

    ExecutiveSimulator(Executive <Time, Value> *model)
        : model_(model), chi_tl(-Time::infinity), chi_tn(Time::infinity)
    {
        dAssert(model);
    }

    virtual ~ExecutiveSimulator()
    {
        if (NetworkElement <Time, Value>::parent)
            NetworkElement <Time, Value>::parent->destroy(model_);
    }

    virtual void push(Child <Time, Value> *new_child) override
    {
        dPrint(XS(this), "A model to push");

        new_child->element->parent = this;
        new_child->element->start(NetworkElement <Time, Value>::tn);

        auto id = heap.emplace(new_child->element,
                               NetworkElement <Time, Value>::tn);
        (*id).heapid = id;
        new_child->element->heapid = id;
    }

    virtual void destroy(Child <Time, Value> *to_delete) override
    {
        dPrint(XS(this), "A model to delete");

        auto it = last_output_list.find(to_delete); /* remove the Child from the
                                                       next transition. */
        if (it != last_output_list.end())
            last_output_list.erase(it);

        heap.erase(to_delete->element->heapid); /* remove the Child from the
                                                   scheduler. */

        to_delete->element->parent = nullptr; /* finally, remove the parent. */
    }

    virtual void start(typename Time::type t) override
    {
        dPrint("ExecutiveSimulator::start: ", t);
        /* Initialize the Chi model. */
        chi_tl = t;
        chi_tn = t + model_->start(t);

        dAssert(model_->element == this);

        auto id = heap.emplace(this, chi_tn);
        chi_heapid = id;
        (*id).heapid = id;

        /* Initialize the default children. */
        std::vector <Child <Time, Value>*> children(model_->children());
        std::for_each(children.begin(), children.end(),
                      [this, t](Child <Time, Value>* child)
                      {
                          child->element->parent = this;
                          child->element->start(t);

                          auto id = heap.emplace(child->element,
                                                 child->element->tn);
                          (*id).heapid = id;
                          child->element->heapid = id;
                      });


        NetworkElement <Time, Value>::tl = t;
        NetworkElement <Time, Value>::tn = heap.top().tn;

        dPrint("ExecutiveSimulator::start_end, tn: ", heap.top().tn, " heap"
               "size: ", heap.size());
    }

    virtual void transition(typename Time::type t) override
    {
#ifndef VLE_OPTIMIZE
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
        bool have_chi = false;

        {
            auto it = heap.ordered_begin();
            auto et = heap.ordered_end();

            for (; it != et && (*it).tn == t; ++it) {
                if ((*it).element == this)
                    have_chi = true;
                else
                    bag.insert((*it).element);
            }
        }

        {
            if (not x().is_empty())
                model_->post({this->model_}, last_output_list);

            for (auto & child : last_output_list) {
                if (child->element == this)
                    have_chi = true;
                else
                    bag.insert(child->element);
            }

            last_output_list.clear();
        }

        for (auto &element : bag) {
            element->transition(t);
            (*(element->heapid)).tn = element->tn;
            heap.update(element->heapid);
        }

        if (have_chi) {
            dPrint(XS(this), "CHI make a transition");
            chi_tn = t + model_->transition(t - chi_tl);
            chi_tl = t;
            (*chi_heapid).tn = chi_tn;
            heap.update(chi_heapid);
        }

        // TODO verify, the push function use instead transition call the start
        // function of the new Child

        NetworkElement <Time, Value>::tn = heap.top().tn;
    }

    virtual void output(typename Time::type t) override
    {
        dPrint(XS(this), "ExecutiveSimulator::output t: ", t);
#ifndef VLE_OPTIMIZE
        if (!t == heap.top().tn)
            throw dsde_internal_error("NetworkSimulator::transition");

        if (!NetworkElement <Time, Value>::tn ==
            heap.top().tn)
            throw dsde_internal_error("NetworkSimulator::transition");
#endif
        dAssert(last_output_list.empty());

        if (t == NetworkElement <Time, Value>::tn && not heap.empty()) {
            dPrint(XS(this), "ExecutiveSimulator::output t: ", t);
            UpdatedPort <Time, Value> lst;
            auto it = heap.ordered_begin();
            auto et = heap.ordered_end();

            do {
                dPrint(XS(this), "ExecutiveSimulator::output first (", heap.size(), ")");
                auto id = (*it).heapid;

                if ((*id).element == this) {
                    model_->output();
                } else {
                    (*id).element->output(t);
                    if (!(*id).element->y().is_empty())
                        lst.emplace((*id).element->model());
                }

                ++it;
                dPrint(XS(this), "ExecutiveSimulator::output first end");
            } while (it != et && it->tn == NetworkElement <Time, Value>::tn);

            model_->post(lst, last_output_list);

            std::for_each(lst.begin(), lst.end(),
                          [](const Child <Time, Value>* child)
                          {
                              child->y.clear();
                          });
        }
        dPrint("ExecutiveSimulator::output_end");
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

    virtual ~NetworkSimulator()
    {
        if (NetworkElement <Time, Value>::parent)
            NetworkElement <Time, Value>::parent->destroy(model_);
    }

    virtual void push(Child <Time, Value> *new_child) override
    {
        dPrint(XS(this), "A model to push");

        new_child->element->parent = this;
        new_child->element->start(NetworkElement <Time, Value>::tn);

        auto id = heap.emplace(new_child->element,
                               NetworkElement <Time, Value>::tn);
        (*id).heapid = id;
        new_child->element->heapid = id;
    }

    virtual void destroy(Child <Time, Value> *to_delete) override
    {
        /*
         * TODO: perhaps not necessary since we delete all the children when we
         * delete the NetworkSimulator.
         */

        dPrint(XS(this), "A model to delete");

        auto it = last_output_list.find(to_delete); /* remove the Child from the
                                                       next transition. */
        if (it != last_output_list.end())
            last_output_list.erase(it);

        heap.erase(to_delete->element->heapid); /* remove the Child from the
                                                   scheduler. */

        to_delete->element->parent = nullptr; /* finally, remove the parent. */
    }

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
    Dynamics()
        : Child <Time, Value>(new Simulator <Time, Value>(this))
    {}

    Dynamics(std::initializer_list <std::string> lst_x,
             std::initializer_list <std::string> lst_y) :
        Child <Time, Value>(lst_x, lst_y, new Simulator <Time, Value>(this))
    {}

    Dynamics(Dynamics&& other) = delete;
    //     : Child <Time, Value>(other)
    // {}

    Dynamics(const Dynamics& other)
        : Child <Time, Value>(other)
    {
        if (other.element) {
            Simulator <Time, Value> *sim(
                new Simulator <Time, Value>(this));

            sim->tl = other.element->tl;
            sim->tn = other.element->tn;
            sim->parent = other.element->parent;
            sim->model_ = this;
            Child <Time, Value>::element = sim;

            if (sim->parent)
                sim->parent->push(this);
        }
    }

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
                    std::initializer_list <std::string> lst_y)
        : Child <Time, Value>(lst_x, lst_y,
                              new NetworkSimulator <Time, Value>(this))
    {}

    NetworkDynamics(NetworkDynamics&& other) = delete;
    //     : Child <Time, Value>(other)
    // {}

    NetworkDynamics(const NetworkDynamics& other)
        : Child <Time, Value>(other)
    {
        if (other.element) {
            NetworkSimulator <Time, Value> *ns(
                new NetworkSimulator <Time, Value>(this));

            ns->tl = other.element->tl;
            ns->tn = other.element->tn;
            ns->parent = other.element->parent;
            ns->model_ = this;
            Child <Time, Value>::element = ns;

            if (other.element->parent)
                ns->parent->push(this);
        }
    }

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

    Executive()
        : Child <Time, Value>(new ExecutiveSimulator <Time, Value>(this))
    {}

    Executive(std::initializer_list <std::string> lst_x,
              std::initializer_list <std::string> lst_y)
        : Child <Time, Value>(lst_x, lst_y,
                              new ExecutiveSimulator <Time, Value>(this))
    {}

    Executive(std::initializer_list <std::string> lst_x,
              std::initializer_list <std::string> lst_y,
              std::initializer_list <std::string> lst_xe,
              std::initializer_list <std::string> lst_ye)
        : Child <Time, Value>(lst_x, lst_y,
                              new ExecutiveSimulator <Time, Value>(this)),
        xe(lst_xe), ye(lst_ye)
    {}

    Executive(Executive&& other) = delete;
    //     : Child <Time, Value>(other)
    // {}

    Executive(const Executive& other)
        : Child <Time, Value>(other)
    {
        if (other.element) {
            ExecutiveSimulator <Time, Value> *es(
                new ExecutiveSimulator <Time, Value>(this));

            es->tl = other.element->tl;
            es->tn = other.element->tn;
            es->parent = other.element->parent;
            es->model_ = this;
            Child <Time, Value>::element = es;
        }
    }

    Executive& operator=(const Executive&) = delete;

    void push(Child <Time, Value>* new_child)
    {
        ((ExecutiveSimulator <Time, Value>*)
         Child <Time, Value>::element)->push(new_child);
    }

    void destroy(Child <Time, Value>* to_delete)
    {
        ((ExecutiveSimulator <Time, Value>*)
         Child <Time, Value>::element)->destroy(to_delete);
    }

    virtual ~Executive() {}

    virtual std::vector <Child <Time, Value>*> children() = 0;

    virtual typename Time::type start(typename Time::type t) = 0;

    virtual typename Time::type transition(typename Time::type e) = 0;

    virtual void output() const = 0;

    virtual std::string observation() const = 0;

    virtual void post(const UpdatedPort <Time, Value> &y,
                      UpdatedPort <Time, Value> &x) const = 0;
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
