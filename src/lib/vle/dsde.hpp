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

#ifndef __VLE_KERNEL_DSDE_HPP__
#define __VLE_KERNEL_DSDE_HPP__

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

/*
 * The synchronizer is the root coordinator in the DEVS terminology.
 *
 * @code
 * vle::Engine <dsde_engine <double, std::string>> engine;
 *
 * struct MyModel : vle::Engine <dsde_engine <double, std::string>>::model
 * {
 *   // [...]
 * };
 * MyModel model;
 * vle::Simulation sim(model, engine);
 * sim.run(0.0, 100.0);
 * sim.run(-1.0, 1.0);
 * sim.run_until(100.);
 * @encode
 */

template <typename Time, typename Value>
struct dsde
{
    typedef typename Time::type time_type;
    typedef Value value_type;

    struct Model
    {
        Model()
            : tl(-Time::infinity), tn(Time::infinity), parent(nullptr)
        {}

        Model(std::initializer_list <std::string> lst_x,
              std::initializer_list <std::string> lst_y)
            : x(lst_x), y(lst_y), tl(-Time::infinity), tn(Time::infinity),
            parent(nullptr)
        {}

        virtual ~Model()
        {}

        mutable vle::PortList <Value> x, y;
        time_type tl, tn;
        Model *parent;
        typename HeapType <Time, Value>::handle_type heapid;

        virtual void start(const time_type& time) = 0;
        virtual void transition(const time_type& time) = 0;
        virtual void output(const time_type& time) = 0;
    };

    using Bag = std::set <Model*>;
    using UpdatedPort = std::set <const Model*>;

    struct AtomicModel : Model
    {
        virtual time_type init(const time_type& time) = 0;
        virtual time_type delta(const time_type& time) = 0;
        virtual void lambda() const = 0;

        AtomicModel()
        {}

        AtomicModel(std::initializer_list <std::string> lst_x,
                    std::initializer_list <std::string> lst_y)
            : Model(lst_x, lst_y)
        {}

        virtual ~AtomicModel()
        {}

        virtual void start(const time_type& time) override final
        {
            Model::tl = time;
            Model::tn = time + init(time);
        }

        virtual void transition(const time_type& time) override final
        {
#ifndef VLE_OPTIMIZE
            if (!(Model::tl <= time && time <= Model::tn))
                throw dsde_internal_error("Simulator::transition");

            if (time < Model::tn and Model::x.is_empty())
                return;
#endif
            Model::tn = time + delta(time - Model::tl);
            Model::tl = time;
            Model::x.clear();
        }

        virtual void output(const time_type& time) override final
        {
            if (time == Model::tn)
                lambda();
        }
    };

    struct CoupledModel : Model
    {
        HeapType <Time, Value> heap;
        UpdatedPort last_output_list;

        typedef std::vector <Model*> children_t;

        /**
         * @brief Get the children of the @e CoupledModel.
         *
         * The @e children function is called only once by the simulation layer
         * after the constructor.
         *
         * @return
         */
        virtual children_t children() = 0;

        virtual void post(const UpdatedPort &out, UpdatedPort &in) const = 0;

        CoupledModel()
            : Model()
        {}

        CoupledModel(std::initializer_list <std::string> lst_x,
                     std::initializer_list <std::string> lst_y)
            : Model(lst_x, lst_y)
        {}

        virtual ~CoupledModel()
        {}

        virtual void start(const time_type& time) override final
        {
            auto cs = children();
            std::for_each(cs.begin(), cs.end(),
                          [=](Model *child)
                          {
                              child->parent = this;
                              child->start(time);

                              auto id = heap.emplace(child, child->tn);
                              child->heapid = id;
                              (*id).heapid = id;
                          });

            Model::tl = time;
            Model::tn = heap.top().tn;
        }

        virtual void transition(const time_type& time) override final
        {
#ifndef VLE_OPTIMIZE
            if (!(Model::tl <= time && time <= Model::tn))
                throw dsde_internal_error("NetworkSimulator::transition");

            if (time < Model::tn && Model::x.is_empty())
                return;
#endif
            Bag bag;

            {
                auto it = heap.ordered_begin();
                auto et = heap.ordered_end();

                for (; it != et && (*it).tn == time; ++it)
                    bag.insert(reinterpret_cast <Model*>((*it).element));
            }

            {
                if (not Model::x.is_empty())
                    post({this}, last_output_list);

                for (auto &child : last_output_list)
                    bag.insert(const_cast <Model*>(child));

                last_output_list.clear();
            }

            std::for_each(bag.begin(), bag.end(),
                          [=](Model *child)
                          {
                              child->transition(time);
                              (*child->heapid).tn = child->tn;
                              heap.update(child->heapid);
                          });

            Model::tl = time;
            Model::tn = heap.top().tn;
            Model::x.clear();
        }

        virtual void output(const time_type& time) override final
        {
#ifndef VLE_OPTIMIZE
            if (!(time == heap.top().tn))
                throw dsde_internal_error("NetworkSimulator::transition");

            if (!(Model::tn == heap.top().tn))
                throw dsde_internal_error("NetworkSimulator::transition");
#endif
            if (time == Model::tn && not heap.empty()) {
                UpdatedPort lst;
                auto it = heap.ordered_begin();
                auto et = heap.ordered_end();

                do {
                    auto id = (*it).heapid;
                    Model *mdl = reinterpret_cast <Model*>((*id).element);

                    mdl->output(time);
                    if (!(mdl->y.is_empty()))
                        lst.emplace(mdl);
                    ++it;
                } while (it != et && it->tn == Model::tn);

                post(lst, last_output_list);

                std::for_each(lst.begin(), lst.end(),
                              [](const Model *child)
                              {
                                  child->y.clear();
                              });
            }
        }
    };

    struct Executive : Model
    {
        HeapType <Time, Value> heap;
        UpdatedPort last_output_list;

        typedef std::vector <Model*> children_t;
        time_type chi_tl, chi_tn;
        typename HeapType <Time, Value>::handle_type chi_heapid;
        mutable vle::PortList <Value> chi_x, chi_y;

        virtual children_t children() = 0;
        virtual time_type init(const time_type& time) = 0;
        virtual time_type delta(const time_type& time) = 0;
        virtual void lambda() const = 0;
        virtual void post(const UpdatedPort &y, UpdatedPort &x) const = 0;

        Executive()
            : Model()
        {}

        Executive(std::initializer_list <std::string> lst_x,
                  std::initializer_list <std::string> lst_y)
            : Model(lst_x, lst_y)
        {}

        Executive(std::initializer_list <std::string> lst_x,
                  std::initializer_list <std::string> lst_y,
                  std::initializer_list <std::string> chi_lst_x,
                  std::initializer_list <std::string> chi_lst_y)
            : Model(lst_x, lst_y), chi_tl(-Time::infinity),
            chi_tn(Time::infinity), chi_x(chi_lst_x), chi_y(chi_lst_y)
        {}

        virtual ~Executive()
        {}

        void insert(Model *mdl)
        {
            dWarning("Executive insert a new model");
            mdl->parent = this;
            mdl->start(Executive::chi_tl);

            auto id = heap.emplace(mdl, mdl->tn);
            mdl->heapid = id;
            (*id).heapid = id;
        }

        void erase(Model *mdl)
        {
            dWarning("Execute erase a model");
            last_output_list.erase(mdl);
            heap.erase(mdl->heapid);
            mdl->parent = nullptr;
        }

        virtual void start(const time_type& time) override final
        {
            chi_tl = time;
            chi_tn = time + init(time);
            auto id = heap.emplace(this, chi_tn);
            chi_heapid = id;
            (*id).heapid = id;

            auto cs = children();
            std::for_each(cs.begin(), cs.end(),
                          [=](Model *child)
                          {
                              child->parent = this;
                              child->start(time);

                              auto id = heap.emplace(child, child->tn);
                              (*id).heapid = id;
                              child->heapid = id;
                          });

            Model::tl = time;
            Model::tn = heap.top().tn;
            Model::x.clear();
        }

        virtual void transition(const time_type &time) override final
        {
#ifndef VLE_OPTIMIZE
            if (!(Model::tl <= time && time <= Model::tn))
                throw dsde_internal_error("Executive::transition");

            if (time < Model::tn && Model::x.is_empty())
                return;
#endif
            Bag bag;
            bool have_chi = false;

            {
                auto it = heap.ordered_begin();
                auto et = heap.ordered_end();

                for (; it != et && (*it).tn == time; ++it) {
                    if ((*it).element == this)
                        have_chi = true;
                    else
                        bag.insert(reinterpret_cast <Model*>((*it).element));
                }
            }

            {
                if (not Model::x.is_empty())
                    post({this}, last_output_list);

                for (auto &child : last_output_list) {
                    if (child == this)
                        have_chi = true;
                    else
                        bag.insert(const_cast <Model*>(child));
                }

                last_output_list.clear();
            }

            std::for_each(bag.begin(), bag.end(),
                          [=](Model *child)
                          {
                              child->transition(time);
                              (*child->heapid).tn = child->tn;
                              heap.update(child->heapid);
                          });

            if (have_chi) {
                time_type e = time - chi_tl;
                chi_tl = time;
                chi_tn = time + delta(e);
                (*chi_heapid).tn = chi_tn;
                heap.update(chi_heapid);
            }

            Model::tn = heap.top().tn;
            Model::tl = time;
            Model::x.clear();
        }

        virtual void output(const time_type &time) override final
        {
#ifndef VLE_OPTIMIZE
            if (!(time == heap.top().tn))
                throw dsde_internal_error("Executive::transition");

            if (!(Model::tn == heap.top().tn))
                throw dsde_internal_error("Executive::transition");
#endif
            if (time == Model::tn && not heap.empty()) {
                UpdatedPort lst;
                auto it = heap.ordered_begin();
                auto et = heap.ordered_end();

                do {
                    auto id = (*it).heapid;
                    Model *mdl = reinterpret_cast <Model*>((*id).element);

                    if (mdl == this) {
                        lambda();
                    } else {
                        mdl->output(time);
                    }
                    if (!(mdl->y.is_empty()))
                        lst.emplace(mdl);
                    ++it;
                } while (it != et && it->tn == Model::tn);

                post(lst, last_output_list);

                std::for_each(lst.begin(), lst.end(),
                              [](const Model *child)
                              {
                                child->y.clear();
                              });
            }
        }
    };

    template <typename Model>
        time_type pre(Model &model, const time_type& time)
        {
            model.start(time);

            return model.tn;
        }

    template <typename Model>
        time_type run(Model &model, const time_type& time)
        {
            model.output(time);
            model.transition(time);

            return model.tn;
        }

    template <typename Model>
        void post(Model& model, const time_type& time)
        {
            (void)model;
            (void)time;
        }
};

};

#endif
