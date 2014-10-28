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

#ifndef __VLE_KERNEL_DTSS_HPP__
#define __VLE_KERNEL_DTSS_HPP__

#include <vle/time.hpp>
#include <vle/port.hpp>
#include <vle/heap.hpp>
#include <memory>
#include <set>
#include <vle/dbg.hpp>

namespace vle {

struct dtss_internal_error : std::logic_error
{
    explicit dtss_internal_error(const std::string& msg)
        : std::logic_error(msg)
    {}
};

namespace dtss {

template <typename Time, typename Value>
struct Model
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    Model()
        : tn(Time::infinity), parent(nullptr)
    {}

    Model(std::initializer_list <std::string> lst_x,
          std::initializer_list <std::string> lst_y)
        : x(lst_x), y(lst_y), tn(Time::infinity),
          parent(nullptr)
    {}

    virtual ~Model()
    {}

    mutable vle::PortList <Value> x, y;
    time_type tn;
    time_type h;
    Model *parent;

    virtual void i_msg(const time_type& time) = 0;
    virtual void s_msg(const time_type& time) = 0;
    virtual void x_msg(const time_type& time) = 0;
    virtual void y_msg(Model <Time, Value>& model,
                       const time_type& time) = 0;
};

template <typename Time, typename Value>
using UpdatedPort = std::set <const Model <Time, Value>*>;

template <typename Time, typename Value>
struct Fnss : Model <Time, Value>
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    virtual void lambda() const = 0;

    Fnss()
        : Model <Time, Value>()
    {}

    Fnss(std::initializer_list <std::string> lst_x,
         std::initializer_list <std::string> lst_y)
        : Model <Time, Value>(lst_x, lst_y)
    {}

    virtual ~Fnss()
    {}

    virtual void i_msg(const time_type& time) override final
    {
        (void)time;
    }

    virtual void s_msg(const time_type& time) override final
    {
        (void)time;
    }

    virtual void x_msg(const time_type& time) override final
    {
        lambda();
        Model <Time, Value>::parent->y_msg(*this, time);
    }
};

template <typename Time, typename Value>
struct Moore : Model <Time, Value>
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    virtual void init(const time_type& time) = 0;
    virtual void delta() = 0;
    virtual void lambda() const = 0;

    Moore()
        : Model <Time, Value>()
    {}

    Moore(std::initializer_list <std::string> lst_x,
          std::initializer_list <std::string> lst_y)
        : Model <Time, Value>(lst_x, lst_y)
    {}

    virtual ~Moore()
    {}

    virtual void i_msg(const time_type& time) override final
    {
        init(time);
        Model <Time, Value>::tn = time;
    }

    virtual void s_msg(const time_type& time) override final
    {
#ifndef VLE_OPTIMIZE
        if (time != Model <Time, Value>::tn)
            throw dtss_internal_error("Synchronization error");
#endif
        lambda();
        Model <Time, Value>::parent->y_msg(this, time);
        delta();
    }

    virtual void x_msg(const time_type& time) override final
    {
#ifndef VLE_OPTIMIZE
        if (time != Model <Time, Value>::tn)
            throw dtss_internal_error("Synchronization error");
#endif
        delta();
        Model <Time, Value>::tn += Model <Time, Value>::h;
    }

    virtual void y_msg(Model <Time, Value>& model,
                       const time_type& time) override final
    {
        (void)model;
        (void)time;
    }
};

template <typename Time, typename Value>
struct CoupledModel : Model <Time, Value>
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    UpdatedPort <Time, Value> last_output_list;

    typedef std::vector <Fnss <Time, Value>*> children_fnss_t;
    typedef std::vector <Moore <Time, Value>*> children_moore_t;

    /**
     * @brief Get the children of the @e CoupledModel.
     *
     * The @e children function is called only once by the simulation layer
     * after the constructor.
     *
     * @return
     */
    virtual children_fnss_t fnss_children() = 0;

    virtual children_moore_t moore_children() = 0;

    virtual void post(const UpdatedPort <Time, Value> &out,
                      UpdatedPort <Time, Value> &in) const = 0;

    CoupledModel()
        : Model <Time, Value>()
    {}

    CoupledModel(std::initializer_list <std::string> lst_x,
                 std::initializer_list <std::string> lst_y)
        : Model <Time, Value>(lst_x, lst_y)
    {}

    virtual ~CoupledModel()
    {}

    virtual void i_msg(const time_type& time) override final
    {
        {
            auto children = fnss_children();
            std::for_each(children.begin(), children.end(),
                          [this, &time](Fnss <Time, Value> *child)
                          {
                              child->parent = this;
                              child->i_msg(time);
                          });
        }

        {
            auto children = moore_children();
            std::for_each(children.begin(), children.end(),
                          [this, &time](Moore <Time, Value> *child)
                          {
                              child->parent = this;
                              child->i_msg(time);
                          });
        }

        Model <Time, Value>::tn = time;
    }

    virtual void s_msg(const time_type& time) override final
    {
#ifndef VLE_OPTIMIZE
        if (time != Model <Time, Value>::tn)
            throw dtss_internal_error("Synchronization error");
#endif

        {
            auto children = moore_children();
            std::for_each(children.begin(), children.end(),
                          [&time](Moore <Time, Value>* child)
                          {
                              child->s_msg(time);
                          });
        }

        // TODO
    }

    virtual void x_msg(const time_type& time) override final
    {
#ifndef VLE_OPTIMIZE
        if (time != Model <Time, Value>::tn)
            throw dtss_internal_error("Synchronization error");
#endif

        // TODO
    }

    virtual void y_msg(Model <Time, Value>& model,
                       const time_type& time) override final
    {
        (void)model;
        (void)time;
        // TODO
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
        model.s_message(time);
        model.x_message(time);

        return model.tn;
    }

    void post(model_type& model, const time_type& time)
    {
        (void)model;
        (void)time;
    }
};
}

};

#endif
