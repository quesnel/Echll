/*
 * Copyright (C) 2015 INRA
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

#ifndef __VLE_KERNEL_DSDE_QSS2_HPP__
#define __VLE_KERNEL_DSDE_QSS2_HPP__

#include <vle/dsde/dsde.hpp>
#include <valarray>

namespace vle { namespace dsde { namespace qss2 {

struct internal_error : std::runtime_error
{
    explicit internal_error(const std::string &msg)
        : std::runtime_error(msg)
    {}
};

typedef std::function <
    double(const std::valarray <double>& cst,
           const std::valarray <double>& v)> compute_fct;

template <typename Time, typename Value>
struct Variable : AtomicModel <Time, Value>
{
    typedef Time time_format;
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    const std::valarray <double> constants;
    time_type sigma;
    std::valarray <double> v;
    std::valarray <double> mv;
    std::valarray <double> c;
    compute_fct computer;
    double y1, y2;

    Variable(const Context &ctx,
             std::size_t system_size,
             compute_fct computer_,
             const std::valarray <double>& constants_)
        : AtomicModel <Time, Value>(ctx, system_size, 1u)
        , constants(constants_)
        , sigma(Time::infinity())
        , v(system_size)
        , mv(system_size)
        , c(system_size)
        , computer(computer_)
        , y1(0.0)
        , y2(0.0)
    {
        if (system_size == 0)
            throw internal_error("bad system size for variable");
    }

    virtual ~Variable()
    {
    }

    virtual time_type init(const vle::Common& common, const time_type& time)
    {
        (void)common;
        (void)time;

        return Time::infinity();
    }

    virtual void lambda() const
    {
        AtomicModel <Time, Value>::y[0] = { y1, y2 };
    }

    virtual time_type delta(const time_type& e, const time_type& r,
                            const time_type& t)
    {
        (void)r;
        (void)t;

        sigma = Time::infinity();

        if (not AtomicModel <Time, Value>::x.empty()) {
            for (std::size_t portid = 0, endportid = AtomicModel <Time, Value>::x.size();
                 portid != endportid; ++portid) {

                if (not AtomicModel <Time, Value>::x[portid].empty()) {
                    for (std::size_t i = 0; i < v.size(); ++i)
                        v[i] += mv[i] * e;

                    double fv = computer(constants, v);
                    double vaux = v[portid];
                    v[portid] = AtomicModel <Time, Value>::x[portid][0];
                    mv[portid] = AtomicModel <Time, Value>::x[portid][1];
                    y1 = computer(constants, v);

                    if (vaux != v[portid])
                        c[portid] = (fv - y1) / (vaux - v[portid]);

                    y2 = 0.0;
                    for (std::size_t i = 0, e = mv.size(); i != e; ++i)
                        y2 += mv[i] * c[i];
                }
            }

            sigma = Time::null();
        }

        return sigma;
    }
};

template <typename Time, typename Value>
struct Integrator : AtomicModel <Time, Value>
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    time_type sigma;
    double dq, dx, mdx, X, q, mq;

    Integrator(const Context &ctx, double dq_, double X_)
        : AtomicModel <Time, Value>(ctx, 1u, 1u)
        , sigma(Time::null())
        , dq(dq_)
        , dx(0.0)
        , mdx(0.0)
        , X(X_)
        , q(X_)
        , mq(0.0)
    {
    }

    virtual ~Integrator()
    {
    }

    virtual time_type init(const vle::Common& common, const time_type& time)
    {
        (void)common;
        (void)time;

        return sigma;
    }

    virtual void lambda() const
    {
        if (sigma)
            AtomicModel <Time, Value>::y[0] = {
                X + dx * sigma + mdx / 2.0 * sigma * sigma,
                dx + mdx * sigma };
        else
            AtomicModel <Time, Value>::y[0] = {
                X,
                dx };
    }

    virtual time_type delta(const time_type& e, const time_type& r,
                            const time_type& t)
    {
        (void)r;
        (void)t;

        if (AtomicModel <Time, Value>::x.empty()) {
            X += dx * sigma + mdx / 2.0 * sigma * sigma;
            q = X;
            dx += mdx * sigma;
            mq = dx;

            if (mdx == 0.0)
                sigma = Time::infinity();
            else
                sigma = std::sqrt(2 * dq / std::abs(mdx));
        } else {
            double a, b, c, s;

            X += dx * e + mdx / 2 * e * e;
            dx = AtomicModel <Time, Value>::x[0][0];
            mdx = AtomicModel <Time, Value>::x[0][1];

            if (sigma != Time::null()) {
                q += mq * e;
                a = mdx / 2.0;
                b = dx - mq;
                c = X - q + dq;
                sigma = Time::infinity();

                if (a == 0.0) {
                    if (b != 0.0) {
                        s = - c / b;
                        if (s > 0.0)
                            sigma = s;
                        c = X - q - dq;
                        s = - c / b;
                        if ((s > 0.0 and s < sigma))
                            sigma = s;
                    }
                } else {
                    s = (- b + std::sqrt(b * b - 4 * a * c)) / 2.0 / a;
                    if (s > 0.0)
                        sigma = s;
                    s = (- b - std::sqrt(b * b - 4 * a * c)) / 2.0 / a;
                    if (s > 0.0 and s < sigma)
                        sigma = s;
                    c = X - q - dq;
                    s = (- b + std::sqrt(b * b - 4 * a * c)) / 2.0 / a;
                    if (s > 0.0 and s < sigma)
                        sigma = s;
                    s = (- b - std::sqrt(b * b - 4 * a * c)) / 2.0 / a;
                    if (s > 0.0 and s < sigma)
                        sigma = s;
                }
            }
        }

        return sigma;
    }
};

template <typename Time, typename Value>
struct Equation : CoupledModel <Time, Value>
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    Integrator <Time, Value> m_integrator;
    Variable <Time, Value> m_variable;

    Equation(const vle::Context &ctx,
             double dq,
             double x,
             std::size_t system_size,
             compute_fct compute,
             const std::valarray <double>& constants)
        : CoupledModel <Time, Value>(ctx, system_size, 1u)
        , m_integrator(ctx, dq, x)
        , m_variable(ctx, system_size, compute, constants)
    {
    }

    virtual ~Equation()
    {
    }

    virtual typename CoupledModel <Time, Value>::children_t
    children(const vle::Common&) override final
    {
        return { &m_integrator, &m_variable };
    }

    virtual void post(const UpdatedPort <Time, Value> &out,
                      UpdatedPort <Time, Value> &in)
        const override final
    {
        (void)out;

        if (not m_variable.y.empty()) {
            copy_values(m_variable.y[0], m_integrator.x[0]);
            in.emplace(&m_integrator);
        }

        if (not m_integrator.y.empty()) {
            copy_values(m_integrator.y[0], m_variable.x[0]);
            copy_values(m_integrator.y[0], CoupledModel <Time, Value>::y[0]);

            in.emplace(&m_variable);
        }

        if (not CoupledModel <Time, Value>::x.empty()) {
            copy_port_values(CoupledModel <Time, Value>::x, m_variable.x);

            in.emplace(&m_variable);
        }
    }
};

}}}

#endif
