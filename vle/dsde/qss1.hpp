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

#ifndef ORG_VLEPROJECT_KERNEL_DSDE_QSS1_HPP
#define ORG_VLEPROJECT_KERNEL_DSDE_QSS1_HPP

#include <vle/dsde/dsde.hpp>
#include <valarray>

namespace vle { namespace dsde { namespace qss1 {

class internal_error : std::runtime_error
{
public:
    internal_error(const std::string &msg);

    internal_error(const internal_error&) = default;

    virtual ~internal_error() noexcept;
};

internal_error::internal_error(const std::string &msg)
    : std::runtime_error(msg)
{}

internal_error::~internal_error() noexcept
{}

typedef double port;

typedef std::function <
    double(const std::valarray <double>& cst,
           const std::valarray <double>& v)> compute_fct;

template <typename Time>
struct Variable : AtomicModel <Time, port>
{
    typedef AtomicModel <Time, port> parent_type;
    typedef typename parent_type::time_format time_format;
    typedef typename parent_type::time_type time_type;
    typedef typename parent_type::value_type value_type;

    compute_fct computer;

    const std::valarray <double> c;
    std::valarray <double> u;
    time_type sigma;

    Variable(const Context &ctx,
             std::size_t system_size,
             compute_fct computer_,
             const std::valarray <double>& constants)
        : AtomicModel <Time, port>(ctx, system_size, 1u)
        , computer(computer_)
        , c(constants)
        , u(system_size)
        , sigma(Time::infinity())
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
        AtomicModel <Time, port>::y[0] = { computer(c, u) };
    }

    virtual time_type delta(const time_type& e, const time_type& r,
                            const time_type& t)
    {
        (void)e;
        (void)r;
        (void)t;

        sigma = Time::infinity();

        for (std::size_t i = 0, end = AtomicModel <Time, port>::x.size(); i != end; ++i) {
            if (not AtomicModel <Time, port>::x[i].empty()) {
                u[i] = AtomicModel <Time, port>::x[i].front();
                sigma = Time::null();
            }
        }

        return sigma;
    }
};

template <typename Time>
struct Integrator : AtomicModel <Time, port>
{
    typedef AtomicModel <Time, port> parent_type;
    typedef typename parent_type::time_format time_format;
    typedef typename parent_type::time_type time_type;
    typedef typename parent_type::value_type value_type;

    time_type sigma;
    double dq, epsilon, X, dX, q;

    Integrator(const Context &ctx, double dq_, double epsilon_, double x_)
        : AtomicModel <Time, port>(ctx, 1u, 1u)
        , sigma(Time::null())
        , dq(dq_)
        , epsilon(epsilon_)
        , X(x_)
        , dX(0.0)
        , q(std::floor(X / dq) * dq)
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
        if (dX == 0.0) {
            parent_type::y[0] = { q };
        } else {
            parent_type::y[0] = { q + dq * dX / std::abs(dX) };
        }
    }

    virtual time_type delta(const time_type& e, const time_type& r,
                            const time_type& t)
    {
        (void)r;
        (void)t;

        if (parent_type::x.empty()) {
            X += sigma * dX;

            if (dX > 0.0) {
                sigma = dq / dX;
                q = q + dq;
            } else if (dX < 0.0) {
                sigma = -dq / dX;
                q = q - dq;
            } else {
                sigma = Time::infinity();
            }
        } else {
            double xv = parent_type::x[0].front();
            X += dX * e;

            if (xv > 0.0) {
                sigma = (q + dq - X) / xv;
            } else if (xv < 0.0) {
                sigma = (q - epsilon - X) / xv;
            } else {
                sigma = Time::infinity();
            }

            dX = xv;
        }

        return sigma;
    }
};

template <typename Time>
struct Equation : CoupledModel <Time, port>
{
    typedef CoupledModel <Time, port> parent_type;
    typedef typename parent_type::time_format time_format;
    typedef typename parent_type::time_type time_type;
    typedef typename parent_type::value_type value_type;

    Integrator <Time> m_integrator;
    Variable <Time> m_variable;

    Equation(const vle::Context &ctx,
             double dq,
             double epsilon,
             double x,
             std::size_t system_size,
             compute_fct compute,
             const std::valarray <double>& constants)
        : CoupledModel <Time, port>(ctx, system_size, 1u)
        , m_integrator(ctx, dq, epsilon, x)
        , m_variable(ctx, system_size, compute, constants)
    {
    }

    virtual ~Equation()
    {
    }

    virtual typename parent_type::children_t
    children(const vle::Common&) override final
    {
        return { &m_integrator, &m_variable };
    }

    virtual void post(const UpdatedPort <Time, port> &out,
                      UpdatedPort <Time, port> &in)
        const override final
    {
        (void)out;

        if (not m_variable.y.empty()) {
            copy_values(m_variable.y[0], m_integrator.x[0]);

            in.emplace(&m_integrator);
        }

        if (not m_integrator.y.empty()) {
            copy_values(m_integrator.y[0], m_variable.x[0]);
            copy_values(m_integrator.y[0], CoupledModel <Time, port>::y[0]);

            in.emplace(&m_variable);
        }

        if (not CoupledModel <Time, port>::x.empty()) {
            copy_port_values(CoupledModel <Time, port>::x, m_variable.x);

            in.emplace(&m_variable);
        }
    }
};

}}}

#endif
