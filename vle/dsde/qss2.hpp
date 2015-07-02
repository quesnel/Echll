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

#ifndef ORG_VLEPROJECT_KERNEL_DSDE_QSS2_HPP
#define ORG_VLEPROJECT_KERNEL_DSDE_QSS2_HPP

#include <vle/dsde/dsde.hpp>
#include <vle/qss-common.hpp>
#include <boost/math/special_functions/sign.hpp>

namespace vle {
namespace dsde {
namespace qss2 {

struct var {
    constexpr var() noexcept
        : v(nan())
        , mv(nan())
    {}

    double v;
    double mv;
};

template <std::size_t N>
class inputport
{
public:
    constexpr inputport() noexcept
        : m_dirac(false)
    {
    }

    constexpr const var &operator[](std::size_t i) const noexcept
    {
        return m_value[i];
    }

    constexpr var &operator[](std::size_t i) noexcept
    {
        return m_value[i];
    }

    constexpr void dirac() noexcept
    {
        m_dirac = true;
    }

    constexpr void clear() noexcept
    {
        m_value.fill(var());
        m_dirac = false;
    }

    constexpr std::size_t size() const noexcept
    {
        return N;
    }

    constexpr bool empty() const noexcept
    {
        if (m_dirac)
            return false;

        for (auto val : m_value)
            if (not std::isnan(val.v))
                return false;

        return true;
    }

    std::array <var, N> m_value;
    bool m_dirac;
};

class doubleport
{
public:
    constexpr const double &operator[](std::size_t i) const noexcept
    {
        return (i == 0) ? m_value.v : m_value.mv;
    }

    constexpr double &operator[](std::size_t i) noexcept
    {
        return (i == 0) ? m_value.v : m_value.mv;
    }

    constexpr std::size_t size() const noexcept
    {
        return 1u;
    }

    constexpr double v() const noexcept
    {
        return m_value.v;
    }

    constexpr double mv() const noexcept
    {
        return m_value.mv;
    }

    constexpr void clear() noexcept
    {
        m_value.v = nan();
        m_value.mv = nan();
    }

    bool empty() const noexcept
    {
        return std::isnan(m_value.v);
    }

    var m_value;
};

template <typename Time, std::size_t N>
class StaticFunction : public AtomicModel <Time, inputport <N>, doubleport>
{
public:
    using inputport_type = inputport <N>;
    using outputport_type = doubleport;
    using parent_type = AtomicModel <Time, inputport_type, outputport_type>;
    using time_format = typename parent_type::time_format;
    using time_type = typename parent_type::time_type;
    using array_type = std::array <double, N>;
    using function_type = Function <Time, array_type>;

    StaticFunction(const Context &ctx,
                   Function <Time, array_type> function_)
        : parent_type(ctx)
        , m_integrate_function(function_)
    {
        static_assert(N > 0,
                      "vle::dsde::qss2::StaticFunction: bad system size");
    }

    virtual time_type init(const vle::Common &/*common*/,
                           const time_type &t) override final
    {
        m_variable.fill(0);
        m_mv.fill(0);
        m_time = t;
        return Time::infinity();
    }

    virtual void lambda() const override final
    {
        parent_type::y.m_value = m_y;
    }

    virtual time_type delta(const time_type &e, const time_type &/*r*/,
                            const time_type &t) override final
    {
        if (not parent_type::x.empty()) {
            m_time = t;

            for (auto i = 0ul; i < N; ++i)
                m_variable[i] += m_mv[i] * e;

            double fv = m_integrate_function(m_variable, m_time);
            array_type vaux;

            for (auto i = 0ul; i < N; ++i) {
                if (not std::isnan(parent_type::x[i].v)) {
                    vaux[i] = m_variable[i];
                    m_variable[i] = parent_type::x[i].v;
                    m_mv[i] = parent_type::x[i].mv;
                } else {
                    vaux[i] = nan();
                }
            }

            m_y.v = m_integrate_function(m_variable, m_time);

            for (auto i = 0ul; i < N; ++i) {
                if (not std::isnan(vaux[i]) and vaux[i] != m_variable[i])
                    m_c[i] = (fv - m_y.v) / (vaux[i] - m_variable[i]);
            }

            m_y.mv = 0;

            for (auto i = 0ul; i != N; ++i)
                m_y.mv += m_mv[i] * m_c[i];

            return 0;
        }

        return Time::infinity();
    }

private:
    array_type m_variable;
    array_type m_mv;
    array_type m_c;
    var m_y;
    Function <Time, array_type> m_integrate_function;
    time_type m_time;
};

template <typename Time, std::size_t N>
class Integrator : public AtomicModel <Time, inputport <N>, doubleport>
{
public:
    using inputport_type = inputport <N>;
    using outputport_type = doubleport;
    using parent_type = AtomicModel <Time, inputport_type, outputport_type>;
    using time_format = typename parent_type::time_format;
    using time_type = typename parent_type::time_type;

    Integrator(const Context &ctx, double dq, double x)
        : parent_type(ctx)
        , m_dx(0)
        , m_mdx(0)
        , m_x(x)
        , m_q(x)
        , m_mq(0)
        , m_dq(dq)
        , m_sigma(0)
    {}

    virtual time_type init(const vle::Common &/*common*/,
                           const time_type &/*time*/) override final
    {
        return m_sigma;
    }

    virtual void lambda() const override final
    {
        parent_type::y[0] = m_x + m_dx * m_sigma + m_mdx / 2. * m_sigma * m_sigma;
        parent_type::y[1] = m_dx + m_mdx * m_sigma;
    }

    virtual time_type delta(const time_type &e, const time_type &/*r*/,
                            const time_type &/*t*/) override final
    {
        if (parent_type::x.empty()) {
            m_x += m_dx * m_sigma + m_mdx / 2. * m_sigma * m_sigma;
            m_q = m_x;
            m_dx += m_mdx * m_sigma;
            m_mq = m_dx;

            if (m_mdx == 0)
                m_sigma = Time::infinity();
            else
                m_sigma = std::sqrt(2. * m_dq / std::abs(m_mdx));
        } else {
            double a, b, c, s;
            m_x += m_dx * e + m_mdx / 2. * e * e;
            m_dx = parent_type::x[0].v;
            m_mdx = parent_type::x[0].mv;

            if (m_sigma != 0) {
                m_q = m_q + m_mq * e;
                a = m_mdx / 2.;
                b = m_dx - m_mq;
                c = m_x - m_q + m_dq;
                m_sigma = Time::infinity();

                if (a == 0) {
                    if (b != 0) {
                        s = -c / b;

                        if (s > 0)
                            m_sigma = s;

                        c = m_x - m_q - m_dq;
                        s = -c / b;

                        if (s > 0 and s < m_sigma)
                            m_sigma = s;
                    }
                } else {
                    s = (-b + std::sqrt(b * b - 4. * a * c)) / 2. / a;

                    if (s > 0)
                        m_sigma = s;

                    s = (-b - std::sqrt(b * b - 4. * a * c)) / 2. / a;

                    if (s > 0 and s < m_sigma)
                        m_sigma = s;

                    c = m_x - m_q - m_dq;
                    s = (-b + std::sqrt(b * b - 4. * a * c)) / 2. / a;

                    if (s > 0 and s < m_sigma)
                        m_sigma = s;

                    s = (-b - std::sqrt(b * b - 4. * a * c)) / 2. / a;

                    if (s > 0 and s < m_sigma)
                        m_sigma = s;
                }
            }
        }

        return m_sigma;
    }

    constexpr inline double value() const noexcept
    {
        return m_x;
    }

private:
    std::array <double, 2> m_y;
    double m_dx;
    double m_mdx;
    double m_x;
    double m_q;
    double m_mq;
    double m_dq;
    time_type m_sigma;
};

template <typename Time, std::size_t N>
class EquationBlock : public CoupledModel <Time,
    inputport <N>, doubleport,
    inputport <N>, doubleport>
{
public:
    using inputport_type = inputport <N>;
    using outputport_type = doubleport;

    using parent_type = CoupledModel
                        <Time, inputport_type, outputport_type, inputport_type, outputport_type>;
    using time_format = typename parent_type::time_format;
    using time_type = typename parent_type::time_type;
    using array_type = std::array <double, N>;

    using UpdatedPort = typename parent_type::UpdatedPort;

    EquationBlock(const vle::Context &ctx,
                  double dq,
                  double x,
                  std::size_t id,
                  Function <Time, array_type> function_)
        : parent_type(ctx)
        , m_integrator(ctx, dq, x)
        , m_staticfunction(ctx, function_)
        , m_id(id)
    {
        if (id >= N)
            throw std::invalid_argument(
                "vle::dsde::qss2::EquationBlock: bad id or system size");
    }

    virtual typename parent_type::children_t
    children(const vle::Common &) override final
    {
        return { &m_integrator, &m_staticfunction };
    }

    virtual void post(const UpdatedPort &/*out*/,
                      UpdatedPort &in) const override final
    {
        if (not m_staticfunction.y.empty()) {
            m_integrator.x[0] = m_staticfunction.y.m_value;
            in.emplace(&m_integrator);
        }

        if (not m_integrator.y.empty()) {
            m_staticfunction.x[m_id] = m_integrator.y.m_value;
            parent_type::y.m_value = m_integrator.y.m_value;
            in.emplace(&m_staticfunction);
        }

        for (auto i = 0ul; i != N; ++i) {
            if (!std::isnan(parent_type::x[i].v)) {
                m_staticfunction.x[i] = parent_type::x[i];
                in.emplace(&m_staticfunction);
            }
        }
    }

    constexpr inline double value() const noexcept
    {
        return m_integrator.value();
    }

private:
    Integrator <Time, N> m_integrator;
    StaticFunction <Time, N> m_staticfunction;
    std::size_t m_id;
};


}
}
}

#endif
