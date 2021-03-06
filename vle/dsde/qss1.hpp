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
#include <vle/qss-common.hpp>
#include <boost/math/special_functions/sign.hpp>

namespace vle {
namespace dsde {
namespace qss1 {

template <std::size_t N>
class inputport
{
public:
    constexpr inputport() noexcept
        : m_dirac(false)
    {
        m_value.fill(nan());
    }

    constexpr double operator[](std::size_t i) const noexcept
    {
        return m_value[i];
    }

    constexpr double &operator[](std::size_t i) noexcept
    {
        return m_value[i];
    }

    constexpr void dirac() noexcept
    {
        m_dirac = true;
    }

    constexpr void clear() noexcept
    {
        m_value.fill(nan());
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
            if (not std::isnan(val))
                return false;

        return true;
    }

    std::array <double, N> m_value;
    bool m_dirac;
};

class doubleport
{
public:
    constexpr doubleport() noexcept
        : m_value(nan())
    {}

    constexpr const double &operator[](std::size_t) const noexcept
    {
        return m_value;
    }

    constexpr double &operator[](std::size_t) noexcept
    {
        return m_value;
    }

    constexpr std::size_t size() const noexcept
    {
        return 1u;
    }

    constexpr void clear() noexcept
    {
        m_value = nan();
    }

    bool empty() const noexcept
    {
        return std::isnan(m_value);
    }

    double m_value;
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
        , m_sigma(Time::infinity())
    {
        static_assert(N > 0,
                      "vle::dsde::qss1::StaticFunction: bad system size");
    }

    virtual time_type init(const vle::Common &/*common*/,
                           const time_type &t) override final
    {
        m_time = t;
        return Time::infinity();
    }

    virtual void lambda() const override final
    {
        parent_type::y[0] = m_integrate_function(m_variable, m_time);
    }

    virtual time_type delta(const time_type &/*e*/, const time_type &/*r*/,
                            const time_type &t) override final
    {
        m_time = t;
        m_sigma = Time::infinity();

        for (auto i = 0ul; i != N; ++i) {
            if (not std::isnan(parent_type::x[i])) {
                m_variable[i] = parent_type::x[i];
                m_sigma = Time::null();
            }
        }

        return m_sigma;
    }

private:
    array_type m_variable;
    Function <Time, array_type> m_integrate_function;
    time_type m_sigma;
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

    Integrator(const Context &ctx, double dq, double epsilon, double x)
        : parent_type(ctx)
        , m_sigma(Time::null())
        , m_dq(dq)
        , m_epsilon(epsilon)
        , m_X(x)
        , m_dX(0.0)
        , m_q(std::floor(x / dq) * dq)
    {}

    virtual time_type init(const vle::Common &/*common*/,
                           const time_type &/*time*/) override final
    {
        return m_sigma;
    }

    virtual void lambda() const override final
    {
        parent_type::y[0] = m_q + m_dq * boost::math::sign(m_dX);
    }

    virtual time_type delta(const time_type &e, const time_type &/*r*/,
                            const time_type &/*t*/) override final
    {
        if (parent_type::x.empty()) {
            m_X += (m_sigma * m_dX);

            if (m_dX > 0.0) {
                m_sigma = m_dq / m_dX;
                m_q = m_q + m_dq;
            } else if (m_dX < 0.0) {
                m_sigma = - m_dq / m_dX;
                m_q = m_q - m_dq;
            } else {
                m_sigma = Time::infinity();
            }
        } else {
            auto xv = parent_type::x[0];
            m_X += (e * m_dX);

            if (xv > 0.0) {
                m_sigma = (m_q + m_dq - m_X) / xv;
            } else if (xv < 0.0) {
                m_sigma = (m_q - m_epsilon - m_X) / xv;
            } else {
                m_sigma = Time::infinity();
            }

            m_dX = xv;
        }

        return m_sigma;
    }

    constexpr inline double value() const noexcept
    {
        return m_X;
    }

private:
    time_type m_sigma;
    double m_dq;
    double m_epsilon;
    double m_X;
    double m_dX;
    double m_q;
};

template <typename Time, std::size_t N>
class EquationBlock : public CoupledModel <Time,
    inputport <N>, doubleport,
    inputport <N>, doubleport,
    TransitionPolicyDefault <Time>>
{
public:
    using inputport_type = inputport <N>;
    using outputport_type = doubleport;

    using parent_type = CoupledModel <Time,
          inputport <N>, doubleport,
          inputport <N>, doubleport,
          TransitionPolicyDefault <Time>>;
    using time_format = typename parent_type::time_format;
    using time_type = typename parent_type::time_type;
    using array_type = std::array <double, N>;

    using UpdatedPort = typename parent_type::UpdatedPort;

    EquationBlock(const vle::Context &ctx,
                  double dq,
                  double epsilon,
                  double x,
                  std::size_t id,
                  Function <Time, array_type> function_)
        : parent_type(ctx)
        , m_integrator(ctx, dq, epsilon, x)
        , m_staticfunction(ctx, function_)
        , m_id(id)
    {
        if (id >= N)
            throw std::invalid_argument(
                "vle::dsde::qss1::EquationBlock: bad id or system size");
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
            m_integrator.x[0] = m_staticfunction.y[0];
            in.emplace(&m_integrator);
        }

        if (not m_integrator.y.empty()) {
            m_staticfunction.x[m_id] = m_integrator.y[0];
            parent_type::y[0] = m_integrator.y[0];
            in.emplace(&m_staticfunction);
        }

        for (auto i = 0ul; i != N; ++i) {
            if (!std::isnan(parent_type::x[i])) {
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

template <typename Time, std::size_t N>
class Equation : public AtomicModel <Time, inputport <N>, doubleport>
{
public:
    using inputport_type = inputport <N>;
    using outputport_type = doubleport;
    using parent_type = AtomicModel <Time, inputport_type, outputport_type>;
    using time_format = typename parent_type::time_format;
    using time_type = typename parent_type::time_type;
    using array_type = std::array <double, N>;
    using function_type = Function <Time, array_type>;

    Equation(const Context &ctx,
             double dq,
             double epsilon,
             double x,
             std::size_t id,
             Function <Time, array_type> function)
        : parent_type(ctx)
        , m_integrate_function(function)
        , m_sigma(Time::null())
        , m_dq(dq)
        , m_epsilon(epsilon)
        , m_X(x)
        , m_dX(0.0)
        , m_q(std::floor(x / dq) * dq)
        , m_id(id)
    {
        static_assert(N != 0ul,
                      "vle::dsde::qss1::Equation: bad system size");

        if (id >= N)
            throw std::invalid_argument(
                "vle::dsde::qss1::Equation: bad id or system size");

        m_variables.fill(0.0);
    }

    virtual time_type init(const vle::Common &/*common*/,
                           const time_type &/*t*/) override final
    {
        m_variables[m_id] = m_q;
        return m_sigma;
    }

    virtual void lambda() const override final
    {
        parent_type::y[0] = m_q + m_dq * boost::math::sign(m_dX);
    }

    virtual time_type delta(const time_type &e, const time_type &r,
                            const time_type &t) override final
    {
        bool perturb = false;

        for (std::size_t i = 0; i != N; ++i) {
            if (not std::isnan(parent_type::x[i])) {
                m_variables[i] = parent_type::x[i];
                perturb = true;
            }
        }

        if (!perturb && r == 0.0) {
            m_X += (m_sigma * m_dX);

            if (m_dX > 0.0) {
                m_sigma = m_dq / m_dX;
                m_q = m_q + m_dq;
            } else if (m_dX < 0.0) {
                m_sigma = - m_dq / m_dX;
                m_q = m_q - m_dq;
            } else {
                m_sigma = Time::infinity();
            }
        } else {
            m_variables[m_id] = m_q + m_dq * boost::math::sign(m_dX);
            auto xv = m_integrate_function(m_variables, t);
            m_X += (e * m_dX);

            if (xv > 0.0) {
                m_sigma = (m_q + m_dq - m_X) / xv;
            } else if (xv < 0.0) {
                m_sigma = (m_q - m_epsilon - m_X) / xv;
            } else {
                m_sigma = Time::infinity();
            }

            m_dX = xv;
        }

        return m_sigma;
    }

    constexpr inline double value() const noexcept
    {
        return m_X;
    }

private:
    array_type m_variables;
    Function <Time, array_type> m_integrate_function;
    time_type m_sigma;
    double m_dq;
    double m_epsilon;
    double m_X;
    double m_dX;
    double m_q;
    std::size_t m_id;
};

}
}
}

#endif
