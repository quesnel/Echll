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

#ifndef ORG_VLEPROJECT_KERNEL_DEVS_QSS1_HPP
#define ORG_VLEPROJECT_KERNEL_DEVS_QSS1_HPP

#include <vle/devs/devs.hpp>
#include <vle/qss-common.hpp>
#include <boost/math/special_functions/sign.hpp>

namespace vle {
namespace devs {
namespace qss {

inline static constexpr double nan() noexcept
{
    return std::numeric_limits <double>::quiet_NaN();
}

struct QssInputPort {
    QssInputPort(std::size_t size) noexcept
        : m_value(size, nan())
        , m_dirac(false)
    {}

    const double &operator[](std::size_t i) const noexcept
    {
        return m_value[i];
    }

    double &operator[](std::size_t i) noexcept
    {
        return m_value[i];
    }

    void dirac() noexcept
    {
        m_dirac = true;
    }

    void clear() noexcept
    {
        std::fill(m_value.begin(), m_value.end(), nan());
        m_dirac = false;
    }

    std::size_t size() const noexcept
    {
        return m_value.size();
    }

    bool empty() const noexcept
    {
        if (m_dirac)
            return false;

        for (auto val : m_value)
            if (not std::isnan(val))
                return false;

        return true;
    }

    std::vector <double> m_value;
    bool m_dirac;
};

struct QssOutputPort {
    constexpr QssOutputPort(double) noexcept
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

template <typename Time, typename Container>
class StaticFunction : public AtomicModel <Time, QssInputPort, QssOutputPort>
{
public:
    using parent_type = AtomicModel <Time, QssInputPort, QssOutputPort>;
    using time_format = typename parent_type::time_format;
    using time_type = typename parent_type::time_type;
    using inputport_type = typename parent_type::inputport_type;
    using outputport_type = typename parent_type::inputport_type;

    StaticFunction(const Context &ctx, std::size_t system_size,
                   Function <Time, Container> function_)
        : parent_type(ctx, system_size, nan())
        , m_integrate_function(function_)
        , m_variable(system_size)
        , m_sigma(Time::infinity())
        , m_time(Time::null())
    {
        if (system_size == 0ul)
            throw std::invalid_argument(
                "vle::dsde::qss1::StaticFunction: bad system size");
    }

    virtual time_type ta() const
    {
        return m_sigma;
    }

    virtual void lambda() const
    {
        parent_type::y[0] = m_integrate_function(m_variable, m_time);
    }

    virtual void internal()
    {
        m_sigma = Time::infinity();
    }

    virtual void external(const time_type &e)
    {
        m_time += e;
        m_sigma = Time::infinity();

        for (std::size_t i = 0ul, end = parent_type::x.size(); i != end; ++i) {
            if (not std::isnan(parent_type::x[i])) {
                m_variable[i] = parent_type::x[i];
                m_sigma = Time::null();
            }
        }
    }

private:
    Function <Time, Container> m_integrate_function;
    Container m_variable;
    time_type m_sigma;
    time_type m_time;
};

template <typename Time>
class Integrator : public AtomicModel <Time, QssInputPort, QssOutputPort>
{
public:
    using parent_type = AtomicModel <Time, QssInputPort, QssOutputPort>;
    using time_format = typename parent_type::time_format;
    using time_type = typename parent_type::time_type;

    Integrator(const Context &ctx, double dq, double epsilon, double x)
        : parent_type(ctx, 1u, nan())
        , m_sigma(Time::null())
        , m_dq(dq)
        , m_epsilon(epsilon)
        , m_X(x)
        , m_dX(0.0)
        , m_q(std::floor(x / dq) * dq)
    {}

    virtual time_type ta() const
    {
        return m_sigma;
    }

    virtual void lambda() const
    {
        parent_type::y[0] = m_q + m_dq * boost::math::sign(m_dX);
    }

    virtual void internal()
    {
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
    }

    virtual void external(const time_type &e)
    {
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

template <typename Time, typename Container>
class EquationBlock : public CoupledModel <
    Time, QssInputPort, QssOutputPort,
    QssInputPort, QssOutputPort >
{
public:
    using parent_type = CoupledModel <Time, QssInputPort, QssOutputPort,
          QssInputPort, QssOutputPort>;
    using time_format = typename parent_type::time_format;
    using time_type = typename parent_type::time_type;
    using child_type = typename parent_type::child_type;

    using UpdatedPort = typename parent_type::UpdatedPort;

    EquationBlock(const vle::Context &ctx,
                  double dq,
                  double epsilon,
                  double x,
                  std::size_t system_size,
                  std::size_t id,
                  Function <Time, Container> function_)
        : parent_type(ctx, system_size, nan())
        , m_integrator(ctx, dq, epsilon, x)
        , m_staticfunction(ctx, system_size, function_)
        , m_id(id)
    {
        if (id >= system_size)
            throw std::invalid_argument(
                "vle::dsde::qss1::EquationBlock: bad id or system size");
    }

    virtual typename parent_type::children_t
    children() override final
    {
        return { &m_integrator, &m_staticfunction };
    }

    virtual void post(const child_type *out, UpdatedPort &in) const override
    {
        if (!out) {
            for (std::size_t i = 0ul, end = parent_type::x.size(); i != end;
                ++i) {
                if (!std::isnan(parent_type::x[i])) {
                    m_staticfunction.x[i] = parent_type::x[i];
                    in.emplace(&m_staticfunction);
                }
            }
        } else if (out == &m_integrator) {
            if (not m_integrator.y.empty()) {
                m_staticfunction.x[m_id] = m_integrator.y[0];
                parent_type::y[0] = m_integrator.y[0];
                in.emplace(&m_staticfunction);
            }
        } else if (out == &m_staticfunction) {
            if (not m_staticfunction.y.empty()) {
                m_integrator.x[0] = m_staticfunction.y[0];
                in.emplace(&m_integrator);
            }
        }
    }

    virtual std::size_t select(const std::vector <child_type *> &) const override
    {
        return 0u;
    }

    constexpr inline double value() const noexcept
    {
        return m_integrator.value();
    }

private:
    Integrator <Time> m_integrator;
    StaticFunction <Time, Container> m_staticfunction;
    std::size_t m_id;
};

template <typename Time, typename Container>
class Equation : public AtomicModel <Time, QssInputPort, QssOutputPort>
{
public:
    using parent_type = AtomicModel <Time, QssInputPort, QssOutputPort>;
    using time_format = typename parent_type::time_format;
    using time_type = typename parent_type::time_type;

    Equation(const Context &ctx,
             double dq,
             double epsilon,
             double x,
             std::size_t system_size,
             std::size_t id,
             Function <Time, Container> function)
        : parent_type(ctx, system_size, nan())
        , m_integrate_function(function)
        , m_variables(system_size)
        , m_sigma(Time::null())
        , m_time(Time::null())
        , m_dq(dq)
        , m_epsilon(epsilon)
        , m_X(x)
        , m_dX(0.0)
        , m_q(std::floor(x / dq) * dq)
        , m_id(id)
    {
        if (system_size == 0ul)
            throw std::invalid_argument(
                "vle::dsde::qss1::Equation: bad system size");

        if (id >= system_size)
            throw std::invalid_argument(
                "vle::dsde::qss1::Equation: bad id or system size");

        m_variables[m_id] = m_q;
    }

    virtual time_type ta() const
    {
        return m_sigma;
    }

    virtual void lambda() const override final
    {
        parent_type::y[0] = m_q + m_dq * boost::math::sign(m_dX);
    }

    virtual void internal()
    {
        m_time += m_sigma;
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
    }

    virtual void external(const time_type &e)
    {
        m_time += e;

        for (std::size_t i = 0, end = parent_type::x.size(); i != end; ++i)
            if (not std::isnan(parent_type::x[i]))
                m_variables[i] = parent_type::x[i];

        m_variables[m_id] = m_q + m_dq * boost::math::sign(m_dX);
        auto xv = m_integrate_function(m_variables, m_time);
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

    constexpr inline double value() const noexcept
    {
        return m_X;
    }

private:
    Function <Time, Container> m_integrate_function;
    Container m_variables;
    time_type m_sigma;
    time_type m_time;
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
