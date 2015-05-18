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
#include <vle/dsde/qss-common.hpp>
#include <boost/math/special_functions/sign.hpp>

namespace vle {
namespace dsde {
namespace qss {

using port = double;

template <typename Time, typename Container>
class StaticFunction : public AtomicModel <Time, port>
{
public:
    typedef AtomicModel <Time, port> parent_type;
    typedef typename parent_type::time_format time_format;
    typedef typename parent_type::time_type time_type;
    typedef typename parent_type::value_type value_type;

    StaticFunction(const Context &ctx, std::size_t system_size,
                   Function <Time, Container> function_)
        : AtomicModel <Time, port>(ctx, system_size, 1u)
        , m_integrate_function(function_)
        , m_variable(system_size)
        , m_sigma(Time::infinity())
    {
        if (system_size == 0ul)
            throw std::invalid_argument(
                "vle::dsde::qss1::StaticFunction: bad system size");
    }

    virtual time_type init(const vle::Common &/*common*/,
                           const time_type &/*t*/) override final
    {
        return Time::infinity();
    }

    virtual void lambda() const override final
    {
        AtomicModel <Time, port>::y[0].emplace_back(
            m_integrate_function(m_variable, 0.0));
    }

    virtual time_type delta(const time_type &/*e*/, const time_type &/*r*/,
                            const time_type &/*t*/) override final
    {
        m_sigma = Time::infinity();

        for (auto i = 0ul, end = AtomicModel <Time, port>::x.size();
             i != end; ++i) {
            if (not AtomicModel <Time, port>::x[i].empty()) {
                m_variable[i] = AtomicModel <Time, port>::x[i].front();
                m_sigma = Time::null();
            }
        }

        return m_sigma;
    }

private:
    Function <Time, Container> m_integrate_function;
    Container m_variable;
    time_type m_sigma;
};

template <typename Time>
class Integrator : public AtomicModel <Time, port>
{
public:
    typedef AtomicModel <Time, port> parent_type;
    typedef typename parent_type::time_format time_format;
    typedef typename parent_type::time_type time_type;
    typedef typename parent_type::value_type value_type;

    Integrator(const Context &ctx, double dq, double epsilon, double x)
        : AtomicModel <Time, port>(ctx, 1u, 1u)
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
        parent_type::y[0].emplace_back(m_q + m_dq * boost::math::sign(m_dX));
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
            auto xv = parent_type::x[0].front();
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

template <typename Time, typename Container>
class EquationBlock : public CoupledModel <Time, port>
{
public:
    typedef CoupledModel <Time, port> parent_type;
    typedef typename parent_type::time_format time_format;
    typedef typename parent_type::time_type time_type;
    typedef typename parent_type::value_type value_type;

    EquationBlock(const vle::Context &ctx,
                  double dq,
                  double epsilon,
                  double x,
                  std::size_t system_size,
                  std::size_t id,
                  Function <Time, Container> function_)
        : CoupledModel <Time, port>(ctx, system_size, 1u)
        , m_integrator(ctx, dq, epsilon, x)
        , m_variable(ctx, system_size, function_)
        , m_id(id)
    {
        if (id >= system_size)
            throw std::invalid_argument(
                "vle::dsde::qss1::EquationBlock: bad id or system size");
    }

    virtual typename parent_type::children_t
    children(const vle::Common &) override final
    {
        return { &m_integrator, &m_variable };
    }

    virtual void post(const UpdatedPort <Time, port> &/*out*/,
                      UpdatedPort <Time, port> &in) const override final
    {
        if (not m_variable.y.empty()) {
            copy_values(m_variable.y[0], m_integrator.x[0]);
            in.emplace(&m_integrator);
        }

        if (not m_integrator.y.empty()) {
            copy_values(m_integrator.y[0], m_variable.x[m_id]);
            copy_values(m_integrator.y[0], CoupledModel <Time, port>::y[0]);
            in.emplace(&m_variable);
        }

        if (not CoupledModel <Time, port>::x.empty()) {
            copy_port_values(CoupledModel <Time, port>::x, m_variable.x);
            in.emplace(&m_variable);
        }
    }

    constexpr inline double value() const noexcept
    {
        return m_integrator.value();
    }

private:
    Integrator <Time> m_integrator;
    StaticFunction <Time, Container> m_variable;
    std::size_t m_id;
};

template <typename Time, typename Container>
class Equation : public AtomicModel <Time, port>
{
public:
    typedef AtomicModel <Time, port> parent_type;
    typedef typename parent_type::time_format time_format;
    typedef typename parent_type::time_type time_type;
    typedef typename parent_type::value_type value_type;

    Equation(const Context &ctx,
             double dq,
             double epsilon,
             double x,
             std::size_t system_size,
             std::size_t id,
             Function <Time, Container> function)
        : AtomicModel <Time, port>(ctx, system_size, 1u)
        , m_integrate_function(function)
        , m_variables(system_size)
        , m_sigma(Time::null())
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
    }

    virtual time_type init(const vle::Common &/*common*/,
                           const time_type &/*t*/) override final
    {
        m_variables[m_id] = m_q;
        return m_sigma;
    }

    virtual void lambda() const override final
    {
        parent_type::y[0].emplace_back(m_q + m_dq * boost::math::sign(m_dX));
    }

    virtual time_type delta(const time_type &e, const time_type &r,
                            const time_type &t) override final
    {
        bool perturb = false;

        for (std::size_t i = 0, end = parent_type::x.size(); i != end; ++i) {
            if (!parent_type::x[i].empty()) {
                m_variables[i] = parent_type::x[i].front();
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
    Function <Time, Container> m_integrate_function;
    Container m_variables;
    time_type m_sigma;
    double m_dq;
    double m_epsilon;
    double m_X;
    double m_dX;
    double m_q;
    std::size_t m_id;
};

template <typename Time, typename Container>
class Block : public AtomicModel <Time, port>
{
public:
    typedef AtomicModel <Time, port> parent_type;
    typedef typename parent_type::time_format time_format;
    typedef typename parent_type::time_type time_type;
    typedef typename parent_type::value_type value_type;

    Block(const Context &ctx, const Container &dq, const Container &epsilon,
          const Container &x, Functions <Time, Container> function)
        : AtomicModel <Time, port>(ctx, x.size(), x.size())
        , m_integrate_function(function)
        , m_variables(x.size(), 0.0)
        , m_X(x)
        , m_dX(x.size(), 0.0)
        , m_q(x.size(), 0.0)
        , m_dq(dq)
        , m_epsilon(epsilon)
        , m_sigma(Time::null())
    {
        if (m_X.size() != m_dq.size() or m_X.size() != m_epsilon.size())
            throw std::invalid_argument("vle::dsde::qss::Block: bad size");
    }

    virtual time_type
    init(const vle::Common &common, const time_type &t) override final
    {
        (void)common;

        for (auto i = 0ul, e = m_X.size(); i != e; ++i) {
            m_q[i] = (std::floor(m_X[i] / m_dq[i]) * m_dq[i]);
            m_variables[i] = m_q[i] + m_dq[i] * boost::math::sign(m_dX[i]);
        }

        m_sigma = update(0.0, t);

        return m_sigma;
    }

    time_type update(const time_type elapsed_time, const time_type t)
    {
        Container xv(m_variables.size(), 0.0);
        m_integrate_function(m_variables, xv, t);
        auto newsigma = Time::infinity();
        auto minsigma = Time::infinity();

        for (auto i = 0ul, end = m_variables.size(); i != end; ++i) {
            m_X[i] += (elapsed_time * m_dX[i]);

            if (xv[i] > 0.0) {
                newsigma = (m_q[i] + m_dq[i] - m_X[i]) / xv[i];
            } else if (xv[i] < 0.0) {
                newsigma = (m_q[i] - m_epsilon[i] - m_X[i]) / xv[i];
            } else {
                newsigma = 0.0;
            }

            minsigma = std::min(minsigma, newsigma);
            m_dX[i] = xv[i];
        }

        return minsigma;
    }

    virtual void
    lambda() const override final
    {
        for (auto i = 0ul, e = m_X.size(); i != e; ++i)
            parent_type::y[i].emplace_back(
                m_q[i] + m_dq[i] * boost::math::sign(m_dX[i]));
    }

    virtual time_type
    delta(const time_type &e, const time_type &r,
          const time_type &t) override final
    {
        // bool perturb = false;

        // for (auto i = 0ul, end = parent_type::x.size(); i != end; ++i) {
        //     if (!parent_type::x[i].empty()) {
        //         m_variables[i] = parent_type::x[i].front();
        //         perturb = true;
        //     }
        // }

        std::cout << "e=" << e << " r=" << r << " t=" << t << "\n";

        auto newsigma = Time::infinity();
        m_sigma = Time::infinity();

        m_sigma = update(e, t);


        for (auto i = 0ul, end = m_variables.size(); i != end; ++i) {
            m_X[i] += (m_sigma * m_dX[i]);

            if (m_dX[i] > 0.0) {
                newsigma = m_dq[i] / m_dX[i];
                m_q[i] = m_q[i] + m_dq[i];
            } else if (m_dX[i] < 0.0) {
                newsigma = - m_dq[i] / m_dX[i];
                m_q[i] = m_q[i] - m_dq[i];
            } else {
                newsigma = Time::infinity();
            }

            m_sigma = std::min(m_sigma, newsigma);
        }

        show();
        std::cout << m_sigma << "\n";
        return m_sigma;
    }

    inline typename Container::value_type value(std::size_t i) const noexcept
    {
        return m_X[i];
    }

private:
    Functions <Time, Container> m_integrate_function;
    Container m_variables;
    Container m_X;
    Container m_dX;
    Container m_q;
    Container m_dq;
    Container m_epsilon;
    time_type m_sigma;

    void show()
    {
        std::cout << "[ variables:";

        for (auto x : m_variables)
            std::cout << x << " ";

        std::cout << "] [ m_X:";

        for (auto x : m_X)
            std::cout << x << " ";

        std::cout << "]\n";
    }
};

}
}
}

#endif
