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

#ifndef ORG_VLEPROJECT_KERNEL_QSS_COMMON_HPP
#define ORG_VLEPROJECT_KERNEL_QSS_COMMON_HPP

#include <functional>

namespace vle {

/**
 * @brief Return the nan value for double.
 *
 * @return return the double representation of nan.
 */
inline static constexpr double nan() noexcept
{
    return std::numeric_limits <double>::quiet_NaN();
}

/**
 * @brief Define a function for a single equation.
 *
 * @code{.cpp}
 * void compute_prey(const std::vector<double>& x, const double t)
 * {
 *     return x[1] * 0.05;
 * }
 *
 * struct prey {
 *     double m_const;
 *
 *     prey(double const)
 *         : m_const(const)
 *     {}
 *
 *     double operator()(const std::vector<double>& x, const double t) const
 *     {
 *         return x[1] * m_const;
 *     }
 *  };
 * @endcode
 */
template <typename Time, typename Container>
using Function = std::function <typename Container::value_type(
    const Container&,
    const typename Time::time_type &)>;

/**
 * @brief Define a function for a single equation.
 *
 * @code{.cpp}
 * void compute_prey(const std::vector<double>& x,
 *                   std::vector<double>& dxdt,const double t)
 * {
 *     dxdt[0] = x[1] * 0.05;
 *     dxdt[1] = 0.25 * x[0] - 0.03 * x[1];
 * }
 *
 * struct prey {
 *     double m_const;
 *
 *     prey(double const)
 *         : m_const(const)
 *     {}
 *
 *     void operator()(const std::vector<double>& x,
 *                     std::vector<double>& dxdt,
 *                     const double t) const
 *     {
 *         dxdt[0] = x[1] * 0.05;
 *         dxdt[1] = 0.25 * x[0] - 0.03 * x[1];
 *     }
 *  };
 * @endcode
 */
template <typename Time, typename Container>
using Functions = std::function <void(
    const Container&,
    Container&,
    const typename Time::time_type &)>;

}

#endif
