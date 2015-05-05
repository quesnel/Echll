/*
 * Copyright (C) 2014 INRA
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

#ifndef ORG_VLEPROJECT_KERNEL_TIME_HPP
#define ORG_VLEPROJECT_KERNEL_TIME_HPP

#include <limits>

namespace vle {

/**
 * @class Time The reprensetation of the simulated time in model.
 *
 * The template parameter T defines (i) the type of time and (ii) several
 * constants to be use. The type and constants are used in many place into
 * kernel and user land.
 */
template <typename T>
struct Time
{
    typedef typename T::time_type time_type;

    static constexpr time_type negative_infinity()
    {
        return T::negative_infinity();
    }

    static constexpr time_type infinity()
    {
        return T::infinity();
    }

    static constexpr time_type null()
    {
        return T::null();
    }

    static constexpr bool is_negative_infinity(const time_type &t)
    {
        return T::is_negative_infinity(t);
    }

    static constexpr bool is_infinity(const time_type &t)
    {
        return T::is_infinity(t);
    }

    static constexpr bool is_null(const time_type &t)
    {
        return T::is_null(t);
    }
};

template <typename Type>
struct StandardConstants
{
    typedef Type time_type;

    static constexpr time_type negative_infinity()
    {
        return -std::numeric_limits <time_type>::infinity();
    }

    static constexpr time_type infinity()
    {
        return std::numeric_limits <time_type>::infinity();
    }

    static constexpr time_type null()
    {
        return time_type(0);
    }

    static constexpr bool is_negative_infinity(const time_type &t)
    {
        return t == StandardConstants <Type>::negative_infinity();
    }

    static constexpr bool is_infinity(const time_type &t)
    {
        return t == StandardConstants <Type>::infinity();
    }

    static constexpr bool is_null(const time_type &t)
    {
        return t == StandardConstants <Type>::null();
    }
};

using FloatTime = Time <StandardConstants <float>>;
using DoubleTime = Time <StandardConstants <double>>;
using LongDoubleTime = Time <StandardConstants <long double>>;

}

#endif
