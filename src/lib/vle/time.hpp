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

#ifndef __VLE_KERNEL_TIME_HPP__
#define __VLE_KERNEL_TIME_HPP__

namespace vle {

/**
 * @class Time
 * @brief Time
 *
 * The template parameter Type is the type to represent the time is the
 * simulation. The template parameter Infinity defines a structure with
 * constant representation of positive and negative infinity.
 *
 * @example
 * template <typename T>
 * struct Infinity
 * {
 *   static constexpr T negative = -std::numeric_limits<T>::infinity();
 *   static constexpr T positive = std::numeric_limits<T>::infinity();
 * };
 *
 * typedef vle::Time <double, Infinity<double>> MyTime;
 * @endexample
 */
template <typename Type, typename Infinity>
    struct Time
    {
        static constexpr Type negative_infinity = Infinity::negative;
        static constexpr Type infinity = Infinity::positive;
        typedef Type type;
    };

}

#endif
