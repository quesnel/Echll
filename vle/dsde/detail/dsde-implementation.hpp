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

#ifndef ORG_VLEPROJECT_KERNEL_DSDE_DETAIL_DSDE_IMPLEMENTATION_HPP
#define ORG_VLEPROJECT_KERNEL_DSDE_DETAIL_DSDE_IMPLEMENTATION_HPP

namespace vle {
namespace dsde {

dsde_internal_error::dsde_internal_error(const std::string& msg)
    : std::logic_error(msg)
{}

dsde_internal_error::~dsde_internal_error() noexcept
{}

template <typename Time>
inline void check_transition_synchronization(typename Time::time_type tl,
                                             typename Time::time_type time,
                                             typename Time::time_type tn)
{
#ifndef VLE_OPTIMIZE
    if (!(tl <= time && time <= tn))
        throw dsde_internal_error("Synchronization error");
#else
    (void)tl;
    (void)time;
    (void)tn;
#endif
}

template <typename Time>
inline void check_output_synchronization(typename Time::time_type tn,
                                         typename Time::time_type time)
{
#ifndef VLE_OPTIMIZE
    if (time != tn)
        throw dsde_internal_error("Synchronization error");
#else
    (void)tn;
    (void)time;
#endif
}

}}

#endif
