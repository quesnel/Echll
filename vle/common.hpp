/*
 * Copyright (C) 2013-2014 INRA
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

#ifndef __VLE_KERNEL_COMMON_HPP__
#define __VLE_KERNEL_COMMON_HPP__

#include <memory>
#include <unordered_map>
#include <string>
#include <vle/utils.hpp>
#include <boost/any.hpp>
#include <boost/optional.hpp>

namespace vle {

/**
 * @brief @e vle::Common stores experimental conditions, simulation engine
 * parameters etc.
 */
typedef std::unordered_map <std::string, boost::any> Common;

template <typename T>
    T common_get(const Common& c, const std::string& name)
    {
        auto it = c.find(name);
        if (it == c.end())
            throw std::invalid_argument(
                stringf("Common: failed to find attribute %s", name.c_str()));

        return boost::any_cast <T>(it->second);
    }

template <typename T>
    const T* common_get_pointer(const Common& c, const std::string& name)
    {
        auto it = c.find(name);
        if (it == c.end())
            throw std::invalid_argument(
                stringf("Common: failed to find attribute %s", name.c_str()));

        return boost::any_cast <T>(&it->second);
    }

template <typename T>
boost::optional <T> get(const Common &c, const std::string &name)
{
    auto it = c.find(name);
    if (it == c.end())
        return boost::none;
    else
        return boost::any_cast <T>(it->second);
}

typedef std::shared_ptr <Common> CommonPtr;

}

#endif
