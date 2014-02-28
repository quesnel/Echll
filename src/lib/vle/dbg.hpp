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

#ifndef __VLE_KERNEL_DBG_HPP__
#define __VLE_KERNEL_DBG_HPP__

#include <string>
#include <stdexcept>

namespace vle {

struct assert_error: std::logic_error
{
    explicit assert_error(const std::string& condition,
                          const std::string& line,
                          const std::string& file)
        : std::logic_error(" Assertion `" + condition + "' failed in file "
                           + file + " at line " + line)
    {}
};

}

#ifdef VLE_NDEBUG
#define _WHITE ""
#define _RED ""
#define _YELLOW ""
#define _CYAN ""
#define _NORMAL ""
#define dError(...)
#define dWarning(...)
#define dInfo(...)
#define dPrint(...)
#define dAssert(x)
#else
#define _WHITE "\x1b[37;1m"
#define _RED "\x1b[31;1m"
#define _YELLOW "\x1b[33;1m"
#define _CYAN "\x1b[36;1m"
#define _NORMAL "\x1b[0;m"
#define dError(...) do { vle::dbg_print(vle::DBG_LEVEL_ERROR, __VA_ARGS__); } while (0)
#define dWarning(...) do { vle::dbg_print(vle::DBG_LEVEL_WARNING, __VA_ARGS__); } while (0)
#define dInfo(...) do { vle::dbg_print(vle::DBG_LEVEL_INFO, __VA_ARGS__); } while (0)
#define dPrint(...) do { vle::dbg_print(vle::DBG_LEVEL_NORMAL, __VA_ARGS__); } while (0)
#define dAssert(condition_) \
    do { \
        if (!(condition_)) { \
            dError("Assertion: ", _YELLOW, "`", #condition_, "'", _RED, \
                   " failed (value: ", _YELLOW, "`", (condition_), "')", _RED, \
                   " in file ", _YELLOW, __FILE__, _RED, " at line ", \
                   _YELLOW, __LINE__, _RED, "\n      in function: ", \
                   _YELLOW, __PRETTY_FUNCTION__); \
            throw vle::assert_error(#condition_, std::to_string(__LINE__), \
                                    __FILE__); \
        } \
    } while (0)
#endif

#include <iostream>

namespace vle {

enum dbg_level
{
    DBG_LEVEL_ERROR,
    DBG_LEVEL_WARNING,
    DBG_LEVEL_INFO,
    DBG_LEVEL_NORMAL
};

//
// a variadic template print function:
// http://stackoverflow.com/questions/14251038/debug-macros-in-c
//
template <typename... ArgTypes>
inline void dbg_print(const ArgTypes&... args)
{
    // trick to expand variadic argument pack without recursion
    using expand_variadic_pack = int[];

    // first zero is to prevent empty braced-init-list
    // void() is to prevent overloaded operator, messing things up
    // trick is to use the side effect of list-initializer to call a function
    // on every argument.
    // (void) is to suppress "statement has no effect" warnings
    (void)expand_variadic_pack{0, ((std::cerr << args), void(), 0)... };
}

template <typename... ArgTypes>
inline void dbg_print(dbg_level level, ArgTypes... args)
{
    switch (level) {
    case DBG_LEVEL_ERROR:
        std::cerr << _RED << "[ERR] ";
        break;
    case DBG_LEVEL_WARNING:
        std::cerr << _YELLOW << "[WRN] ";
        break;
    case DBG_LEVEL_INFO:
        std::cerr << _CYAN << "[INF] ";
        break;
    case DBG_LEVEL_NORMAL:
        std::cerr << _NORMAL;
        break;
    }

    dbg_print(args...);

    std::cerr << _NORMAL << std::endl;
}

}

#endif
