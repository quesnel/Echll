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

#ifndef __VLE_KERNEL_CONTEXT_HPP__
#define __VLE_KERNEL_CONTEXT_HPP__

#include <boost/any.hpp>
#include <algorithm>
#include <functional>
#include <memory>
#include <cstdio>
#include <cstdarg>
#include <cstdint>
#include <cinttypes>
#include <unistd.h>

#define ECHLL_YELLOW "\x1b[33m"
#define ECHLL_RED "\x1b[31m"
#define ECHLL_NORMAL "\x1b[0m"

namespace vle {

struct ContextImpl;
typedef std::shared_ptr <ContextImpl> Context;

}

inline void vle_log_null(const vle::Context& ctx, const char *format, ...)
{
    (void)ctx;
    (void)format;
}

#if defined __GNUC__
#define VLE_PRINTF(format__, args__) __attribute__ ((format (printf, format__, args__)))
#endif

#define vle_log_cond(ctx, prio, arg...)                                 \
    do {                                                                \
        if ((ctx) && (ctx)->get_log_priority() >= prio) {               \
            (ctx)->log(prio, __FILE__, __LINE__, __PRETTY_FUNCTION__,   \
                       ## arg);                                         \
        }                                                               \
    } while (0)

#define LOG_DEBUG 3
#define LOG_INFO  2
#define LOG_ERR   1

#ifdef ENABLE_LOGGING
#  ifdef ENABLE_DEBUG
#    define vle_dbg(ctx, arg...) vle_log_cond((ctx), LOG_DEBUG, ## arg)
#  else
#    define vle_dbg(ctx, arg...) vle_log_null((ctx), ## arg)
#  endif
#  define vle_info(ctx, arg...) vle_log_cond((ctx), LOG_INFO, ## arg)
#  define vle_err(ctx, arg...) vle_log_cond((ctx), LOG_ERR, ## arg)
#else
#  define vle_dbg(ctx, arg...) vle_log_null((ctx), ## arg)
#  define vle_info(ctx, arg...) vle_log_null((ctx), ## arg)
#  define vle_err(ctx, arg...) vle_log_null((ctx), ## arg)
#endif

namespace vle {

void log_to_stderr(const ContextImpl& ctx, int priority, const char *file,
                   int line, const char *fn, const char *format, va_list args);

/**
 * Default @e ContextImpl initializes logger system with the standard error
 * output (@e stderr).
 */
struct ContextImpl
{
    ContextImpl()
        : m_log_fn(log_to_stderr)
        , m_thread_number(0)
        , m_log_priority(LOG_DEBUG)
        , m_is_a_tty(fileno(stderr))
    {}

    ContextImpl(const ContextImpl&) = default;
    ContextImpl& operator=(const ContextImpl&) = default;
    ContextImpl(ContextImpl&&) = default;
    ContextImpl& operator=(ContextImpl&&) = default;
    ~ContextImpl() = default;

    typedef std::function <void(const ContextImpl& ctx, int priority,
                                const char *file, int line, const char *fn,
                                const char *format, va_list args)> log_fn;

    void set_log_fn(log_fn fn)
    {
        m_log_fn = fn;
    }

    void log(int priority, const char *file, int line,
             const char *fn, const char *formats, ...) VLE_PRINTF(6, 7)
    {
        if (m_log_priority >= priority) {
            va_list args;
            va_start(args, formats);

            try {
                m_log_fn(*this, priority, file, line, fn, formats, args);
            } catch (...) {
            }

            va_end(args);
        }
    }

    int get_log_priority() const
    {
        return m_log_priority;
    }

    void set_log_priority(int priority)
    {
        m_log_priority = std::max(std::min(priority, 3), 0);
    }

    unsigned get_thread_number() const
    {
        return m_thread_number;
    }

    void set_thread_number(unsigned thread_number)
    {
        m_thread_number = thread_number;
    }

    void set_user_data(const boost::any &user_data)
    {
        m_user_data = user_data;
    }

    bool is_on_tty() const
    {
        return m_is_a_tty;
    }

    const boost::any& get_user_data() const { return m_user_data; }
    boost::any& get_user_data() { return m_user_data; }

private:
    log_fn       m_log_fn;
    boost::any   m_user_data;
    unsigned int m_thread_number = 0;
    int          m_log_priority  = 1;
    bool         m_is_a_tty;
};

inline void log_to_stderr(const ContextImpl& ctx, int priority,
                          const char *file, int line, const char *fn,
                          const char *format, va_list args)
{
    (void)ctx;
    (void)priority;
    (void)file;
    (void)line;

    if (ctx.is_on_tty()) {
        ::fprintf(stderr, ECHLL_YELLOW "echll:" ECHLL_RED " %s\n\t"
                  ECHLL_NORMAL, fn);
    } else {
        ::fprintf(stderr, "echll: %s\n\t", fn);
    }

    ::vfprintf(stderr, format, args);
    ::fputc('\n', stderr);
}

}

#endif
