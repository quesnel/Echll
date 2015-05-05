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

#ifndef ORG_VLEPROJECT_KERNEL_CONTEXT_IMPLEMENTATION_HPP
#define ORG_VLEPROJECT_KERNEL_CONTEXT_IMPLEMENTATION_HPP

#define ECHLL_YELLOW "\x1b[33m"
#define ECHLL_RED "\x1b[31m"
#define ECHLL_NORMAL "\x1b[0m"

#include <iostream>
#include <fstream>
#include <boost/format.hpp>

namespace vle {

bool is_a_tty();

bool is_a_tty()
{
#ifdef __unix__
    return ::fileno(stderr);
#else
    return false;
#endif
}

template <typename T>
struct no_stream_deleter {
    void operator()(T *)
    {}
};

ContextImpl::ContextImpl()
    : m_log(&std::clog, no_stream_deleter <std::ostream>())
    , m_thread_number(0)
    , m_log_priority(1)
    , m_is_a_tty(is_a_tty())
{}

ContextImpl::ContextImpl(const std::string &filename)
    : m_log(&std::clog, no_stream_deleter <std::ostream>())
    , m_thread_number(0)
    , m_log_priority(1)
    , m_is_a_tty(is_a_tty())
{
    std::shared_ptr<std::ofstream> ofs(new std::ofstream(filename));

    if (ofs->is_open())
        m_log = ofs;
    else
        std::cerr << boost::format("fail to open log file %1%\n") % filename;
}

std::ostream &ContextImpl::log() const
{
    return *m_log.get();
}

std::ostream &ContextImpl::dbg() const
{
    return std::cerr << ECHLL_YELLOW << "Debug: " << ECHLL_NORMAL;
}

int ContextImpl::get_log_priority() const
{
    return m_log_priority;
}

void ContextImpl::set_log_priority(int priority)
{
    m_log_priority = std::max(std::min(priority, 3), 0);
}

unsigned ContextImpl::get_thread_number() const
{
    return m_thread_number;
}

void ContextImpl::set_thread_number(unsigned thread_number)
{
    m_thread_number = thread_number;
}

void ContextImpl::set_user_data(const boost::any &user_data)
{
    m_user_data = user_data;
}

bool ContextImpl::is_on_tty() const
{
    return m_is_a_tty;
}

const boost::any &ContextImpl::get_user_data() const
{
    return m_user_data;
}

boost::any &ContextImpl::get_user_data()
{
    return m_user_data;
}

}

#undef ECHLL_YELLOW
#undef ECHLL_RED
#undef ECHLL_NORMAL

#endif
