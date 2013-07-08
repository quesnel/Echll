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

#ifndef __VLE_KERNEL_ENVIRONMENT_HPP__
#define __VLE_KERNEL_ENVIRONMENT_HPP__

#include <vle/export.hpp>
#include <vector>
#include <string>
#include <memory>
#include <cstdarg>

namespace vle {

enum EnvironmentLogOptions
{
    ENVIRONMENT_LOG_FILE,
    ENVIRONMENT_LOG_STDOUT
};

class VLE_API Environment
{
public:
    Environment(EnvironmentLogOptions opt);

    ~Environment();

    void get_version(int *major, int *minor, int *patch) const;

    void print(const char *format, ...);

    void print(const char *format, va_list ap);

    void warning(const char *format, ...);

    void warning(const char *format, va_list ap);

    void set_verbose_level(int level);

    int get_verbose_level() const;

    std::string get_prefix_path() const;

    std::string get_current_package() const;

    bool set_current_package(const std::string &package);

    std::string get_current_package_path() const;

private:
    Environment(const Environment &rhs);
    Environment& operator=(const Environment &rhs);

    class Pimpl;
    std::unique_ptr < Pimpl > m;
};

typedef std::weak_ptr < Environment > EnvironmentWeakPtr;
typedef std::shared_ptr < Environment > EnvironmentPtr;

}

#endif
