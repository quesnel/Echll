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

#ifndef __VLE_KERNEL_ENVIRONMENT_HPP__
#define __VLE_KERNEL_ENVIRONMENT_HPP__

#include <vle/export.hpp>
#include <string>
#include <memory>
#include <stdexcept>

namespace vle {

class Environment;

typedef std::shared_ptr <Environment> EnvironmentPtr;

enum PackageDirectoryType
{
    PACKAGE_DIRECTORY,
    PACKAGE_DATA_DIRECTORY,
    PACKAGE_EXP_DIRECTORY,
    PACKAGE_SIMULATOR_DIRECTORY
};

struct environment_init_error : std::logic_error
{
    explicit environment_init_error(const std::string& msg)
        : std::logic_error(msg)
    {}
};

class VLE_API Environment
{
public:
    Environment();

    Environment(const Environment &rhs) = delete;

    Environment& operator=(const Environment &rhs) = delete;

    ~Environment();

    void get_version(int *major, int *minor, int *patch) const;

    void print(const std::string &msg);

    void warning(const std::string &msg);

    void error(const std::string &msg);

    void set_verbose_level(int level);

    int get_verbose_level() const;

    std::string get_prefix_path() const;

    void set_prefix_path(const std::string &path) const;

    std::string get_current_package() const;

    bool set_current_package(const std::string &package);

    std::string get_current_package_path() const;

    std::string get_package_path(const std::string &package,
                                 PackageDirectoryType type) const;

    std::string get_package_path(PackageDirectoryType type) const;

private:
    /// @cond SKIP
    class Pimpl;
    std::unique_ptr <Pimpl> m;
    /// @endcond
};

}

#endif
