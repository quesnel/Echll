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

#include <vle/environment.hpp>
#include <vle/path.hpp>
#include "config.h"
#include <cstdio>
#include <cstdarg>

namespace vle {

struct Environment::Pimpl
{
    Pimpl()
        : logstream(stderr), verbose_level(0)
    {
    }

    ~Pimpl()
    {
        fclose(logstream);
    }

    FILE *logstream;
    std::string prefix_path;
    std::string pkgs_path;
    std::string current_package;
    int verbose_level;
};

Environment::Environment(EnvironmentLogOptions opt)
    : m(new Environment::Pimpl())
{
    char *home = NULL;

    if ((home = ::getenv("VLE_HOME")) == NULL)
        if ((home = ::getenv("HOME")) == NULL)
            if ((home = ::getenv("PWD")) == NULL)
                home = ::getenv("TMP");

    if (!home) {
        warning("Failed to read VLE_HOME, HOME and TMP environment varaible\n");
        exit(-1);
    }

    std::vector < std::string > lst = { home, ".vle" };
    m->prefix_path = vle::Path::make_path(lst.begin(), lst.end());

    lst.push_back("pkgs-2.0");
    m->pkgs_path = vle::Path::make_path(lst.begin(), lst.end());

    if (!vle::Path::exist_directory(m->prefix_path)) {
        if (!vle::Path::create_directories(m->prefix_path)) {
            warning("Failed to initialize VLE_HOME directory `%s'\n",
                    m->prefix_path.c_str());
            exit(-1);
        }
    }

    if (!vle::Path::exist_directory(m->pkgs_path)) {
        if (!vle::Path::create_directories(m->pkgs_path)) {
            warning("Failed to initialize packages directory `%s'\n",
                    m->pkgs_path.c_str());
            exit(-1);
        }
    }

    if (opt == ENVIRONMENT_LOG_FILE) {
        std::vector < std::string > lst = { m->prefix_path, "vle.log" };
        std::string path = vle::Path::make_path(lst.begin(), lst.end());

        FILE *stream = fopen(path.c_str(), "w");
        if (stream)
            m->logstream = stream;
        else
            warning("Failed to initialize log file `%s'\n",
                    path.c_str());
    }
}

Environment::~Environment()
{
}

void Environment::print(const char *format, ...)
{
    va_list ap;

    va_start(ap, format);
    print(format, ap);
    va_end(ap);
}

void Environment::print(const char *format, va_list ap)
{
    std::vfprintf(m->logstream, format, ap);
}

void Environment::warning(const char *format, ...)
{
    va_list ap;

    va_start(ap, format);
    warning(format, ap);
    va_end(ap);
}

void Environment::warning(const char *format, va_list ap)
{
    if (m->logstream != stderr || m->logstream != stdout)
        std::fprintf(m->logstream, "\033[31mWarning: \033[m");
    else
        std::fprintf(m->logstream, "Warning: ");

    std::vfprintf(m->logstream, format, ap);
}

void Environment::set_verbose_level(int level)
{
    m->verbose_level = std::min(3, std::max(0, level));
}

int Environment::get_verbose_level() const
{
    return m->verbose_level;
}

void Environment::get_version(int *major, int *minor, int *patch) const
{
    if (*major)
        *major = VERS_MAJ;

    if (*minor)
        *minor= VERS_MIC;

    if (*patch)
        *patch = VERS_MIN;
}

std::string Environment::get_prefix_path() const
{
    return m->prefix_path;
}

bool Environment::set_current_package(const std::string &package)
{
    if (package.empty())
        return false;

    m->current_package = package;

    return true;
}

std::string Environment::get_current_package() const
{
    return m->current_package;
}

std::string Environment::get_current_package_path() const
{
    std::vector < std::string > lst = { m->pkgs_path, m->current_package };

    return vle::Path::make_path(lst.begin(), lst.end());
}

}
