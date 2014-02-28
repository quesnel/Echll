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

#include <vle/environment.hpp>
#include <vle/dbg.hpp>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <chrono>
#include <vector>
#include "i18n.hpp"
#include "path.hpp"
#include "config.h"
#include <ctime>

namespace vle {

class Environment::Pimpl
{
public:
    Pimpl()
        : verbose_level(0), directories({"", "data", "exp", "simulators"})
    {
        std::string home = vle::Path::make_path(Path::get_home_path(),
                                                "vle");
        set_current_home(home);
    }

    void set_current_home(const std::string& home_path)
    {
        if (!vle::Path::exist_directory(home_path)) {
            if (!vle::Path::create_directories(home_path)) {
                dWarning("Failed to build home path ", home_path);
                throw std::runtime_error("make home path");
            }
        }

        std::string pkgs = vle::Path::make_path(home_path, "pkgs-2.0");
        if (!vle::Path::exist_directory(pkgs)) {
            if (!vle::Path::create_directories(pkgs)) {
                dWarning("Failed to build package path ", pkgs);
                throw std::runtime_error("make package path");
            }
        }

        pkgs_path = pkgs;
        prefix_path = prefix_path;
    }

    int init(const std::string &filepath)
    {

        int ret = prefix_init(prefix_path);
        if (ret)
            return ret;

        logfilepath = filepath;
        if (logfilepath.empty())
            logfilepath = vle::Path::make_path(prefix_path, "vle-2.0.log");

        logfile.open(logfilepath);
        if (!logfile.is_open())
            return -4;

        return 0;
    }

    int prefix_init(const std::string &prefix)
    {
        std::string pkgs = vle::Path::make_path(prefix, "pkgs-2.0");

        if (!vle::Path::exist_directory(prefix))
            if (!vle::Path::create_directories(prefix))
                return -2;

        if (!vle::Path::exist_directory(pkgs))
            if (!vle::Path::create_directories(pkgs))
                return -3;

        pkgs_path = pkgs;
        prefix_path = prefix;

        return 0;
    }

    ~Pimpl()
    {
    }

    typedef std::recursive_mutex lock_type;
    typedef std::unique_lock <lock_type> scoped_lock_type;

    lock_type mutex;
    std::ofstream logfile;
    std::string logfilepath;
    std::string prefix_path;
    std::string pkgs_path;
    std::string current_package;
    int verbose_level;
    std::vector <std::string> directories;
#ifndef VLE_NDEBUG_MODE
    std::chrono::time_point <std::chrono::system_clock> start;
    std::chrono::time_point <std::chrono::system_clock> end;
#endif
};

Environment::Environment()
    : m(new Environment::Pimpl())
{
#ifndef VLE_NDEBUG_MODE
    m->start = std::chrono::system_clock::now();
    std::time_t end_time = std::chrono::system_clock::to_time_t(m->start);
    dInfo(_("Environment is created at time: "), std::ctime(&end_time));
#endif
}

Environment::~Environment()
{
#ifndef VLE_NDEBUG_MODE
    m->end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = m->end - m->start;
    dInfo(_("Environment is deleted. Elapsed time: "), elapsed_seconds.count(), "s");
#endif
}

int Environment::init()
{
    std::string logfilepath = "vle.log";

    return m->init(logfilepath);
}

void Environment::print(const std::string& msg)
{
    m->logfile << msg << std::endl;
}

void Environment::warning(const std::string& msg)
{
    m->logfile << _("[Warning] ") << msg << std::endl;
}

void Environment::error(const std::string& msg)
{
    m->logfile << _("[Error] ") << msg << std::endl;
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

bool Environment::set_prefix_path(const std::string &path) const
{
    int ret = m->prefix_init(path);
    if (ret)
        return false;

    m->prefix_path = path;

    return true;
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
    return vle::Path::make_path(m->pkgs_path, m->current_package);
}

std::string Environment::get_package_path(const std::string &package,
                                          PackageDirectoryType type) const
{
    return vle::Path::make_path(m->pkgs_path,
                                package,
                                m->directories[static_cast <int>(type)]);
}

std::string Environment::get_package_path(PackageDirectoryType type) const
{
    return vle::Path::make_path(m->pkgs_path,
                                m->current_package,
                                m->directories[static_cast <int>(type)]);
}

}
