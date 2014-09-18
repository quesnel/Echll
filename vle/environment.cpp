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
#include <fstream>
#include <locale>
#include <thread>
#include <mutex>
#include <chrono>
#include <vector>
#include "i18n.hpp"
#include "path.hpp"
#include <ctime>

namespace vle {

struct HomeSystem
{
    std::string homepath;
    std::string pkgpath;
    std::shared_ptr <std::ostream> logfile;

    HomeSystem(const std::string& dirpath)
        : homepath(dirpath),
        pkgpath(vle::Path::make_path(dirpath, "pkgs-2.0"))
    {
        if (!vle::Path::exist_directory(dirpath) &&
            !vle::Path::create_directories(dirpath))
            throw environment_init_error(_("Fail to build home directory"));

        if (!vle::Path::exist_directory(pkgpath) &&
            !vle::Path::create_directories(pkgpath))
            throw environment_init_error(_("Fail to build pkgs directory"));

        try {
            std::string filename(vle::Path::make_path(homepath, "vle.log"));

            logfile = std::make_shared<std::ofstream>(filename.c_str());

            if (!(*logfile))
                throw std::exception();
            logfile->imbue(std::locale::classic());
        } catch (const std::exception& /*e*/) {
            throw environment_init_error(_("Failed to open log file"));
        }
    }

    void swap(HomeSystem& homesystem)
    {
        std::swap(homepath, homesystem.homepath);
        std::swap(pkgpath, homesystem.pkgpath);
        std::swap(logfile, homesystem.logfile);
    }
};

class Environment::Pimpl
{
public:
    Pimpl() :
        m_homesystem(vle::Path::make_path(Path::get_home_path(), "vle")),
        verbose_level(0),
        directories({"", "data", "exp", "simulators"})
    {}

    void set_current_home(const std::string& home_path)
    {
        try {
            HomeSystem hs(home_path);
            m_homesystem.swap(hs);
        } catch (const std::exception& e) {
            debugf("Failed to switch home directory to %s",
                   home_path.c_str());
        }
    }

    ~Pimpl()
    {
    }

    HomeSystem m_homesystem;

    std::string m_currentpackage;
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
    debugf(_("Environment is created at time: %s"), std::ctime(&end_time));
#endif
}

Environment::~Environment()
{
#ifndef VLE_NDEBUG_MODE
    m->end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = m->end - m->start;
    debugf(_("Environment is deleted. Elapsed time: %f s."),
           elapsed_seconds.count());
#endif
}

void Environment::print(const std::string& msg)
{
    if (m->m_homesystem.logfile.get())
        (*m->m_homesystem.logfile) << msg << std::endl;
}

void Environment::warning(const std::string& msg)
{
    if (m->m_homesystem.logfile.get())
        (*m->m_homesystem.logfile) << _("[Warning] ") << msg << std::endl;
}

void Environment::error(const std::string& msg)
{
    if (m->m_homesystem.logfile.get())
        (*m->m_homesystem.logfile) << _("[Error] ") << msg << std::endl;
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
#ifdef HAVE_CONFIG_H
    if (*major)
        *major = VERS_MAJ;

    if (*minor)
        *minor= VERS_MIC;

    if (*patch)
        *patch = VERS_MIN;
#else
    (void)major;
    (void)minor;
    (void)patch;
#endif
}

std::string Environment::get_prefix_path() const
{
    return m->m_homesystem.homepath;
}

void Environment::set_prefix_path(const std::string &path) const
{
    m->set_current_home(path);
}

bool Environment::set_current_package(const std::string &package)
{
    if (package.empty())
        return false;

    m->m_currentpackage = package;

    return true;
}

std::string Environment::get_current_package() const
{
    return m->m_currentpackage;
}

std::string Environment::get_current_package_path() const
{
    return vle::Path::make_path(m->m_homesystem.pkgpath,
                                m->m_currentpackage);
}

std::string Environment::get_package_path(const std::string &package,
                                          PackageDirectoryType type) const
{
    return vle::Path::make_path(m->m_homesystem.pkgpath,
                                package,
                                m->directories[static_cast <int>(type)]);
}

std::string Environment::get_package_path(PackageDirectoryType type) const
{
    return vle::Path::make_path(m->m_homesystem.pkgpath,
                                m->m_currentpackage,
                                m->directories[static_cast <int>(type)]);
}

}
