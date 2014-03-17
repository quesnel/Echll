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

#ifdef HAVE_CONFIG_H
# include <config.h>
#endif

#include <algorithm>
#include <cstdio>
#include "path.hpp"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

namespace vle {

bool Path::exist_file(const std::string &filename)
{
    struct stat sb;

    if (::stat(filename.c_str(), &sb) == -1)
        return false;

    return (sb.st_mode & S_IFMT) == S_IFREG;
}

bool Path::exist_directory(const std::string &dirname)
{
    struct stat sb;

    if (::stat(dirname.c_str(), &sb) == -1)
        return false;

    return (sb.st_mode & S_IFMT) == S_IFDIR;
}

bool Path::create_directories(const std::string &dirname)
{
    typedef std::string::size_type Int;

    Int begin = 0;
    Int end;

    std::string t(dirname);
    do {
        end = t.find('/', begin);

        if (end != std::string::npos) {
            if (end)
                t[end] = '\0';
            begin = end + 1;
        }

        if (end == 0)
            continue;

        struct stat sb;
        if (::stat(t.c_str(), &sb)) {
            if (::mkdir(t.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
                return false;
        } else if (S_ISDIR (sb.st_mode) == 0)
            return false;

        if (end != std::string::npos)
            t[end] = '/';
    } while (end != std::string::npos);

    return true;
}

std::string Path::get_temporary_path()
{
    const char *names[] = { "TMPDIR", "TMP", "TEMP" };
    const int names_size = sizeof(names) / sizeof(names[0]);

    for (int i = 0; i != names_size; ++i)
        if (::getenv(names[i]) && exist_directory(::getenv(names[i])))
            return names[i];

    return "/tmp";
}

std::string Path::get_home_path()
{
    char *home = nullptr;

    if ((home = ::getenv("VLE_HOME")) == nullptr)
        if ((home = ::getenv("HOME")) == nullptr)
            if ((home = ::getenv("PWD")) == nullptr)
                if ((home = ::getenv("TMP")) == nullptr)
                    return std::string();

    return std::string(home);
}

}
