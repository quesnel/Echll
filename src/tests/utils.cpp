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
#include "path.hpp"

#define CATCH_CONFIG_MAIN
#include "catch.hpp"

TEST_CASE("try-make_path_api", "run")
{
    {
        std::string path = vle::Path::make_path("A", "B", "C");
        REQUIRE(path == "A/B/C");
    }

    {
        std::string path = vle::Path::make_path("struct");
        REQUIRE(path == "struct");
    }
}

TEST_CASE("try-environment-package-path", "run")
{
    vle::EnvironmentPtr env = std::make_shared<vle::Environment>();
    REQUIRE(env->init());

    std::string tmp_path = vle::Path::get_temporary_path();
    env->set_prefix_path(tmp_path);

    REQUIRE(env->get_prefix_path() == tmp_path);
    REQUIRE(env->get_package_path("test", vle::PACKAGE_DATA_DIRECTORY) ==
            vle::Path::make_path(tmp_path, "pkgs-2.0", "test", "data"));
    REQUIRE(env->get_package_path("test", vle::PACKAGE_EXP_DIRECTORY) ==
            vle::Path::make_path(tmp_path, "pkgs-2.0", "test", "exp"));
    REQUIRE(env->get_package_path("test", vle::PACKAGE_SIMULATOR_DIRECTORY) ==
            vle::Path::make_path(tmp_path, "pkgs-2.0", "test", "simulators"));
}
