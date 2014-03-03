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

#ifdef ENABLE_NLS
# include <libintl.h>
#else
# define gettext(x)          (x)
# define dgettext(domain, x) (x)
#endif
#define _(x)                 dgettext(PACKAGE, x)

#include <vle/environment.hpp>
#include <iostream>

int main(int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    std::setlocale(LC_ALL, "");

#ifdef ENABLE_NLS
    ::bindtextdomain(PACKAGE, LOCALE_DIR);
    ::bind_textdomain_codeset(PACKAGE, "UTF-8");
#endif

    vle::EnvironmentPtr env;
    try {
        env = std::make_shared<vle::Environment>();
    } catch (const std::exception& e) {
        std::cerr << _("Failed to initialize VLE: "), e.what();
        return EXIT_FAILURE;
    }

#ifdef HAVE_CONFIG_H
    env->warning(_(PACKAGE_NAME " " VERSION " started"));
#else
    env->warning(_("VLE started"));
#endif

    return EXIT_SUCCESS;
}
