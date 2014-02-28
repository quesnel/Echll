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

#ifndef __VLE_KERNEL_EXPORT_HPP__
#define __VLE_KERNEL_EXPORT_HPP__

#if defined _WIN32 || defined __CYGWIN__
  #define VLE_HELPER_DLL_IMPORT __declspec(dllimport)
  #define VLE_HELPER_DLL_EXPORT __declspec(dllexport)
  #define VLE_HELPER_DLL_LOCAL
#else
  #if __GNUC__ >= 4
    #define VLE_HELPER_DLL_IMPORT __attribute__ ((visibility ("default")))
    #define VLE_HELPER_DLL_EXPORT __attribute__ ((visibility ("default")))
    #define VLE_HELPER_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
  #else
    #define VLE_HELPER_DLL_IMPORT
    #define VLE_HELPER_DLL_EXPORT
    #define VLE_HELPER_DLL_LOCAL
  #endif
#endif

#ifdef VLE_DLL
  #ifdef vlelib_EXPORTS
    #define VLE_API VLE_HELPER_DLL_EXPORT
  #else
    #define VLE_API VLE_HELPER_DLL_IMPORT
  #endif
  #define VLE_LOCAL VLE_HELPER_DLL_LOCAL
#else
  #define VLE_API
  #define VLE_LOCAL
#endif

#endif
