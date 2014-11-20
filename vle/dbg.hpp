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

#ifndef __VLE_KERNEL_DBG_HPP__
#define __VLE_KERNEL_DBG_HPP__


//#if defined __GNUC__
//#define VLE_GCC_PRINTF(format__, args__) __attribute__ ((format (printf, format__, args__)))
//#endif

//namespace vle {

//VLE_API void debugf(const char* format, ...) VLE_GCC_PRINTF(1, 2);

//#ifdef NDEBUG
//inline void debugf(const char* format, ...)
//{
    //(void)format;
//}
//#else
//void debugf(const char* format, ...)
//{
    //std::vector <char> buffer(1024, '\0');
    //int sz;
    //va_list ap;

    //for (;;) {
        //va_start(ap, format);
        //sz = std::vsnprintf(buffer.data(), buffer.size(), format, ap);
        //va_end(ap);

        //if (sz < 0) {
            //return;
        //} else if (static_cast <std::size_t>(sz) < buffer.size()) {
            //std::clog << "DEBUG: " << buffer.data() << "\n";
            //return;
        //} else {
            //buffer.resize(sz + 1);
        //}
    //}
//}
//#endif

//}

#endif
