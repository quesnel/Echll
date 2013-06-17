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

#ifndef __VLE_KERNEL_BASE_HPP__
#define __VLE_KERNEL_BASE_HPP__

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#ifndef VLE_NDEBUG
#include <boost/format.hpp>
#include <cassert>
#include <iostream>
namespace vle {

    using boost::format;

}
#endif

namespace vle {

    template <typename Type, typename Infinity >
        struct Time
        {
            static constexpr Type negative_infinity = Infinity::negative;
            static constexpr Type infinity = Infinity::positive;
            typedef Type type;
        };

    template <typename Type, typename Null >
        struct Value
        {
            bool is_null(const Type &t) const
            {
                return Null::is_null(t);
            }

            typedef Type type;
        };

    template <typename Value >
        struct PortList
        {
            typedef std::vector <typename Value::type> Values;

            std::vector <Values> ports;
            std::map <std::string, int> accessor;
            bool empty;

            PortList()
                : empty(true)
            {
            }

            PortList(std::initializer_list < std::string > lst)
                : empty(true)
            {
                for (auto & str : lst)
                    add(str);
            }

            int add(const std::string &name)
            {
                accessor[name] = ports.size();
                ports.push_back(std::vector <typename Value::type>());
                return ports.size() - 1;
            }

            const Values& operator[](int i) const
            {
                return ports[i];
            }

            const Values& operator[](const std::string &name) const
            {
                return ports[accessor.find(name)->second];
            }

            Values& operator[](int i)
            {
                empty = false;
                return ports[i];
            }

            Values& operator[](const std::string &name)
            {
                empty = false;
                return ports[accessor.find(name)->second];
            }

            void clear()
            {
                empty = true;

                for (auto & values : ports)
                    values.clear();
            }

            bool is_empty() const
            {
                return empty;
            }
        };

    template <typename Time, typename Value, typename Child >
        struct Synchronizer
        {
            Child child;

            typename Time::type run(typename Time::type begin,
                                    typename Time::type end)
            {
                child.start(begin);

                while (child.tn < end) {
                    child.output(child.tn);
                    child.transition(child.tn);
                }

                return child.tn;
            }
        };

    template <typename Time, typename Value, typename Child >
        struct SynchronizerLoop
        {
            Child child;
            typename Time::type begin;

            SynchronizerLoop(typename Time::type begin)
                : begin(begin)
            {
                child.start(begin);
            }

            bool loop(typename Time::type end)
            {
                if (child.tn < end) {
                    child.output(child.tn);
                    child.transition(child.tn);
                }

                return child.tn < end;
            }
        };
}

#endif
