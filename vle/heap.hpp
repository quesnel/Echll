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

#ifndef __VLE_KERNEL_HEAP_HPP__
#define __VLE_KERNEL_HEAP_HPP__

#include <boost/heap/fibonacci_heap.hpp>
#include <vle/dbg.hpp>

namespace vle {

template <typename T>
    struct HeapElementCompare : std::binary_function <T, T, bool>
    {
        bool operator()(const T &lhs, const T &rhs) const
        {
            return lhs.tn > rhs.tn;
        }
    };

template <typename Time, typename Value>
    struct HeapElement
    {
        typedef typename boost::heap::fibonacci_heap <HeapElement <Time, Value>,
              boost::heap::compare <HeapElementCompare <HeapElement <Time,
              Value>>>>::handle_type handle_t;

        void *element;
        handle_t heapid;
        typename Time::type tn;

        HeapElement(void *elt, typename Time::type tn)
            : element(elt), tn(tn)
        {}
    };

template <typename Time, typename Value>
    using HeapType = boost::heap::fibonacci_heap <HeapElement <Time, Value>,
          boost::heap::compare <HeapElementCompare
              <HeapElement <Time, Value>>>>;

template <typename Time, typename Value>
std::ostream& operator<<(std::ostream& out,
                         const HeapElement <Time, Value>& elt)
{
    return out << elt.element << ", " << ", " << elt.tn;
}

template <typename Time, typename Value>
std::ostream& operator<<(std::ostream& out,
                         const HeapType <Time, Value>& heap)
{
    auto it = heap.ordered_begin();
    auto et = heap.ordered_end();
    unsigned long int i = 0;

    for (; it != et; ++it, ++i)
        out << i << ":" << *it << "\n";

    return out;
}

}

#endif
