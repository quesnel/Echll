/*
 * Copyright (C) 2014 INRA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#ifndef __VLE2_UTILS_HPP__
#define __VLE2_UTILS_HPP__

#include <functional>

namespace vle
{
    /**
     * The @e ScopeExit structure permits the use of the @c Scope_Guard C++
     * idiom.
     *
     * @code
     * void world::add_person(person const& a_person)
     * {
     *     bool commit = false;
     *
     *     persons_.push_back(a_person);
     *     ScopeExit on_exit1([&commit, this](void)
     *     {
     *         if (!commit)
     *             persons_.pop_back();
     *     });
     *
     *     // ...
     *
     *     commit = true;
     * }
     * @endcode
     */
    struct ScopeExit
    {
        ScopeExit(std::function <void (void)> fct)
            : fct(fct)
        {
        }

        ~ScopeExit()
        {
            fct();
        }

    private:
        std::function<void (void)> fct;
    };
}

#endif
