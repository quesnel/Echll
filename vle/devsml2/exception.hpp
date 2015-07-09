/* Copyright (C) 2015 INRA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef FR_INRA_VLE_DEVSML_EXCEPTION_HPP
#define FR_INRA_VLE_DEVSML_EXCEPTION_HPP

#include <stdexcept>
#include <string>

namespace vle { namespace devsml2 {

/** Base class for all vle devsml error. Derives from @e
 * std::runtime_error. Call member function @e what() to get human
 * readable message associated with the error
 */
class devsml_error : public std::runtime_error
{
public:
    devsml_error(const std::string& what);
};

class parser_error : public devsml_error
{
public:
    parser_error(const std::string& filename,
                 const std::string& message);

    std::string filename() const;

    std::string message() const;

private:
    std::string m_filename;
    std::string m_message;

    static std::string format_what(const std::string& msg,
                                   const std::string& filename);
};

class semantic_error : public devsml_error
{
public:
    semantic_error(const std::string& type,
                   const std::string& variable,
                   const std::string& msg);

    std::string type() const;

    std::string variable() const;

    std::string msg() const;

private:
    std::string m_type;
    std::string m_variable;
    std::string m_message;

    static std::string format_what(const std::string& type,
                                   const std::string& variable,
                                   const std::string& msg);
};

}}

#include <vle/devsml2/detail/exception_implementation.hpp>

#endif
