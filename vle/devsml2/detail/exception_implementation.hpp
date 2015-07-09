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

#ifndef FR_INRA_VLE_DEVSML_DETAIL_EXCEPTION_IMPLEMENTATION_HPP
#define FR_INRA_VLE_DEVSML_DETAIL_EXCEPTION_IMPLEMENTATION_HPP

namespace vle { namespace devsml2 {

inline
devsml_error::devsml_error(const std::string& what)
    : std::runtime_error(what)
{
}

inline
parser_error::parser_error(const std::string& filename,
                           const std::string& message)
    : devsml_error(format_what(filename, message))
    , m_filename(filename)
    , m_message(message)
{
}

inline
std::string parser_error::filename() const
{
    return m_filename;
}

inline
std::string parser_error::message() const
{
    return m_message;
}

inline
std::string parser_error::format_what(const std::string& filename,
                                      const std::string& message)
{
    if (filename.empty())
        return std::string("<unspecified file>");
    else {
        std::string msg;

        msg.reserve(filename.size() + 2 + message.size());
        msg = filename + ": " + message;

        return msg;
    }
}

inline
semantic_error::semantic_error(const std::string& type,
                               const std::string& variable,
                               const std::string& msg)
    : devsml_error(format_what(type, variable, msg))
{
}

inline
std::string semantic_error::type() const
{
    return m_type;
}

inline
std::string semantic_error::variable() const
{
    return m_variable;
}

inline
std::string semantic_error::msg() const
{
    return m_message;
}

inline
std::string semantic_error::format_what(const std::string& type,
                                        const std::string& variable,
                                        const std::string& msg)
{
    std::string result;
    result.reserve(8 + type.size() + 1 + variable.size() + 2 + msg.size() + 1);

    result = "fail in " + type +  '(' + variable + "):" + msg;

    return result;
}

}}

#endif
