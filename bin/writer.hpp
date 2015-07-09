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

#ifndef FR_INRA_VLE_DEVSML_WRITER_HPP
#define FR_INRA_VLE_DEVSML_WRITER_HPP

#include <vle/devsml2/exception.hpp>
#include <string>
#include <ostream>

namespace vle { namespace devsml2 {

enum license_type { LICENSE_BSD2, LICENSE_GPL, WITHOUT_LICENSE };

struct license
{
    license();

    std::string authors;
    std::string copyright_owners;
    license_type type;
};

class writer_error : public devsml_error
{
public:
    writer_error(const std::string& filename, const std::string& msg);

    std::string filename() const;

    std::string message() const;

private:
    std::string m_filename;
    std::string m_message;

    static std::string format_what(const std::string& msg,
                                   const std::string& filename);
};

std::ostream& operator<<(std::ostream& os, const license& lic);

}}

#endif
