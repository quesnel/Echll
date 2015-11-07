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

#include "writer.hpp"

namespace vle { namespace devsml2 {

license::license()
    : authors("DEVSML2 C++ convertor")
    , copyright_owners("2015 INRA")
    , type(LICENSE_BSD2)
{
}

writer_error::writer_error(const std::string& filename, const std::string& msg)
    : devsml_error(format_what(msg, filename))
    , m_filename(filename)
    , m_message(msg)
{
}

std::string writer_error::filename() const
{
    return m_filename;
}

std::string writer_error::message() const
{
    return m_message;
}

std::string writer_error::format_what(const std::string& msg,
                                      const std::string& filename)
{
    std::string result;
    result.reserve(filename.size() + msg.size() + 2);

    result = filename + ": " + msg;

    return result;
}

std::ostream& operator<<(std::ostream& os, const license& lic)
{
    os << "/*";

    if (not lic.copyright_owners.empty())
        os << " * " << lic.copyright_owners << '\n';

    if (not lic.authors.empty())
        os << " * " << lic.authors << '\n';

    os << " *\n";

    switch (lic.type) {
    case LICENSE_GPL:
        os << " * This program is free software; you can redistribute it and/or modify\n"
            " * it under the terms of the GNU General Public License as published by\n"
            " * the Free Software Foundation; either version 2 of the License, or\n"
            " * (at your option) any later version.\n"
            "\n"
            " * This program is distributed in the hope that it will be useful,\n"
            " * but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
            " * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
            " * GNU General Public License for more details.\n"
            "\n"
            " * You should have received a copy of the GNU General Public License along\n"
            " * with this program; if not, write to the Free Software Foundation, Inc.,\n"
            " * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.\n";
        break;
    case LICENSE_BSD2:
        os << " * Permission is hereby granted, free of charge, to any person obtaining a copy\n"
            " * of this software and associated documentation files (the \"Software\"), to deal\n"
            " * in the Software without restriction, including without limitation the rights\n"
            " * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell\n"
            " * copies of the Software, and to permit persons to whom the Software is\n"
            " * furnished to do so, subject to the following conditions:\n"
            " *\n"
            " * The above copyright notice and this permission notice shall be included in\n"
            " * all copies or substantial portions of the Software.\n"
            " *\n"
            " * THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR\n"
            " * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,\n"
            " * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE\n"
            " * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER\n"
            " * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,\n"
            " * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE\n"
            " * SOFTWARE.\n";
        break;
    case WITHOUT_LICENSE:
    default:
        break;
    }

    return os << " */";
}

}}
