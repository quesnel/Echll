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

#ifndef FR_INRA_VLE_DEVSML_VLE1X_HPP
#define FR_INRA_VLE_DEVSML_VLE1X_HPP

#include "vle1x.hpp"
#include "writer.hpp"
#include <ostream>

namespace vle { namespace devsml2 { namespace vle1x {

struct headers
{
    headers(int type_)
        : type(0)
    {
    }

    int type;
};

std::ostream& operator<<(std::ostream& os, const headers& heads)
{
    switch (heads.type) {
    case 1:
        os << "#include <vle/devs/Dynamics.hpp>\n"
           << "#include <vle/devs/DynamicsDbg.hpp>\n"
           << "#include <vle/value/Map.hpp>\n"
           << "#include <vle/value/Integer.hpp>\n"
           << "#include <vle/value/Double.hpp>\n"
           << "#include <vle/value/Bool.hpp>\n"
           << "#include <vle/value/String.hpp>\n";
    case 0:
        os << "#include <string>\n"
           << "#include <vector>\n";
    }

    return os;
}

void write_entities(std::ostream& os, const DevsML& data, const license& lic)
{
    headers hds(0);

    os << lic << '\n' << hds << '\n';

    os << "#ifndef devsml_convert_vle1x_entities_hpp\n"
       << "#define devsml_convert_vle1x_entities_hpp\n"
       << "\n"
       << "namespace devsml_convert {\n";

    for (const auto& entity : data.entities)
        os << "struct " << entity.name << ";\n";

    os << '\n';

    for (const auto& entity : data.entities) {
        os << "struct " << entity.name;
        if (entity.supertype)
            os << " : " << entity.supertype->name;
        os << " {\n";
        for (const auto& variable : entity.variables) {
            os << "    ";
            switch (variable.type) {
            case VARIABLE_TYPE_INT:
                os << "int";
                break;
            case VARIABLE_TYPE_DOUBLE:
                os << "double";
                break;
            case VARIABLE_TYPE_STRING:
                os << "std::string";
                break;
            case VARIABLE_TYPE_BOOLEAN:
                os << "bool";
                break;
            case VARIABLE_TYPE_ENTITY:
                os << variable.entity->name;
                break;
            }
            os << ' ' << variable.name << '\n';
        }
        os << "};\n";
    }

    os << "} // namespace devsml_convert\n\n"
       << "#endif\n";
}

void write(const DevsML& data,
           const std::string& output_directory,
           const std::string& package_name,
           const license& lic = license());
{
    std::string filename(output_directory);
    filename += "/src/entities.hpp";

    {
        std::ofstream ofs(filename);
        if (not ofs)
            throw writer_error(filename, "failed to open");

        write_entities(ofs, data, lic);
    }

    for (const auto& atomic : data.atomics) {
        filename = output_directory;
        filename += "/src/" + atomic.name + ".cpp";

        {
            std::ofstream ofs(filename);
            if (not ofs)
                throw writer_error(filename, "failed to open");

            write_atomic(ofs, data, lic);
        }
    }

    filename = output_directory:
        filename += "/exp/" + data.top->name + ".vpz";

    {
        std::ofstream ofs(filename);
        if (not ofs)
            throw write_error(filename, "failed to open");

        write_coupled(ofs, data, lic);
    }
}

}}}
