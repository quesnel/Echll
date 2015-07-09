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

#include "echll.hpp"
#include "writer.hpp"
#include <ostream>
#include <fstream>

namespace vle { namespace devsml2 { namespace echll {

void write_entities(std::ostream& os, const DevsML& data);

void write(const DevsML& data,
           const std::string& output_directory,
           const std::string& package_name,
           const license& lic = license());

void write_entities(std::ostream& os, const DevsML& data)
{
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
}

void write(const DevsML& data,
           const std::string& output_directory,
           const std::string& /*package_name*/,
           const license& lic)
{
    std::string filename(output_directory);
    filename += "/" + data.top->name + ".cpp";

    std::ofstream ofs(filename);
    if (not ofs)
        throw writer_error(filename, "failed to open");

    ofs << lic << '\n'
        << "#include <vle/echll/dsde.hpp>\n"
        << "#include <string>\n"
        << "#include <vector>\n";

    write_entities(ofs, data);

    //for (const auto& atomic : data.atomics)
        //write_atomic(ofs, data);

    //write_coupled(ofs, data);
}

}}}
