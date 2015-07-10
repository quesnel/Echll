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

#include <vle/devsml2/parser.hpp>
#include "writer.hpp"
#include "echll.hpp"
#include "vle1x.hpp"
#include <iostream>
#include <cstring>

namespace {

bool parse(const char *filename, vle::devsml2::DevsML& data) noexcept;
void write(const std::string outputdirectory, const std::string package,
           const vle::devsml2::DevsML& data) noexcept;
void help() __attribute__ ((noreturn));

bool parse(const char *filename, vle::devsml2::DevsML& data) noexcept
{
    bool ret = false;

    try {
        vle::devsml2::parser::DevsML parser_data;
        vle::devsml2::parser::parse_filename(filename, parser_data);
        vle::devsml2::semantic_convert(parser_data, data);
        ret = true;
    } catch (const vle::devsml2::parser_error& e) {
        std::cerr << "Parsing error\n\t" << e.what() << '\n';
    } catch (const vle::devsml2::semantic_error& e) {
        std::cerr << "Semantic error\n\t" << e.what() << '\n';
    } catch (const std::exception& e) {
        std::cerr << "Unknown error\n\t" << e.what() << '\n';
    } catch (...) {
        std::cerr << "Strange error, please send file to maintainer\n";
    }

    return ret;
}

void write(const std::string outputdirectory,
           const std::string package,
           const vle::devsml2::DevsML& data) noexcept
{
    try {
        vle::devsml2::echll::write(data, outputdirectory, package);
    } catch (const vle::devsml2::writer_error& e) {
        std::cerr << "Writing error\n\t" << e.what() << '\n';
    } catch (const std::exception& e) {
        std::cerr << "Unknown error\n\t" << e.what() << '\n';
    } catch (...) {
        std::cerr << "Strange error, please send file to maintainer\n";
    }
}

void help()
{
    std::cout << "devml2 [options...] {filenames...]\n"
        << "Options are:\n"
        << "\t-h/--help               This message\n"
        << "\t-o/--output-directory f Change the output directory (default=$PWD)\n"
        << "\t-p/--package f          Change the package (default=examples)\n"
        << std::endl;

    std::exit(0);
}

} // anonymous namespace

int main(int argc, char **argv)
{
    std::vector <char *> filenames;
    std::string outputdirectory(".");
    std::string package;

    int i = 1;
    while (i < argc) {
        if (i + 1 < argc) {
            if (std::strcmp(argv[i], "-o") == 0 ||
                std::strcmp(argv[i], "--output-directory") == 0)
                outputdirectory = argv[++i];
            else if (std::strcmp(argv[i], "-p") == 0 ||
                     std::strcmp(argv[i], "--package") == 0)
                package = argv[++i];
            else if (std::strcmp(argv[i], "-h") == 0 ||
                     std::strcmp(argv[i], "--help") == 0)
                help();
            else
                filenames.push_back(argv[i]);
        } else
            filenames.push_back(argv[i]);

        ++i;
    }

    for (auto& filename : filenames) {
        vle::devsml2::DevsML data;

        if (parse(filename, data))
            write(outputdirectory, package, data);
    }

    return 0;
}
