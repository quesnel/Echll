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
 *gcc  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <vle/devsml2/parser.hpp>
#include <fstream>
#include <iterator>
#include <iostream>
#include <cassert>
#include <cstdlib>

namespace test {

void parse_entity()
{
    const std::string buffer("entity toto extends X { int counter\ndouble arrival }");
    vle::devsml2::parser::DevsML main;

    bool result = vle::devsml2::parser::parse_memory(buffer, main);
    assert(result);

    assert(main.entities.empty() == false);
    assert(main.entities[0].name == std::string("toto"));
    assert(main.entities[0].supertype == std::string("X"));
    assert(main.entities[0].variables.size() == 2u);
    assert(main.entities[0].variables[0].type == std::string("int"));
    assert(main.entities[0].variables[0].name == "counter");
    assert(main.entities[0].variables[1].type == std::string("double"));
    assert(main.entities[0].variables[1].name == "arrival");
}

void parse_entity_without_variable()
{
    const std::string buffer("entity toto extends X");
    vle::devsml2::parser::DevsML main;

    bool result = vle::devsml2::parser::parse_memory(buffer, main);
    assert(result);

    assert(main.entities.empty() == false);
    assert(main.entities[0].name == std::string("toto"));
    assert(main.entities[0].supertype == std::string("X"));
    assert(main.entities[0].variables.empty());
}

void parse_entity_without_supertype()
{
    const std::string buffer("entity toto { int counter\ndouble arrival }");
    vle::devsml2::parser::DevsML main;

    bool result = vle::devsml2::parser::parse_memory(buffer, main);
    assert(result);

    assert(main.entities.empty() == false);
    assert(main.entities[0].name == std::string("toto"));
    assert(main.entities[0].supertype.empty());
    assert(main.entities[0].variables.size() == 2u);
    assert(main.entities[0].variables[0].type == std::string("int"));
    assert(main.entities[0].variables[0].name == "counter");
    assert(main.entities[0].variables[1].type == std::string("double"));
    assert(main.entities[0].variables[1].name == "arrival");
}

void parse_atomic()
{
    const std::string buffer("atomic toto extends X {"
                             "vars {int counter double arrival}"
                             "interfaceIO {input tata tutu output toto titi} "
                             "state-time-machine {x 1.0 y 2.0 z infinity a toto} "
                             "state-machine { start in passive "
                             "{ \"counter = 0\narrival = 1.5\" }"
                             "deltext (S:passive, X:[st]) => S\":active "
                             "outfn (S:active) => Y:[ job ] { \"job.id = count;\" } "
                             "deltint (S:finishing) => S\":passive "
                             "confluent ignore-input "
                             "}"
                             "}");
    vle::devsml2::parser::DevsML main;

    bool result = vle::devsml2::parser::parse_memory(buffer, main);
    assert(result);

    assert(main.atomics.empty() == false);
    assert(main.atomics[0].name == std::string("toto"));
    assert(main.atomics[0].supertype == std::string("X"));
    assert(main.atomics[0].variables.size() == 2u);
    assert(main.atomics[0].variables[0].type == std::string("int"));
    assert(main.atomics[0].variables[0].name == "counter");
    assert(main.atomics[0].variables[1].type == std::string("double"));
    assert(main.atomics[0].variables[1].name == "arrival");

    assert(main.atomics[0].interfaceIO.size() == 2u);
    assert(main.atomics[0].interfaceIO[0].type == vle::devsml2::parser::MESSAGE_TYPE_INPUT);
    assert(main.atomics[0].interfaceIO[0].ref == "tata");
    assert(main.atomics[0].interfaceIO[0].name == "tutu");
    assert(main.atomics[0].interfaceIO[1].type == vle::devsml2::parser::MESSAGE_TYPE_OUTPUT);
    assert(main.atomics[0].interfaceIO[1].ref == "toto");
    assert(main.atomics[0].interfaceIO[1].name == "titi");

    assert(main.atomics[0].stas.size() == 4u);
    assert(main.atomics[0].stas[0].name == "x");
    assert(boost::get <double>(&main.atomics[0].stas[0].value));
    assert(boost::get <double>(main.atomics[0].stas[0].value) == 1.0);
    assert(main.atomics[0].stas[1].name == "y");
    assert(boost::get <double>(&main.atomics[0].stas[1].value));
    assert(boost::get <double>(main.atomics[0].stas[1].value) == 2.0);
    assert(main.atomics[0].stas[2].name == "z");
    assert(boost::get <double>(&main.atomics[0].stas[2].value));
    assert(boost::get <double>(main.atomics[0].stas[2].value) == +HUGE_VAL);
    assert(main.atomics[0].stas[3].name == "a");
    assert(boost::get <std::string>(&main.atomics[0].stas[3].value));
    assert(boost::get <std::string>(main.atomics[0].stas[3].value) == "toto");

    assert(main.atomics[0].statemachine.deltext.size() == 1u);
    assert(main.atomics[0].statemachine.outfn.size() == 1u);
    assert(main.atomics[0].statemachine.deltint.size() == 1u);
    assert(main.atomics[0].statemachine.confluent.size() == 1u);
    assert(main.atomics[0].statemachine.confluent[0]
           == vle::devsml2::parser::CONFLUENT_IGNORE_INPUT);

    assert(main.atomics[0].statemachine.initState.state == "passive");
}

void parse_coupled()
{
    const std::string buffer(
        "coupled EF {"
        " models {"
        "  atomic Genr g"
        "  atomic Transd t"
        " }"
        " interfaceIO {"
        "  input ent.Job solved "
        "  output ent.Reult rslt "
        " }"
        " couplings {"
        "  eic this : solved -> t : solved"
        "  eoc t : rest -> this : rslt"
        "  ic efpef : stop -> g : sp"
        " }"
        "}"
        );

    vle::devsml2::parser::DevsML main;

    bool result = vle::devsml2::parser::parse_memory(buffer, main);
    assert(result);

    assert(main.coupleds.empty() == false);
    assert(main.coupleds[0].name == std::string("EF"));
    assert(main.coupleds[0].supertype == std::string(""));

    assert(main.coupleds[0].components.size() == 2u);
    assert(main.coupleds[0].components[0].type == vle::devsml2::parser::COMPONENT_ATOMIC);
    assert(main.coupleds[0].components[0].model == "Genr");
    assert(main.coupleds[0].components[0].name == "g");
    assert(main.coupleds[0].components[1].type == vle::devsml2::parser::COMPONENT_ATOMIC);
    assert(main.coupleds[0].components[1].model == "Transd");
    assert(main.coupleds[0].components[1].name == "t");

    assert(main.coupleds[0].interfaceIO.size() == 2u);

    assert(main.coupleds[0].couplings.size() == 3u);
}

} // namespace test

int main()
{
    int ret = EXIT_SUCCESS;

    try {
        test::parse_entity();
        test::parse_entity_without_variable();
        test::parse_entity_without_supertype();
        test::parse_atomic();
        test::parse_coupled();
    } catch (const std::exception& e) {
        std::cerr << "Failure: " << e.what() << std::endl;
        ret = EXIT_FAILURE;
    }

    return ret;
}
