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

#ifndef FR_INRA_VLE_DEVSML_PARSER_HPP
#define FR_INRA_VLE_DEVSML_PARSER_HPP

#include <vle/devsml2/exception.hpp>
#include <boost/variant.hpp>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace vle { namespace devsml2 { namespace parser {

struct Variable {
    std::string type;
    std::string name;
};

enum MessageType {
    MESSAGE_TYPE_INPUT,
    MESSAGE_TYPE_OUTPUT
};

struct Message {
    MessageType type;
    std::string ref;
    std::string name;
};

struct Entity {
    std::string name;
    std::string supertype;
    std::vector <Variable> variables;
};

typedef boost::variant <std::string, double> STAValue;

struct STA {
    std::string name;
    STAValue value;
};

enum ConfluentType {
    CONFLUENT_IGNORE_INPUT,
    CONFLUENT_INPUT_ONLY,
    CONFLUENT_INPUT_FIRST,
    CONFLUENT_INPUT_LATER
};

struct Deltint {
    std::string state;
    std::string target;
    std::string code;
};

struct Deltext {
    std::string state;
    std::vector <std::string> msgs;
    std::string target;
    std::string setsig;
    std::string code;
};

struct Outfn {
    std::string state;
    std::vector <std::string> msgs;
    std::string code;
};

struct InitState {
    std::string state;
    std::string code;
};

struct StateMachine {
    InitState initState;
    std::vector <Deltext> deltext;
    std::vector <Outfn> outfn;
    std::vector <Deltint> deltint;
    std::vector <ConfluentType> confluent;
};

struct Atomic {
    std::string name;
    std::string supertype;
    std::vector <Variable> variables;
    std::vector <Message> interfaceIO;
    std::vector <STA> stas;
    StateMachine statemachine;
};

enum ComponentType {
    COMPONENT_ATOMIC,
    COMPONENT_COUPLED
};

enum CouplingType {
    COUPLING_EIC,
    COUPLING_IC,
    COUPLING_EOC
};

struct Component {
    ComponentType type;
    std::string model;
    std::string name;
};

struct Coupling {
    CouplingType type;
    std::string src;
    std::string msgtype;
    std::string dst;
    std::string dstmsgtype;
};

struct Coupled {
    std::string name;
    std::string supertype;
    std::vector <Component> components;
    std::vector <Message> interfaceIO;
    std::vector <Coupling> couplings;
};

struct DevsML {
    std::vector <Entity> entities;
    std::vector <Atomic> atomics;
    std::vector <Coupled> coupleds;
};

template <typename Iterator, typename Skipper>
bool parse(Iterator &first, Iterator end, const Skipper& skipper,
           vle::devsml2::parser::DevsML &main);

inline bool parse_filename(const std::string& filename, vle::devsml2::parser::DevsML &main);

inline bool parse_memory(const std::string& buffer, vle::devsml2::parser::DevsML &main);

}}}

#include <vle/devsml2/detail/parser_implementation.hpp>

#endif
