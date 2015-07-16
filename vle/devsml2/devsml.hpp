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

#ifndef FR_INRA_VLE_DEVSML_DATA_HPP
#define FR_INRA_VLE_DEVSML_DATA_HPP

#include <vle/devsml2/parser.hpp>
#include <vector>
#include <string>

namespace vle { namespace devsml2 {

enum VariableType {
    VARIABLE_TYPE_INT,
    VARIABLE_TYPE_DOUBLE,
    VARIABLE_TYPE_STRING,
    VARIABLE_TYPE_BOOLEAN,
    VARIABLE_TYPE_ENTITY
};

enum ComponentType {
    COMPONENT_ATOMIC,
    COMPONENT_COUPLED
};

struct Entity;

struct Variable {
    std::string name;
    Entity *entity;
    VariableType type;
};

struct Entity {
    std::string name;
    Entity *supertype;
    std::vector <Variable> variables;
};

struct Message {
    std::string name;
    Entity *ref;
};

struct Abstract {
    std::vector <Message> ins;
    std::vector <Message> outs;
    ComponentType type;

protected:
    /**
     * @brief Destructor is protected to avoid the use of the delete operator
     * from a pointer of an Abstract object.
     */
    ~Abstract() {}
};

struct Component {
    Abstract *model;
    std::string name;
};

struct Coupling {
    Abstract *src, *dst;
    Message *msgtype, *dstmsgtype;
};

struct Coupled : Abstract {
    std::string name;
    Coupled *supertype;
    std::vector <Component> components;
    std::vector <Coupling> couplings;
};

typedef boost::variant <Variable *, double> STAValue;

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
    STA *state;
    STA *target;
    std::string code;
};

struct Deltext {
    STA *state;
    STA *target;
    std::vector <Message*> msgs;
    std::string setsig;
    std::string code;
};

struct Outfn {
    STA *state;
    std::vector <Message*> msgs;
    std::string code;
};

struct InitState {
    STA *state;
    std::string code;
};

struct StateMachine {
    InitState initState;
    std::vector <Deltext> deltext;
    std::vector <Outfn> outfn;
    std::vector <Deltint> deltint;
    std::vector <ConfluentType> confluent;
};

struct Atomic : Abstract {
    std::string name;
    Atomic *supertype;
    std::vector <Variable> variables;
    std::vector <STA> stas;
    StateMachine statemachine;
};

struct DevsML {
    std::vector <Entity> entities;
    std::vector <Coupled> coupleds;
    std::vector <Atomic> atomics;
    Coupled *top;
};

void semantic_convert(vle::devsml2::parser::DevsML& p, DevsML& out);

}}

#include <vle/devsml2/detail/devsml_implementation.hpp>

#endif
