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

#ifndef FR_INRA_VLE_DEVSML_DEVSML_IMPLEMENTATION_HPP
#define FR_INRA_VLE_DEVSML_DEVSML_IMPLEMENTATION_HPP

namespace vle { namespace devsml2 {

template <typename T>
struct GetByNameCompare
{
    bool operator()(const T& t, const std::string& name) const
    {
        return t.name < name;
    }

    bool operator()(const std::string& name, const T& t) const
    {
        return name < t.name;
    }
};

template <typename T>
typename T::value_type *get_pointer_by_name(T& t, const std::string& name)
{
    if (not name.empty()) {
        typename T::iterator it = std::lower_bound(
            t.begin(), t.end(),
            name,
            GetByNameCompare <typename T::value_type>());

        if (it != t.end() and it->name == name)
            return &(*it);
    }

    return nullptr;
}

template <typename T>
bool sort_by_name(T& t)
{
    std::sort(t.begin(), t.end(),
              [](const typename T::value_type& a, const typename T::value_type& b)
              {
                  return a.name < b.name;
              });

    return std::adjacent_find(t.begin(), t.end(),
                              [](const typename T::value_type& a, const typename T::value_type& b)
                              {
                                  return a.name == b.name;
                              }) == t.end();
}

inline
std::vector <Message> convert_message(std::vector <parser::Message>& in,
                                      parser::MessageType type,
                                      std::vector <Entity>& entities)
{
    if (not sort_by_name(in))
        throw semantic_error("messages", std::string(), "too many message with same name");

    std::vector <Message> ret;

    for (std::size_t i = 0, e = in.size(); i != e; ++i) {
        if (in[i].type == type) {
            ret.emplace_back();
            ret.back().name = in[i].name;
            ret.back().ref = get_pointer_by_name(entities, in[i].ref);

            if (not ret.back().ref)
                throw semantic_error("message", in[i].name, "unknown entity in message");
        }
    }

    return ret;
}

inline
std::vector <Variable> convert_variables(const std::vector <parser::Variable> &in,
                                         std::vector <Entity>& entities)
{
    std::vector <Variable> ret;
    ret.resize(in.size());

    for (const auto& var : in) {
        ret.back().name = var.name;
        ret.back().entity = nullptr;

        if (var.type == "int")
            ret.back().type = VARIABLE_TYPE_INT;
        else if (var.type == "double")
            ret.back().type = VARIABLE_TYPE_DOUBLE;
        else if (var.type == "string")
            ret.back().type = VARIABLE_TYPE_STRING;
        else if (var.type == "boolean")
            ret.back().type = VARIABLE_TYPE_BOOLEAN;
        else {
            ret.back().entity = get_pointer_by_name(entities, var.type);
            if (not ret.back().entity)
                throw semantic_error("variable", var.name, "unknown entity in variable");
        }
    }

    return ret;
}

inline
std::vector <Entity> convert_entities(std::vector <parser::Entity>& in)
{
    if (not sort_by_name(in))
        throw semantic_error("entities", std::string(), "too many entities with same name");

    std::vector <Entity> ret(in.size());

    for (std::size_t i = 0, e = ret.size(); i != e; ++i)
        ret[i].name = in[i].name;

    for (std::size_t i = 0, e = ret.size(); i != e; ++i) {
        ret[i].supertype = nullptr;

        if (not in[i].supertype.empty()) {
            ret[i].supertype = get_pointer_by_name(ret, in[i].supertype);
            if (not ret[i].supertype)
                throw semantic_error("entity", ret[i].name, "unknown supertype");
        }

        ret[i].variables = convert_variables(in[i].variables, ret);
    }

    return ret;
}

inline
std::vector <STA> convert_stas(std::vector <parser::STA>& in, std::vector <Variable>& variables)
{
    if (not sort_by_name(in))
        throw semantic_error("stas", std::string(), "too many state-time-advance with same name");

    std::vector <STA> ret(in.size());

    for (std::size_t i = 0, e = in.size(); i != e; ++i) {
        ret[i].name = in[i].name;

        if (boost::get <double>(&in[i].value)) {
            ret[i].value = boost::get <double>(in[i].value);
        } else if (boost::get <std::string>(&in[i].value)) {
            Variable *variable = get_pointer_by_name(variables, boost::get <std::string>(in[i].value));
            if (not variable)
                throw semantic_error("state-time-advance", ret[i].name, "unknown variable type");
            ret[i].value = variable;
        } else
            throw semantic_error("state-time-advance", ret[i].name, "unknown type");
    }

    return ret;
}

inline
StateMachine convert_statemachine(parser::StateMachine& in, Atomic& atomic)
{
    StateMachine ret;

    ret.initState.state = get_pointer_by_name(atomic.stas, in.initState.state);
    if (not ret.initState.state)
        throw semantic_error("state-machine", atomic.name, "unknown init state");

    for (auto& fct : in.deltext) {
        ret.deltext.emplace_back();

        ret.deltext.back().state = get_pointer_by_name(atomic.stas, fct.state);
        if (not ret.deltext.back().state)
            throw semantic_error("state-machine", atomic.name, "unknown state in deltext");

        ret.deltext.back().target = get_pointer_by_name(atomic.stas, fct.target);
        if (not ret.deltext.back().target)
            throw semantic_error("state-machine", atomic.name, "unknown target in deltext");

        ret.deltext.back().setsig = fct.setsig;
        ret.deltext.back().code = fct.code;

        for (auto& msg : fct.msgs) {
            Message *message = get_pointer_by_name(atomic.ins, msg);
            if (not message)
                throw semantic_error("state-machine", atomic.name, "unknown message in deltext");

            ret.deltext.back().msgs.emplace_back(message);
        }
    }

    for (auto& fct : in.deltint) {
        ret.deltint.emplace_back();

        ret.deltint.back().state = get_pointer_by_name(atomic.stas, fct.state);
        if (not ret.deltint.back().state)
            throw semantic_error("state-machine", atomic.name, "unknown state in delint");

        ret.deltint.back().target = get_pointer_by_name(atomic.stas, fct.target);
        if (not ret.deltint.back().target)
            throw semantic_error("state-machine", atomic.name, "unknown target in deltext");

        ret.deltint.back().code = fct.code;
    }

    for (auto& fct : in.outfn) {
        ret.outfn.emplace_back();

        ret.outfn.back().state = get_pointer_by_name(atomic.stas, fct.state);
        if (not ret.outfn.back().state)
            throw semantic_error("state-machine", atomic.name, "unknown state in outfn");

        ret.outfn.back().code = fct.code;

        for (auto& msg : fct.msgs) {
            Message *message = get_pointer_by_name(atomic.outs, msg);
            if (not message)
                throw semantic_error("state-machine", atomic.name, "unknown message in outfn");

            ret.outfn.back().msgs.emplace_back(message);
        }
    }

   switch (in.confluent) {
   case parser::CONFLUENT_IGNORE_INPUT:
       ret.confluent = CONFLUENT_IGNORE_INPUT;
       break;
   case parser::CONFLUENT_INPUT_ONLY:
       ret.confluent = CONFLUENT_INPUT_ONLY;
       break;
   case parser::CONFLUENT_INPUT_FIRST:
       ret.confluent = CONFLUENT_INPUT_FIRST;
       break;
   case parser::CONFLUENT_INPUT_LATER:
       ret.confluent = CONFLUENT_INPUT_LATER;
       break;
   }

    return ret;
}

inline
std::vector <Atomic> convert_atomics(std::vector <parser::Atomic>& in,
                                     std::vector <Entity>& entities)
{
    if (not sort_by_name(in))
        throw semantic_error("atomics", std::string(), "too many atomic model with same name");

    std::vector <Atomic> ret(in.size());

    for (std::size_t i = 0, e = ret.size(); i != e; ++i)
        ret[i].name = in[i].name;

    for (std::size_t i = 0, e = ret.size(); i != e; ++i) {
        ret[i].type = COMPONENT_ATOMIC;
        ret[i].ins = convert_message(in[i].interfaceIO,
                                     parser::MESSAGE_TYPE_INPUT,
                                     entities);
        ret[i].outs = convert_message(in[i].interfaceIO,
                                      parser::MESSAGE_TYPE_OUTPUT,
                                      entities);
        if (not in[i].supertype.empty()) {
            ret[i].supertype = get_pointer_by_name(ret, in[i].supertype);

            if (not ret[i].supertype)
                throw semantic_error("atomics", ret[i].name, "unknonwn supertype");
        } else
            ret[i].supertype = nullptr;

        ret[i].variables = convert_variables(in[i].variables, entities);
        ret[i].stas = convert_stas(in[i].stas, ret[i].variables);
        ret[i].statemachine = convert_statemachine(in[i].statemachine, ret[i]);
    }

    return ret;
}

inline
std::vector <Component> convert_components(const Coupled &thismodel,
                                           std::vector <parser::Component>& in,
                                           std::vector <Atomic>& atomics,
                                           std::vector <Coupled>& coupleds)
{
    if (not sort_by_name(in))
        throw semantic_error("coupleds", std::string(), "too many components with same name");

    std::vector <Component> ret(in.size());

    for (std::size_t i = 0, e = in.size(); i != e; ++i) {
        if (in[i].name == "this")
            throw semantic_error("coupleds", in[i].name, "component can not have the name `this'");

        ret[i].name = in[i].name;

        Abstract *atomic = get_pointer_by_name(atomics, in[i].model);
        Abstract *coupled = get_pointer_by_name(coupleds, in[i].model);

        if (atomic and coupled)
            throw semantic_error("coupled", thismodel.name,
                                 "Two atomic and coupled components have same name");

        if (atomic)
            ret[i].model = atomic;
        else if (coupled)
            ret[i].model = coupled;
        else
            throw semantic_error("coupled", thismodel.name, "unknown component");
    }

    return ret;
}

inline
std::vector <Coupling> convert_couplings(Coupled *thismodel, std::vector <parser::Coupling>& in)
{
    std::vector <Coupling> ret(in.size());

    for (std::size_t i = 0, e = in.size(); i != e; ++i) {
        if (in[i].src == "this") {
            ret[i].src = thismodel;
            ret[i].msgtype = get_pointer_by_name(thismodel->ins, in[i].msgtype);
            if (not ret[i].msgtype)
                throw semantic_error("coupled", thismodel->name, "unknown message type in coupling");
        } else {
            Component *component = get_pointer_by_name(thismodel->components, in[i].src);
            if (not component)
                throw semantic_error("coupled", thismodel->name, "unknown source in coupling in coupling");

            ret[i].src = component->model;
            ret[i].msgtype = get_pointer_by_name(ret[i].src->outs, in[i].msgtype);
            if (not ret[i].msgtype)
                throw semantic_error("coupled", thismodel->name, "unknown message type in coupling");
        }

        if (in[i].dst == "this") {
            ret[i].dst = thismodel;
            ret[i].dstmsgtype = get_pointer_by_name(thismodel->outs, in[i].dstmsgtype);
            if (not ret[i].dstmsgtype)
                throw semantic_error("coupled", thismodel->name, "unknown message type in coupling");
        } else {
            Component *component = get_pointer_by_name(thismodel->components, in[i].dst);
            if (not component)
                throw semantic_error("coupled", thismodel->name, "unknown destination in coupling");

            ret[i].dst = component->model;
            ret[i].dstmsgtype = get_pointer_by_name(ret[i].dst->ins, in[i].dstmsgtype);
            if (not ret[i].dstmsgtype)
                throw semantic_error("coupled", thismodel->name, "unknown message type in coupling");
        }
    }

    return ret;
}

inline
std::vector <Coupled> convert_coupleds(std::vector <parser::Coupled>& in,
                                       std::vector <Atomic>& atomics,
                                       std::vector <Entity>& entities)
{
    if (not sort_by_name(in))
        throw semantic_error("coupleds", std::string(), "too many coupleds with same name");

    std::vector <Coupled> ret(in.size());

    for (std::size_t i = 0, e = ret.size(); i != e; ++i)
        ret[i].name = in[i].name;

    for (std::size_t i = 0, e = ret.size(); i != e; ++i) {
        ret[i].type = COMPONENT_COUPLED;
        ret[i].ins = convert_message(in[i].interfaceIO,
                                     parser::MESSAGE_TYPE_INPUT,
                                     entities);
        ret[i].outs = convert_message(in[i].interfaceIO,
                                      parser::MESSAGE_TYPE_OUTPUT,
                                      entities);

        if (not in[i].supertype.empty()) {
            Coupled *supertype = get_pointer_by_name(ret, in[i].supertype);
            if (not supertype)
                throw semantic_error("coupled", ret[i].name, "unknown supertype");

            ret[i].supertype = supertype;
        } else
            ret[i].supertype = nullptr;

        ret[i].components = convert_components(ret[i], in[i].components, atomics, ret);
    }

    for (std::size_t i = 0, e = ret.size(); i != e; ++i)
        ret[i].couplings = convert_couplings(&ret[i], in[i].couplings);

    return ret;
}

inline
void semantic_convert(vle::devsml2::parser::DevsML& p, DevsML& out)
{
    out.entities = convert_entities(p.entities);
    out.atomics = convert_atomics(p.atomics, out.entities);
    out.coupleds = convert_coupleds(p.coupleds, out.atomics, out.entities);

    if (not out.coupleds.empty()) {
        out.top = &out.coupleds.back();
    } else {
        out.top = nullptr;
    }
}

}}

#endif
