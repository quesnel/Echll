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

#ifndef FR_INRA_VLE_DEVSML_PARSER_IMPLEMENTATION_HPP
#define FR_INRA_VLE_DEVSML_PARSER_IMPLEMENTATION_HPP

#ifndef NDEBUG
#define BOOST_SPIRIT_DEBUG
#endif

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/fusion/include/adapt_struct.hpp>
#include <boost/spirit/include/support_istream_iterator.hpp>

BOOST_FUSION_ADAPT_STRUCT(
    vle::devsml2::parser::Variable,
    (std::string, type)
    (std::string, name)
    )

BOOST_FUSION_ADAPT_STRUCT(
    vle::devsml2::parser::Entity,
    (std::string, name)
    (std::string, supertype)
    (std::vector <vle::devsml2::parser::Variable>, variables)
    )

BOOST_FUSION_ADAPT_STRUCT(
    vle::devsml2::parser::Message,
    (vle::devsml2::parser::MessageType, type)
    (std::string, ref)
    (std::string, name)
    )

BOOST_FUSION_ADAPT_STRUCT(
    vle::devsml2::parser::STA,
    (std::string, name)
    (vle::devsml2::parser::STAValue, value)
    )

BOOST_FUSION_ADAPT_STRUCT(
    vle::devsml2::parser::Deltint,
    (std::string, state)
    (std::string, target)
    (std::string, code)
    )

BOOST_FUSION_ADAPT_STRUCT(
    vle::devsml2::parser::Deltext,
    (std::string, state)
    (std::vector <std::string>, msgs)
    (std::string, target)
    (std::string, setsig)
    (std::string, code)
    )

BOOST_FUSION_ADAPT_STRUCT(
    vle::devsml2::parser::Outfn,
    (std::string, state)
    (std::vector <std::string>, msgs)
    (std::string, code)
    )

BOOST_FUSION_ADAPT_STRUCT(
    vle::devsml2::parser::InitState,
    (std::string, state)
    (std::string, code)
    )

BOOST_FUSION_ADAPT_STRUCT(
    vle::devsml2::parser::StateMachine,
    (vle::devsml2::parser::InitState, initState)
    (std::vector <vle::devsml2::parser::Deltext>, deltext)
    (std::vector <vle::devsml2::parser::Outfn>, outfn)
    (std::vector <vle::devsml2::parser::Deltint>, deltint)
    (std::vector <vle::devsml2::parser::ConfluentType>, confluent)
    )

BOOST_FUSION_ADAPT_STRUCT(
    vle::devsml2::parser::Atomic,
    (std::string, name)
    (std::string, supertype)
    (std::vector <vle::devsml2::parser::Variable>, variables)
    (std::vector <vle::devsml2::parser::Message>, interfaceIO)
    (std::vector <vle::devsml2::parser::STA>, stas)
    (vle::devsml2::parser::StateMachine, statemachine)
    )

BOOST_FUSION_ADAPT_STRUCT(
    vle::devsml2::parser::Component,
    (vle::devsml2::parser::ComponentType, type)
    (std::string, model)
    (std::string, name)
    )

BOOST_FUSION_ADAPT_STRUCT(
    vle::devsml2::parser::Coupling,
    (vle::devsml2::parser::CouplingType, type)
    (std::string, src)
    (std::string, msgtype)
    (std::string, dst)
    (std::string, dstmsgtype)
    )

BOOST_FUSION_ADAPT_STRUCT(
    vle::devsml2::parser::Coupled,
    (std::string, name)
    (std::string, supertype)
    (std::vector <vle::devsml2::parser::Component>, components)
    (std::vector <vle::devsml2::parser::Message>, interfaceIO)
    (std::vector <vle::devsml2::parser::Coupling>, couplings)
    )

BOOST_FUSION_ADAPT_STRUCT(
    vle::devsml2::parser::DevsML,
    (std::vector <vle::devsml2::parser::Entity>, entities)
    (std::vector <vle::devsml2::parser::Atomic>, atomics)
    (std::vector <vle::devsml2::parser::Coupled>, coupleds)
    )

namespace vle { namespace devsml2 { namespace parser {

template <typename Iterator, typename Skipper>
struct Parser
    : boost::spirit::qi::grammar <Iterator, DevsML(), Skipper>
{
    Parser();

    boost::spirit::qi::rule <Iterator, DevsML(), Skipper> devsml_rule;
    boost::spirit::qi::rule <Iterator, Atomic(), Skipper> atomic_rule;
    boost::spirit::qi::rule <Iterator, Coupled(), Skipper> coupled_rule;
    boost::spirit::qi::rule <Iterator, Component(), Skipper> component_rule;
    boost::spirit::qi::rule <Iterator, Coupling(), Skipper> coupling_rule;
    boost::spirit::qi::rule <Iterator, Entity(), Skipper> entity_rule;
    boost::spirit::qi::rule <Iterator, STA(), Skipper> sta_rule;
    boost::spirit::qi::rule <Iterator, Message(), Skipper> message_rule;
    boost::spirit::qi::rule <Iterator, Variable(), Skipper> variable_rule;

    boost::spirit::qi::rule <Iterator, StateMachine(), Skipper> statemachine_rule;
    boost::spirit::qi::rule <Iterator, InitState(), Skipper> initstate_rule;
    boost::spirit::qi::rule <Iterator, Outfn(), Skipper> outfn_rule;
    boost::spirit::qi::rule <Iterator, Deltext(), Skipper> deltext_rule;
    boost::spirit::qi::rule <Iterator, Deltint(), Skipper> deltint_rule;

    boost::spirit::qi::rule <Iterator, std::string()> identifier_rule;
    boost::spirit::qi::rule <Iterator, std::string()> vartype_rule;
    boost::spirit::qi::rule <Iterator, std::string()> code_rule;

    boost::spirit::qi::symbols<char, MessageType> message_type_choice;
    boost::spirit::qi::symbols<char, ConfluentType> confluent_type_choice;
    boost::spirit::qi::symbols<char, ComponentType> component_type_choice;
    boost::spirit::qi::symbols<char, CouplingType> coupling_type_choice;
};

template <typename Iterator, typename Skipper>
Parser <Iterator, Skipper>::Parser()
    : Parser::base_type(devsml_rule)
{
    namespace phx = boost::phoenix;
    namespace qi = boost::spirit::qi;
    namespace ascii = boost::spirit::ascii;

    using namespace qi::labels;
    using qi::eol;
    using qi::alpha;
    using qi::alnum;
    using qi::graph;
    using qi::lit;
    using qi::lexeme;
    using qi::int_;
    using qi::double_;
    using qi::raw;
    using ascii::char_;

    message_type_choice.add(
        "input", vle::devsml2::parser::MESSAGE_TYPE_INPUT)
        ("output", vle::devsml2::parser::MESSAGE_TYPE_OUTPUT);
    confluent_type_choice.add(
        "ignore-input", vle::devsml2::parser::CONFLUENT_IGNORE_INPUT)
        ("input-only", vle::devsml2::parser::CONFLUENT_INPUT_ONLY)
        ("input-first", vle::devsml2::parser::CONFLUENT_INPUT_FIRST)
        ("input-later", vle::devsml2::parser::CONFLUENT_INPUT_LATER);
    component_type_choice.add(
        "atomic", vle::devsml2::parser::COMPONENT_ATOMIC)
        ("coupled", vle::devsml2::parser::COMPONENT_COUPLED);
    coupling_type_choice.add(
        "eic", vle::devsml2::parser::COUPLING_EIC)
        ("ic", vle::devsml2::parser::COUPLING_IC)
        ("eoc", vle::devsml2::parser::COUPLING_EOC);

    code_rule =
        raw[lexeme['"' >> +(char_ - '"') >> '"']];

    identifier_rule =
        raw[lexeme[((alpha | char_('_'))
                    >> *(alnum | char_('.') | char_('_')))]];

    vartype_rule =
        raw[lexeme[((alpha | char_('_'))
                    >> *(alnum | char_('.') | char_('_')))]];

    variable_rule =
        vartype_rule
        >> identifier_rule
        ;

    entity_rule =
        "entity"
        >> identifier_rule
        >> -("extends" >> identifier_rule)
        >> -('{' >> *variable_rule >> '}')
        ;

    message_rule =
        message_type_choice
        >> identifier_rule
        >> identifier_rule
        ;

    sta_rule =
        identifier_rule
        >> (double_ | identifier_rule)
        ;

    initstate_rule =
        "start"
        >> qi::lit("in")
        >> identifier_rule
        >> -(qi::lit("{") >> code_rule >> qi::lit("}"))
        ;

    deltext_rule =
        "deltext"
        >> qi::lit('(')
        >> qi::lit("S:")
        >> identifier_rule
        >> qi::lit(',')
        >> qi::lit("X:")
        >> qi::lit('[')
        >> +identifier_rule
        >> qi::lit(']')
        >> qi::lit(')')
        >> qi::lit("=>")
        >> qi::lit("S\":")
        >> identifier_rule
        >> -(qi::lit('(') >> identifier_rule >> qi::lit(')'))
        >> -(qi::lit("{") >> code_rule >> qi::lit("}"))
        ;

    outfn_rule =
        "outfn"
        >> qi::lit('(')
        >> qi::lit("S:")
        >> identifier_rule
        >> qi::lit(')')
        >> qi::lit("=>")
        >> qi::lit("Y:")
        >> qi::lit('[')
        >> +identifier_rule
        >> qi::lit(']')
        >> -(qi::lit("{") >> code_rule >> qi::lit("}"))
        ;

    deltint_rule =
        "deltint"
        >> qi::lit('(')
        >> qi::lit("S:")
        >> identifier_rule
        >> qi::lit(')')
        >> qi::lit("=>")
        >> qi::lit("S\":")
        >> identifier_rule
        >> -(qi::lit("{") >> code_rule >> qi::lit("}"))
        ;

    statemachine_rule =
        "state-machine"
        >> qi::lit('{')
        >> initstate_rule[phx::at_c<0>(_val) = _1]
        >> *( deltext_rule[phx::push_back(phx::at_c<1>(_val), _1)]
            | outfn_rule[phx::push_back(phx::at_c<2>(_val), _1)]
            | deltint_rule[phx::push_back(phx::at_c<3>(_val), _1)]
            | ("confluent" >> confluent_type_choice[phx::push_back(phx::at_c<4>(_val), _1)])
            )
        >> '}'
        ;

    atomic_rule =
        "atomic"
        >> identifier_rule
        >> -("extends" >> identifier_rule)
        >> '{'
        >> -("vars" >> qi::lit('{') >> *variable_rule >> '}')
        >> -("interfaceIO" >> qi::lit('{') >> *message_rule >> '}')
        >> -("state-time-machine" >> qi::lit('{') >> *sta_rule >> '}')
        >> statemachine_rule
        >> '}'
        ;

    component_rule =
        component_type_choice
        >> identifier_rule
        >> identifier_rule
        ;

    coupling_rule =
        coupling_type_choice
        >> identifier_rule
        >> ':'
        >> identifier_rule
        >> "->"
        >> identifier_rule
        >> ':'
        >> identifier_rule
        ;

    coupled_rule =
        "coupled"
        >> identifier_rule
        >> -("extends" >> identifier_rule)
        >> '{'
        >> "models" >> qi::lit('{') >> *component_rule >> '}'
        >> "interfaceIO" >> qi::lit('{') >> *message_rule >> '}'
        >> "couplings" >> qi::lit('{') >> *coupling_rule >> '}'
        >> '}'
        ;

    devsml_rule =
        *( entity_rule[phx::push_back(phx::at_c<0>(_val), _1)]
         | atomic_rule[phx::push_back(phx::at_c<1>(_val), _1)]
         | coupled_rule[phx::push_back(phx::at_c<2>(_val), _1)]
         );

#ifndef NDEBUG
    BOOST_SPIRIT_DEBUG_NODES((devsml_rule)(atomic_rule)
                             (coupled_rule)(component_rule)
                             (coupling_rule)
                             (entity_rule)(sta_rule)(message_rule)
                             (variable_rule)(statemachine_rule)
                             (initstate_rule)(outfn_rule)
                             (deltext_rule)(deltint_rule)
                             (identifier_rule)(vartype_rule)
                             (code_rule))
#endif
}

template <typename Iterator, typename Skipper>
bool parse(Iterator &first, Iterator end, const Skipper& skipper, vle::devsml2::parser::DevsML &main)
{
    Parser <Iterator, Skipper> p;
    return boost::spirit::qi::phrase_parse(first, end, p, skipper, main);
}

bool parse_filename(const std::string& filename, vle::devsml2::parser::DevsML &main)
{
    std::ifstream ifs(filename);

    if (!ifs)
        throw parser_error(filename, "fail to open file");

    ifs.unsetf(std::ios::skipws);

    boost::spirit::istream_iterator begin(ifs);
    boost::spirit::istream_iterator end;

    return parse(begin, end, boost::spirit::qi::space, main) && begin == end;
}

bool parse_memory(const std::string& buffer, vle::devsml2::parser::DevsML &main)
{
    std::string::const_iterator begin(buffer.begin());
    std::string::const_iterator end(buffer.end());

    return parse(begin, end, boost::spirit::qi::space, main) && begin == end;
}

}}}

#endif
