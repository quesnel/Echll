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
#include <cassert>

namespace vle { namespace devsml2 { namespace echll {

inline std::string abstract_name_get(const Abstract *model);

std::ostream& operator<<(std::ostream& os, const STAValue& value);

void write_variable(std::ostream& os, const Variable& variable);
void write_entities(std::ostream& os, const DevsML& data);
void write_internal(std::ostream& os, const Deltint& fn);
void write_internals(std::ostream& os, const Atomic& atomic);
void write_external(std::ostream& os, const Deltext& fn);
void write_externals(std::ostream& os, const Atomic& atomic);
void write_state_machine(std::ostream& os, const Atomic& atomic);

template <typename Ins, typename Outs>
    void write_model_port_type(std::ostream& os, const std::string& name,
                               const Ins& ins, const Outs& outs);

void write_atomic(std::ostream& os, const Atomic& atomic);
void write_coupled(std::ostream& os, const Coupled& coupled);
void write_main(std::ostream& os, const DevsML& data);

void write_variable(std::ostream& os, const Variable& variable)
{
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

    os << ' ' << variable.name << ";\n";
}

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

        for (const auto& var: entity.variables)
            write_variable(os, var);

        os << "};\n\n";
    }
}

struct code_ostream {
    code_ostream(const std::string& code_)
        : code(code_)
    {}

    const std::string& code;
};

std::ostream& operator<<(std::ostream& os, const code_ostream& code);

std::ostream& operator<<(std::ostream& os, const code_ostream& code)
{
    if (not code.code.empty())
        return os << "{\n" << code.code << "}\n";

    return os;
}

std::ostream& operator<<(std::ostream& os, const STAValue& value)
{
    const double *real = boost::get <const double>(&value);
    if (real != nullptr)
        return os << *real;

    const Variable* const* variable = boost::get <const Variable* const>(&value);
    if (variable != nullptr)
        return os << (*variable)->name;

    assert(false && "STAValue convertion failure");

    return os;
}

void write_internal(std::ostream& os, const Deltint& fn)
{
    os << "    case " << fn.state->name << ":\n"
       << "        m_current_state = " << fn.target->name << ";\n"
       << "        " << fn.code << "\n"
       << "        return " << fn.target->value << ";\n";

    os << ";\n";
}

void write_internals(std::ostream& os, const Atomic& atomic)
{
    os << "    time_type internal()\n"
       << "    {\n";

    if (not atomic.statemachine.deltint.empty()) {
        os << "    switch (m_current_state) {\n";

        for (const auto& fn : atomic.statemachine.deltint)
            write_internal(os, fn);

        if (atomic.statemachine.deltint.size() != atomic.stas.size())
            os << "    default:\n"
               << "        return FIXME;\n";

        os << "    }\n";
    }

    os << "    }\n";
}

void write_external(std::ostream& os, const Deltext& fn)
{
    os << "    case " << fn.state->name <<": {\n";

    if (not fn.msgs.empty()) {
        os << "       if (";

        for (auto i = 0ul, e = fn.msgs.size(); i != e; ++i) {
            os << "not x." << fn.msgs[i]->name << ".empty()";

            if ((i + 1) == e)
                os << ") {\n";
            else
                os << "and        \n";
        }
    }

    if (fn.target)
        os << "        m_current_state = " << fn.target->name << ";\n";

    if (not fn.code.empty())
        os << "        " << fn.code << "\n";

    else if (fn.setsig == "continue")
        os << "        return r;\n";
    else if (fn.setsig.find("resched") == 0)
        os << "        return e;\n";
    else
        os << "        return " << fn.target->value << ";\n";

    if (not fn.msgs.empty())
        os << "        }\n";

    os << "    }\n"
       << "    break;\n";
}

void write_externals(std::ostream& os, const Atomic& atomic)
{
    (void)atomic;

    os << "    time_type external(const time_type& e, const time_type& r, const time_type& t)\n"
       << "    {\n";

    if (not atomic.statemachine.deltext.empty()) {
        os << "    switch (m_current_state) {\n";

        for (const auto& fn: atomic.statemachine.deltext)
            write_external(os, fn);

        if (atomic.statemachine.deltext.size() != atomic.stas.size())
            os << "    default:\n"
               << "        return FIXME;\n";

        os << "    }\n";
    }

    os << "    }\n";
}

void write_state_machine(std::ostream& os, const Atomic& atomic)
{
    os << "    virtual time_type\n"
       << "    init(vle::Common /*common*/, const time_type& time) override\n"
       << "    {\n"
       << "        m_current_state = "
       << atomic.statemachine.initState.state->name << ";\n"
       << "        " << atomic.statemachine.initState.code << ";\n"
       << "        return 0.0;\n" // FIXME
       << "    }\n";

    os << "    virtual time_type\n"
       << "    delta(const time_type& e, const time_type& r, const time_type& t) override\n"
       << "    {\n"
       << "        if (x.empty() && r == 0.0) {\n";

    //
    // FIXME We can improve this code generation for both IGNORE_INPUT and
    // INPUT_ONLY which can be merged with internal or external condition part.
    //

    switch (atomic.statemachine.confluent) {
    case CONFLUENT_IGNORE_INPUT:
        os << "            internal();\n";
        break;
    case CONFLUENT_INPUT_ONLY:
        os << "            external(e, r, t);\n";
        break;
    case CONFLUENT_INPUT_FIRST:
        os << "            external(e, r, t);\n"
           << "            internal();\n";
        break;
    case CONFLUENT_INPUT_LATER:
        os << "            internal();\n"
           << "            external(e, r, t);\n";
        break;
    }

    os << "        } else if (x.empty()) {\n"
       << "            return internal();\n"
       << "        } else {\n"
       << "            return external(e, r, t);\n"
       << "        }\n"
       << "    }\n";

    os << "    virtual void\n"
       << "    lambda() override const\n"
       << "    {\n"
       << "        switch (m_current_state) {\n";

    for (const auto& fn : atomic.statemachine.outfn)
        os << "        case " << fn.state->name << ":\n"
           << "        {\n            " << fn.code << "\n        }\n"
           << "            break;\n";

    os << "        default:\n"
       << "             break;\n"
       << "    }\n";

    os << "    }\n";
}

template <typename Ins, typename Outs>
    void write_model_port_type(std::ostream& os, const std::string& name,
                               const Ins& ins, const Outs& outs)
{
    os << "struct " << name << "_input_port {\n";

    for (const auto& msg : ins)
        os << "    std::vector <" << msg.ref->name << "> " << msg.name << ";\n";

    os << "    void clear() noexcept\n"
       << "    {\n";
    for (const auto& msg : ins)
        os << "        " << msg.name << ".clear();\n";
    os << "    }\n";

    os << "    bool empty() const noexcept\n"
       << "    {\n";

    for (const auto& msg : ins)
        os << "        if (not " << msg.name << ".empty())\n"
           << "            return false;\n";

    os << "        return true;\n";
    os << "    }\n";

    os << "};\n";

    os << "struct " << name << "_output_port {\n";

    for (const auto& msg : outs)
        os << "    std::vector <" << msg.ref->name << "> " << msg.name << "\n";

    os << "    void clear() noexcept\n"
       << "    {\n";
    for (const auto& msg : outs)
        os << "        " << msg.name << ".clear();\n";
    os << "    }\n";

    os << "    bool empty() const noexcept\n"
       << "    {\n";

    for (const auto& msg : outs)
        os << "        if (not " << msg.name << ".empty())\n"
           << "            return false;\n";

    os << "        return true;\n";
    os << "    }\n";


    os << "};\n";
}

void write_atomic(std::ostream& os, const Atomic& atomic)
{
    write_model_port_type(os, atomic.name, atomic.ins, atomic.outs);

    os << "class " << atomic.name << ":\n"
       << "    public vle::dsde::AtomicModel <Time, "
       << atomic.name << "_input_port, "
       << atomic.name << "_output_port>\n"
       << "{\n"
       << "public:\n"
       << "    " << atomic.name << "(const vle::Context& ctx)\n"
       << "         : AtomicModel(ctx)\n"
       << "    {}\n\n";

    for (const auto& var : atomic.variables)
        write_variable(os, var);

    os << "    enum { ";
    for (auto i = atomic.stas.begin(), e = atomic.stas.end(); i != e; ++i) {
        os << i->name;
        if ((i + 1) != e)
            os << ", ";
    }
    os << " } m_current_state;\n";

    write_state_machine(os, atomic);

    os << "};\n\n";
}

inline
std::string abstract_name_get(const Abstract *model)
{
    assert(model && "abstract_name_get called with null model");

    switch (model->type) {
    case COMPONENT_ATOMIC:
        return static_cast <const Atomic*>(model)->name;
    case COMPONENT_COUPLED:
        return static_cast <const Coupled*>(model)->name;
    }
}

void write_coupled(std::ostream& os, const Coupled& coupled)
{
    write_model_port_type(os, coupled.name, coupled.ins, coupled.outs);

    os << "class " << coupled.name << ":\n"
       << "    public vle::dsde::AtomicModel <Time, "
       << coupled.name << "_input_port, "
       << coupled.name << "_output_port>\n"
       << "{\n"
       << "public:\n"
       << "    " << coupled.name << "(const vle::Context& ctx)\n"
       << "        : CoupledModel(ctx)\n"
       << "    {}\n\n";

    for (const auto& component: coupled.components)
        os << "    " << abstract_name_get(component.model)
                     << " " << component.name << ";\n";

    os << '\n';

    //
    // FIXME perhaps provides two post behaviour: small number of models (based
    // on the UpdatedPort parameter and big number of models base of an
    // class attribute std::multimap.
    //

    os << "    void post(const UpdatedPort& out, UpdatedPort& in) const\n"
       << "    {\n"
       << "        for (const auto& mdl : out) {\n"
       << "            switch (mdl) {\n";

    for (const auto& component: coupled.components) {
        os << "             case &" << component.name << ":\n";

        for (const auto& coupling : coupled.couplings) {
            if (abstract_name_get(coupling.src) == component.name) {
                os << "                "
                   << abstract_name_get(coupling.dst) << ".y = "
                   << abstract_name_get(coupling.dst) << ".x;\n";
            }
        }
    }

    // FIXME missing "this" in input or output message.

    os << "            }\n"
       << "        }\n"
       << "    }\n";


    os << "};\n\n";
}

void write_main(std::ostream& os, const DevsML& data)
{
    os << "int main(int argc, char **argv)\n"
        << "{\n"
        << "    vle::Context ctx = std::make_shared <vle::ContextImpl>();\n"
        << "    DSDE dsde_engine;\n"
        << "    " << data.top->name << " model(ctx);\n"
        << "    vle::SimulationDbg <MyDSDE> sim(ctx, dsde_engine);\n"
        << "    double final_date = sim.run(model, 0., 100.);\n"
        << "}\n"
        << std::endl;
}

void write(const DevsML& data,
           const std::string& output_directory,
           const std::string& /*package_name*/,
           const license& lic)
{
    std::cout << "entities: " << data.entities.size()
              << " atomic: " << data.atomics.size()
              << " coupleds: " << data.coupleds.size()
              << " root: " << data.top
              << '\n';

    if (not data.top)
        return;

    std::string filename(output_directory);
    if (!filename.empty())
        filename += '/';

    filename += data.top->name + ".cpp";

    std::cout << "write: " << filename << '\n';

    std::ofstream ofs(filename);
    if (not ofs)
        throw writer_error(filename, "failed to open");

    ofs << lic << '\n'
        << "#include <vle/echll/dsde/dsde.hpp>\n"
        << "#include <vle/echll/port.hpp>\n"
        << "#include <boost/variant.hpp>\n"
        << "#include <string>\n"
        << "#include <vector>\n"
        << '\n'
        << "using Value = boost::variant <int, double, std::string, bool>;\n"
        << "using Time = vle::DoubleTime;\n"
        << "using time_type = Time::time_type;\n"
        << "using Port = vle::SparsePortList <Value>;\n"
        << "using DSDE = vle::dsde::Engine <Time>;\n"
        << "using AtomicModel = vle::dsde::AtomicModel <Time, Port, Port>;\n"
        << "using CoupledModel = vle::dsde::CoupledModel\n"
        << "                 <Time, Port, Port, Port, Port,\n"
        << "                  vle::dsde::TransitionPolicyDefault <Time>>;\n"
        << '\n';

    write_entities(ofs, data);

    for (const auto& atomic : data.atomics)
        write_atomic(ofs, atomic);

    for (const auto& coupled: data.coupleds)
        write_coupled(ofs, coupled);

    ofs << '\n';

    write_main(ofs, data);
}

}}}
