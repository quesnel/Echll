/*
 * Copyright (C) 2013-2014 INRA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright notice,
 *       this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ORG_VLEPROJECT_KERNEL_GENERIC_HPP
#define ORG_VLEPROJECT_KERNEL_GENERIC_HPP

#include <vle/dsde/dsde.hpp>
#include <boost/format.hpp>
#include <fstream>
#include <memory>
#include <sstream>
#include <unordered_map>

namespace vle {
namespace dsde {

class fileformat_error : std::runtime_error
{
public:
    fileformat_error()
        : std::runtime_error("dsde::fileformat: file format error")
    {}

    fileformat_error(const fileformat_error &) = default;

    fileformat_error(std::size_t idx, std::size_t size)
        : std::runtime_error(
              (boost::format("dsde_fileformat: child index [%1%]"
                             ">= size of the children list (%2%)")
               % idx % size).str())
    {}

    fileformat_error(std::size_t idx)
        : std::runtime_error(
              (boost::format("dsde::fileformat: port index [%1%] too big")
               % idx).str())
    {}

    ~fileformat_error() noexcept
    {}
};

class factory_error : std::runtime_error
{
public:
    factory_error(const std::string &dynamicsname)
        : std::runtime_error(
              (boost::format("dsde_factory: unknown dynamics [%1%]")
               % dynamicsname).str())
    {}

    factory_error(const factory_error &) = default;

    ~factory_error() noexcept
    {}
};

using GenericPort = vle::PortList <boost::any>;

template <typename Time>
class Factory
{
public:
    using time_format = Time;
    using time_type = typename Time::time_type;
    using model_type = Model <Time, GenericPort, GenericPort>;
    using modelptr_type = std::unique_ptr <model_type*>;
    using function_type = std::function <modelptr_type(void)>;

    modelptr_type get(const std::string &dynamicsname) const
    {
        auto it = functions.find(dynamicsname);

        if (it != functions.end())
            return std::move(it->second());

        throw factory_error(dynamicsname);
    }

private:
    std::unordered_map <std::string, function_type> functions;
};

/*
 * @e GenericCoupledModel can read a TGF file format to initialize children
 * and connections.
 *
 * \example
 * Generator
 * Generator
 * Counter
 * #
 * 0 3 0 0
 * 1 2 0 0
 * 2 3 0 0
 * \endexample
 *
 * @TODO Move this class into a dsde-basic ?
 */
template <typename Time, typename Policy = TransitionPolicyThread <Time>>
class GenericCoupledModel :
    public CoupledModel <Time, GenericPort, GenericPort,
                         GenericPort, GenericPort, Policy>
{
public:
    using parent_type = CoupledModel <Time, GenericPort, GenericPort,
                        GenericPort, GenericPort, Policy>;
    using time_format = Time;
    using time_type = typename Time::time_type;
    using model_type = Model <Time, GenericPort, GenericPort>;

    using UpdatedPort = typename parent_type::UpdatedPort;
    using Bag = typename parent_type::Bag;

    typedef std::pair <model_type*, int> inputport;
    typedef std::pair <model_type*, int> outputport;
    typedef std::vector <std::unique_ptr <model_type>> vertices;

    enum ReaderOption {
        INDEXED_BY_INT,
        INDEXED_BY_STRING
    };

    struct hash_inputport {
        std::size_t operator()(const inputport &lhs) const
        {
            std::hash <model_type*> hasher;
            return hasher(lhs.first);
        }
    };

    struct equalto_inputport {
        constexpr bool operator()(const inputport &lhs, const inputport &rhs) const
        {
            return lhs.first == rhs.first && rhs.second == lhs.second;
        }
    };

    typedef std::unordered_multimap <
    inputport, outputport, hash_inputport, equalto_inputport > edges;

    GenericCoupledModel(const Context &ctx)
        : parent_type(ctx)
    {}

    GenericCoupledModel(const Context &ctx, std::size_t x, std::size_t y)
        : parent_type(ctx, x, y)
    {}

    virtual ~GenericCoupledModel()
    {}

    virtual typename parent_type::children_t
    children(const vle::Common & /*common*/) override final
    {
        return {};
    }

    virtual void apply_common(const vle::Common &common)
    {
        (void)common;
    }

    virtual vle::Common update_common(const vle::Common &common,
                                      const GenericCoupledModel::vertices &v,
                                      const GenericCoupledModel::edges &e,
                                      int child)
    {
        (void) common;
        (void) v;
        (void) e;
        (void) child;
        return vle::Common();
    }

    virtual void start(const vle::Common &common,
                       const time_type &time) override
    {
        apply_common(common);
        int tgf_source = common_get <int>(common, "tgf-source");
        int tgf_format = common_get <int>(common, "tgf-format");
        typedef std::shared_ptr <Factory <Time>> FactoryPtr;
        FactoryPtr factory = common_get <FactoryPtr>(common, "tgf-factory");

        if (tgf_source == 0) {
            std::string tgf_filesource = common_get <std::string>(common,
                                         "tgf-filesource");
            std::ifstream ifs(tgf_filesource);

            if (!ifs)
                throw std::invalid_argument(
                    stringf("GenericCoupledModel: failed to open file %s",
                            tgf_filesource.c_str()));

            read(ifs, *factory.get(), (tgf_format == 0) ? INDEXED_BY_INT :
                 INDEXED_BY_STRING);
        } else {
            std::string string_source = common_get <std::string>(common,
                                        "tgf-stringsource");
            std::istringstream iss(string_source);
            read(iss, *factory.get(), (tgf_format == 0) ? INDEXED_BY_INT :
                 INDEXED_BY_STRING);
        }

        typename parent_type::children_t ret;
        ret.reserve(m_children.size());

        for (std::size_t i = 0, e = m_children.size(); i != e; ++i) {
            m_children[i]->parent = this;
            vle::Common localcommon = update_common(common,
                                                    m_children,
                                                    m_connections,
                                                    boost::numeric_cast <int>(i));
            m_children[i]->start(localcommon, time);
            auto id = parent_type::heap.emplace(
                          m_children[i].get(),
                          m_children[i]->tn);
            m_children[i]->heapid = id;
            (*id).heapid = id;
        };

        parent_type::tl = time;
        parent_type::tn = parent_type::heap.top().tn;
    }

    virtual void post(const UpdatedPort &out,
                      UpdatedPort &in) const override final
    {
        for (auto &model : out) {
            std::size_t i = 0;
            std::size_t e = (model == this) ?
                            parent_type::x.size() : model->y.size();

            for (; i != e; ++i) {
                if ((model == this &&
                     !parent_type::x[i].empty()) or
                    (model != this && !model->y[i].empty())) {
                    auto result = m_connections.equal_range(
                                      std::make_pair(
                                          const_cast <model_type*>(model),
                                          i));

                    for (; result.first != result.second; ++result.first) {
                        model_type *dst = const_cast <model_type*>(
                                result.first->second.first);
                        std::size_t portdst = boost::numeric_cast <std::size_t>
                                              (result.first->second.second);
                        in.emplace(dst);

                        if (dst == this) {
                            if (model == this) {
                                assert(false);
                            } else {
                                dst->y[portdst].insert(dst->y[portdst].end(),
                                                       model->y[i].begin(),
                                                       model->y[i].end());
                            }
                        } else {
                            if (model == this) {
                                dst->x[portdst].insert(dst->x[portdst].end(),
                                                       model->x[i].begin(),
                                                       model->x[i].end());
                            } else {
                                dst->x[portdst].insert(dst->x[portdst].end(),
                                                       model->y[i].begin(),
                                                       model->y[i].end());
                            }
                        }
                    }
                }
            }
        }
    }

    vertices m_children;
    edges m_connections;

private:
    void read(std::istream &is, const Factory <Time> &factory,
              ReaderOption opt)
    {
        while (is.good()) {
            std::string dynamics;

            if (is >> dynamics) {
                if (!dynamics.empty() && dynamics[0] != '#') {
                    m_children.emplace_back(factory.get(dynamics));
                } else {
                    break;
                }
            } else
                throw fileformat_error();
        }

        if (opt == INDEXED_BY_INT) {
            while (is.good()) {
                std::size_t model_i, model_j;
                std::size_t port_i, port_j;

                if (is >> model_i >> model_j >> port_i >> port_j) {
                    if (model_i > m_children.size())
                        throw fileformat_error(model_i, m_children.size());

                    if (model_j > m_children.size())
                        throw fileformat_error(model_j, m_children.size());

                    model_type *src;

                    if (model_i == 0) {
                        src = this;

                        if (port_i >= src->x.size())
                            throw fileformat_error(port_i);
                    } else {
                        src = m_children[model_i - 1].get();

                        if (port_i >= src->y.size())
                            throw fileformat_error(port_i);
                    }

                    model_type *dst;

                    if (model_j == 0) {
                        dst = this;

                        if (port_j >= dst->y.size())
                            throw fileformat_error(port_j);
                    } else {
                        dst = m_children[model_j - 1].get();

                        if (port_j >= dst->x.size())
                            throw fileformat_error(port_j);
                    };

                    m_connections.emplace(std::make_pair(src, port_i),
                                          std::make_pair(dst, port_j));
                }
            }
        } else {
            while (is.good()) {
                std::size_t model_i, model_j;
                std::size_t port_i, port_j;
                std::string str_port_i, str_port_j;

                if (is >> model_i >> model_j >> str_port_i >> str_port_j) {
                    if (model_i > m_children.size())
                        throw fileformat_error(model_i, m_children.size());

                    if (model_j > m_children.size())
                        throw fileformat_error(model_j, m_children.size());

                    model_type *src;

                    if (model_i == 0) {
                        src = this;
                        port_i = src->x.add_port(str_port_i);
                    } else {
                        src = m_children[model_i - 1].get();
                        port_i = src->y.add_port(str_port_i);
                    }

                    model_type *dst;

                    try {
                        if (model_j == 0) {
                            dst = this;
                            port_j = dst->y.add_port(str_port_j);
                        } else {
                            dst = m_children[model_j - 1].get();
                            port_j = dst->x.add_port(str_port_j);
                        }
                    } catch (const std::exception &/*e*/) {
                        throw fileformat_error();
                    }

                    m_connections.emplace(std::make_pair(src, port_i),
                                          std::make_pair(dst, port_j));
                }
            }
        }
    }
};

}
}

#endif
