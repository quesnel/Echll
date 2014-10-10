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

#ifndef __VLE_KERNEL_DSDE_HPP__
#define __VLE_KERNEL_DSDE_HPP__

#include <vle/common.hpp>
#include <vle/time.hpp>
#include <vle/port.hpp>
#include <vle/heap.hpp>
#include <vle/dbg.hpp>
#include <unordered_map>
#include <memory>
#include <set>
#include <thread>

namespace vle {

struct dsde_internal_error : std::logic_error
{
    explicit dsde_internal_error(const std::string& msg)
        : std::logic_error(msg)
    {}
};

namespace dsde {

struct fileformat_error: std::invalid_argument
{
    fileformat_error()
        : std::invalid_argument("dsde::fileformat: file format error")
    {}

    fileformat_error(std::size_t idx, std::size_t size)
        : std::invalid_argument(
            vle::stringf("dsde::fileformat: child index [%" PRIuMAX "]"
                         ">= size of the children list (%" PRIuMAX ")",
                         (std::uintmax_t)idx,
                         (std::uintmax_t)size))
    {}

    fileformat_error(std::size_t idx)
        : std::invalid_argument(
            vle::stringf("dsde::fileformat: port index [%" PRIuMAX "] too big",
                         (std::uintmax_t)idx))
    {}
};

struct factory_error : std::invalid_argument
{
    factory_error(const std::string& dynamicsname)
        : std::invalid_argument(
            vle::stringf("dsde::factory: unknown dynamics [%s]",
                         dynamicsname.c_str()))
    {}
};

template <typename Time>
inline void check_transition_synchronization(typename Time::type tl,
                                             typename Time::type time,
                                             typename Time::type tn)
{
#ifndef VLE_OPTIMIZE
    if (!(tl <= time && time <= tn))
        throw dsde_internal_error("Synchronization error");
#else
    (void)tl;
    (void)time;
    (void)tn;
#endif
}

template <typename Time>
inline void check_output_synchronization(typename Time::type tn,
                                         typename Time::type time)
{
#ifndef VLE_OPTIMIZE
    if (time != tn)
        throw dsde_internal_error("Synchronization error");
#else
    (void)tn;
    (void)time;
#endif
}

template <typename Time, typename Value>
struct ComposedModel;

template <typename Time, typename Value>
struct Model
{
    typedef typename Time::type time_type;
    typedef Value value_type;

    Model()
        : tl(-Time::infinity), tn(Time::infinity), parent(nullptr)
    {}

    Model(std::initializer_list <std::string> lst_x,
          std::initializer_list <std::string> lst_y)
        : x(lst_x), y(lst_y), tl(-Time::infinity), tn(Time::infinity),
          parent(nullptr)
    {}

    virtual ~Model()
    {}

    mutable vle::PortList <Value> x, y;
    time_type tl, tn;
    ComposedModel <Time, Value> *parent;
    typename HeapType <Time, Value>::handle_type heapid;

    virtual void start(const Common& common, const time_type& time) = 0;
    virtual void transition(const time_type& time) = 0;
    virtual void output(const time_type& time) = 0;
};

template <typename Time, typename Value>
using Bag = std::set <Model <Time, Value>*>;

template <typename Time, typename Value>
using UpdatedPort = std::set <const Model <Time, Value>*>;

template <typename Time, typename Value>
struct ComposedModel : Model <Time, Value>
{
    typedef typename Time::type time_type;
    typedef Value value_type;

    ComposedModel()
        : Model <Time, Value>()
    {}

    ComposedModel(std::initializer_list <std::string> lst_x,
                  std::initializer_list <std::string> lst_y)
        : Model <Time, Value>(lst_x, lst_y)
    {}

    virtual ~ComposedModel()
    {}

    virtual void post(const UpdatedPort <Time, Value> &out,
                      UpdatedPort <Time, Value> &in) const = 0;

    UpdatedPort <Time, Value> last_output_list;
    UpdatedPort <Time, Value> root;
};

template <typename Time, typename Value>
struct AtomicModel : Model <Time, Value>
{
    typedef typename Time::type time_type;
    typedef Value value_type;

    virtual time_type init(const vle::Common& common, const time_type& time) = 0;
    virtual time_type delta(const time_type& time) = 0;
    virtual void lambda() const = 0;

    AtomicModel()
        : Model <Time, Value>()
    {}

    AtomicModel(std::initializer_list <std::string> lst_x,
                std::initializer_list <std::string> lst_y)
        : Model <Time, Value>(lst_x, lst_y)
    {}

    virtual ~AtomicModel()
    {}

    virtual void start(const vle::Common& common, const time_type& time) override
    {
        Model <Time, Value>::tl = time;
        Model <Time, Value>::tn = time + init(common, time);
    }

    virtual void transition(const time_type& time) override
    {
        check_transition_synchronization <Time>(Model <Time, Value>::tl,
                                                time,
                                                Model <Time, Value>::tn);

        if (time < Model <Time, Value>::tn and Model <Time, Value>::x.is_empty())
            return;

        Model <Time, Value>::tn = time + delta(time - Model <Time, Value>::tl);
        Model <Time, Value>::tl = time;
        Model <Time, Value>::x.clear();
    }

    virtual void output(const time_type& time) override
    {
        if (time == Model <Time, Value>::tn)
            lambda();
    }
};

template <typename Time, typename Value>
struct TransitionPolicyDefault
{
    typedef typename Time::type time_type;

    void operator()(Bag <Time, Value>& bag, const time_type& time,
                    HeapType <Time, Value> &heap)
    {
        for (auto *child: bag) {
            child->transition(time);
            child->x.clear();

            (*child->heapid).tn = child->tn;
            heap.update(child->heapid);
        }
    }
};

template <typename Time, typename Value>
struct TransitionPolicyThread
{
    typedef typename Time::type time_type;

    std::vector <std::thread> pool;

    TransitionPolicyThread()
        : pool(std::thread::hardware_concurrency())
    {}

    TransitionPolicyThread(const TransitionPolicyThread& /*other*/)
        : pool(std::thread::hardware_concurrency())
    {}

    void work(Bag <Time, Value>& bag, const time_type& time,
              const std::size_t idx)
    {
        if (idx >= bag.size())
            return;

        std::size_t current_job_id = idx;
        auto it = bag.begin();
        std::advance(it, idx);

        for (;;) {
            (*it)->transition(time);
            current_job_id += pool.size();

            if (current_job_id >= bag.size())
                break;

            std::advance(it, pool.size());
        }
    }

    void operator()(Bag <Time,Value>& bag, const time_type& time,
                    HeapType <Time, Value> &heap)
    {
        if (bag.size() == 1) {
            Model <Time, Value>* child = (*bag.begin());

            child->transition(time);
            child->x.clear();
            (*child->heapid).tn = child->tn;
            heap.update(child->heapid);
        } else {
            for (size_t i = 0; i < pool.size(); ++i) {
                pool[i] = std::thread(&TransitionPolicyThread::work, this,
                                      std::ref(bag), time, i);
            }

            for (auto& worker : pool)
                if (worker.get_id() != std::thread::id())
                    worker.join();

            for (auto* child : bag) {
                (*child->heapid).tn = child->tn;
                heap.update(child->heapid);
                child->x.clear();
            }
        }
    }
};

template <typename Time, typename Value,
          typename Policy = TransitionPolicyThread <Time, Value>>
struct CoupledModel : ComposedModel <Time, Value>
{
        typedef typename Time::type time_type;
        typedef Value value_type;
        typedef Policy transition_policy;

        HeapType <Time, Value> heap;
        transition_policy policy;

        typedef std::vector <Model <Time, Value>*> children_t;

        /**
         * @brief Get the children of the @e CoupledModel.
         *
         * The @e children function is called only once by the simulation layer
         * after the constructor.
         *
         * @return
         */
        virtual children_t children(const vle::Common& common) = 0;

        virtual void post(const UpdatedPort <Time, Value> &out,
                          UpdatedPort <Time, Value> &in) const override = 0;

        CoupledModel()
            : ComposedModel <Time, Value>()
        {}

        CoupledModel(std::initializer_list <std::string> lst_x,
                     std::initializer_list <std::string> lst_y)
            : ComposedModel <Time, Value>(lst_x, lst_y)
        {}

        virtual ~CoupledModel()
        {}

        virtual void start(const vle::Common& common, const time_type& time) override
        {
            auto cs = children(common);

            for (auto child : cs) {
                child->parent = this;
                child->start(common, time);

                auto id = heap.emplace(child, child->tn);
                child->heapid = id;
                (*id).heapid = id;
            };

            Model <Time, Value>::tl = time;
            Model <Time, Value>::tn = heap.top().tn;
        }

        virtual void transition(const time_type& time) override
        {
            check_transition_synchronization <Time>(Model <Time, Value>::tl,
                                                    time,
                                                    Model <Time, Value>::tn);

            if (time < Model <Time, Value>::tn && Model <Time, Value>::x.is_empty())
                return;

            Bag <Time, Value> bag;

            {
                auto it = heap.ordered_begin();
                auto et = heap.ordered_end();

                for (; it != et && (*it).tn == time; ++it)
                    bag.insert(
                        reinterpret_cast <Model <Time, Value>*>(
                            (*it).element));
            }

            if (not ComposedModel <Time, Value>::root.empty()) {
                post(ComposedModel <Time, Value>::root,
                     ComposedModel <Time, Value>::last_output_list);
                for (auto& r : ComposedModel <Time, Value>::root) {
                    r->y.clear();
                }
                ComposedModel <Time, Value>::root.clear();
            }

            if (not Model <Time, Value>::x.is_empty()) {
                post({this}, ComposedModel <Time, Value>::last_output_list);
                Model <Time, Value>::x.clear();
            }

            for (auto &child : ComposedModel <Time, Value>::last_output_list)
                bag.insert(const_cast <Model <Time, Value>*>(child));

            ComposedModel <Time, Value>::last_output_list.clear();

            for (auto& b : bag) {
                assert(b != this);
            }

            policy(bag, time, heap);

            Model <Time, Value>::tl = time;
            Model <Time, Value>::tn = heap.top().tn;
        }

        virtual void output(const time_type& time) override
        {
            check_output_synchronization <Time>(heap.top().tn, time);

            if (time == Model <Time, Value>::tn && not heap.empty()) {
                UpdatedPort <Time, Value> lst;
                auto it = heap.ordered_begin();
                auto et = heap.ordered_end();

                assert(it != et);
                assert((std::size_t)std::distance(it, et) == heap.size());

                do {
                    auto id = (*it).heapid;
                    Model <Time, Value> *mdl =
                        reinterpret_cast <Model <Time, Value>*>(
                            (*id).element);

                    mdl->output(time);
                    if (!(mdl->y.is_empty()))
                        lst.emplace(mdl);
                    ++it;
                } while (it != et && it->tn == Model <Time, Value>::tn);

                post(lst, ComposedModel <Time, Value>::last_output_list);

                auto ithis = ComposedModel <Time, Value>::last_output_list.find(this);
                if (ithis != ComposedModel <Time, Value>::last_output_list.end()) {
                    ComposedModel <Time, Value>::last_output_list.erase(ithis);
                    ComposedModel <Time, Value>::parent->root.emplace(this);
                    assert(not (Model <Time, Value>::y.is_empty()));
                }

                for (auto *child : lst)
                    child->y.clear();
            }
        }
    };

template <typename Time, typename Value>
struct Factory
{
    typedef typename Time::type time_type;
    typedef Value value_type;

    typedef std::unique_ptr <Model <Time, Value>> modelptr;
    typedef std::function <modelptr(void)> function_t;

    modelptr get(const std::string& dynamicsname) const
    {
        auto it = functions.find(dynamicsname);
        if (it != functions.end())
            return std::move(it->second());

        throw factory_error(dynamicsname);
    }

    std::unordered_map <std::string, function_t> functions;
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
template <typename Time, typename Value,
         typename Policy = TransitionPolicyDefault <Time, Value>>
struct GenericCoupledModel : CoupledModel <Time, Value, Policy>
{
    typedef typename Time::type time_type;
    typedef Value value_type;
    typedef Policy transition_policy;

    typedef std::pair <Model <Time, Value>*, int> inputport;
    typedef std::pair <Model <Time, Value>*, int> outputport;
    typedef std::vector <std::unique_ptr <Model <Time, Value>>> vertices;

    enum ReaderOption
    {
        INDEXED_BY_INT,
        INDEXED_BY_STRING
    };

    struct hash_inputport
    {
        std::size_t operator()(const inputport& lhs) const
        {
            std::hash <Model <Time, Value>*> hasher;

            return hasher(lhs.first);
        }
    };

    struct equalto_inputport
    {
        constexpr bool operator()(const inputport& lhs,
                                  const inputport& rhs) const
        {
            return lhs.first == rhs.first &&
                rhs.second == lhs.second;
        }
    };

    typedef std::unordered_multimap <
        inputport, outputport, hash_inputport, equalto_inputport> edges;

    vertices m_children;
    edges m_connections;

    void read(std::istream& is, const Factory <Time, Value>& factory,
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

                    Model <Time, Value>* src;

                    if (model_i == 0) {
                        src = this;

                        if (port_i >= src->x.size())
                            throw fileformat_error(port_i);
                    } else {
                        src = m_children[model_i - 1].get();

                        if (port_i >= src->y.size())
                            throw fileformat_error(port_i);
                    }

                    Model <Time, Value>* dst;

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

                    Model <Time, Value>* src;

                    if (model_i == 0) {
                        src = this;
                        port_i = src->x.index(str_port_i);
                    } else {
                        src = m_children[model_i - 1].get();
                        port_i = src->y.index(str_port_i);
                    }

                    Model <Time, Value>* dst;

                    try {
                        if (model_j == 0) {
                            dst = this;
                            port_j = dst->y.index(str_port_j);
                        } else {
                            dst = m_children[model_j - 1].get();
                            port_j = dst->x.index(str_port_j);
                        }
                    } catch (const std::exception& e) {
                        throw fileformat_error();
                    }

                    m_connections.emplace(std::make_pair(src, port_i),
                                          std::make_pair(dst, port_j));
                }
            }
        }
    }

    GenericCoupledModel(std::istream& is,
                        const Factory <Time, Value>& factory,
                        ReaderOption opt = INDEXED_BY_INT)
        : CoupledModel <Time, Value, Policy>()
    {
        read(is, factory, opt);
    }

    GenericCoupledModel(std::initializer_list <std::string> lst_x,
                        std::initializer_list <std::string> lst_y,
                        std::istream& is,
                        const Factory <Time, Value>& factory,
                        ReaderOption opt = INDEXED_BY_INT)
        : CoupledModel <Time, Value, Policy>(lst_x, lst_y)
    {
        read(is, factory, opt);
    }

    virtual typename CoupledModel <Time, Value>::children_t
        children(const vle::Common& /*common*/) override final
        {
            return {};
        }

    virtual vle::Common update_common(
        const vle::Common& common,
        const GenericCoupledModel::vertices& v,
        const GenericCoupledModel::edges& e,
        int child)
    {
        (void) common;
        (void) v;
        (void) e;
        (void) child;

        return vle::Common();
    }

    virtual void start(const vle::Common& common,
                       const time_type& time) override final
    {
        typename CoupledModel <Time, Value>::children_t ret;
        ret.reserve(m_children.size());

        for (int i = 0, e = m_children.size(); i != e; ++i) {
            m_children[i]->parent = this;
            vle::Common localcommon = update_common(common,
                                                    m_children,
                                                    m_connections,
                                                    i);

            m_children[i]->start(localcommon, time);

            auto id = CoupledModel <Time, Value, Policy>::heap.emplace(
                m_children[i].get(),
                m_children[i]->tn);

            m_children[i]->heapid = id;
            (*id).heapid = id;
        };

        Model <Time, Value>::tl = time;
        Model <Time, Value>::tn =
            CoupledModel <Time, Value, Policy>::heap.top().tn;
    }

    virtual void post(const UpdatedPort <Time, Value> &out,
                      UpdatedPort <Time, Value> &in) const override final
    {
        for (auto& model : out) {
            std::size_t i = 0;
            std::size_t e = (model == this) ?
                Model <Time, Value>::x.size() : model->y.size();

            for (; i != e; ++i) {
                if ((model == this &&
                     !Model <Time, Value>::x[i].empty()) or
                    (model != this && !model->y[i].empty())) {
                    auto result = m_connections.equal_range(
                        std::make_pair(
                            const_cast <Model <Time, Value>*>(model),
                            i));

                    for (; result.first != result.second; ++result.first) {
                        Model <Time, Value>* dst =
                            const_cast <Model <Time, Value>*>(
                                result.first->second.first);

                        int portdst = result.first->second.second;

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

    virtual ~GenericCoupledModel()
    {}
};

template <typename Time, typename Value,
          typename Policy = TransitionPolicyThread <Time, Value>>
    struct Executive : ComposedModel <Time, Value>
    {
        typedef typename Time::type time_type;
        typedef Value value_type;
        typedef Policy transition_policy;

        HeapType <Time, Value> heap;

        typedef std::vector <Model <Time, Value>*> children_t;
        time_type chi_tl, chi_tn;
        typename HeapType <Time, Value>::handle_type chi_heapid;
        mutable vle::PortList <Value> chi_x, chi_y;
        transition_policy policy;
        vle::Common localcommon;

        virtual children_t children() = 0;
        virtual time_type init(const time_type& time) = 0;
        virtual time_type delta(const time_type& time) = 0;
        virtual void lambda() const = 0;
        virtual void post(const UpdatedPort <Time, Value> &y,
                          UpdatedPort <Time, Value> &x) const = 0;

        Executive()
            : ComposedModel <Time, Value>()
        {}

        Executive(std::initializer_list <std::string> lst_x,
                  std::initializer_list <std::string> lst_y)
            : ComposedModel <Time, Value>(lst_x, lst_y)
        {}

        Executive(std::initializer_list <std::string> lst_x,
                  std::initializer_list <std::string> lst_y,
                  std::initializer_list <std::string> chi_lst_x,
                  std::initializer_list <std::string> chi_lst_y)
            : ComposedModel <Time, Value>(lst_x, lst_y), chi_tl(-Time::infinity),
              chi_tn(Time::infinity), chi_x(chi_lst_x), chi_y(chi_lst_y)
        {}

        virtual ~Executive()
        {}

        void insert(Model <Time, Value> *mdl)
        {
            mdl->parent = this;
            mdl->start(localcommon, Executive::chi_tl);

            auto id = heap.emplace(mdl, mdl->tn);
            mdl->heapid = id;
            (*id).heapid = id;
        }

        void erase(Model <Time, Value >*mdl)
        {
            ComposedModel <Time, Value>::last_output_list.erase(mdl);
            heap.erase(mdl->heapid);
            mdl->parent = nullptr;
        }

        virtual void start(const vle::Common& common, const time_type& time) override
        {
            localcommon = common;
            chi_tl = time;
            chi_tn = time + init(time);
            auto id = heap.emplace(this, chi_tn);
            chi_heapid = id;
            (*id).heapid = id;

            auto cs = children();
            std::for_each(cs.begin(), cs.end(),
                          [=](Model <Time, Value> *child)
          {
              child->parent = this;
              child->start(localcommon, time);

              auto id = heap.emplace(child, child->tn);
              (*id).heapid = id;
              child->heapid = id;
          });

            Model <Time, Value>::tl = time;
            Model <Time, Value>::tn = heap.top().tn;
            Model <Time, Value>::x.clear();
        }

        virtual void transition(const time_type &time) override
        {
            check_transition_synchronization <Time>(Model <Time, Value>::tl,
                                                    time,
                                                    Model <Time, Value>::tn);

            if (time < Model <Time, Value>::tn && Model <Time, Value>::x.is_empty())
                return;

            Bag <Time, Value> bag;
            bool have_chi = false;

            {
                auto it = heap.ordered_begin();
                auto et = heap.ordered_end();

                for (; it != et && (*it).tn == time; ++it) {
                    if ((*it).element == this)
                        have_chi = true;
                    else
                        bag.insert(
                            reinterpret_cast <Model <Time, Value>*>(
                                (*it).element));
                }
            }

            {
                if (not Model <Time, Value>::x.is_empty())
                    post({this}, ComposedModel <Time, Value>::last_output_list);

                for (auto &child : ComposedModel <Time, Value>::last_output_list) {
                    if (child == this)
                        have_chi = true;
                    else
                        bag.insert(const_cast <Model <Time, Value>*>(child));
                }

                ComposedModel <Time, Value>::last_output_list.clear();
            }

            policy(bag, time, heap);

            if (have_chi) {
                time_type e = time - chi_tl;
                chi_tl = time;
                chi_tn = time + delta(e);
                (*chi_heapid).tn = chi_tn;
                heap.update(chi_heapid);
            }

            Model <Time, Value>::tn = heap.top().tn;
            Model <Time, Value>::tl = time;
            Model <Time, Value>::x.clear();
        }

        virtual void output(const time_type &time) override
        {
            check_output_synchronization <Time>(heap.top().tn, time);

            if (time == Model <Time, Value>::tn && not heap.empty()) {
                UpdatedPort <Time, Value> lst;
                auto it = heap.ordered_begin();
                auto et = heap.ordered_end();

                do {
                    auto id = (*it).heapid;
                    Model <Time, Value> *mdl =
                        reinterpret_cast <Model <Time, Value>*>((*id).element);

                    if (mdl == this) {
                        lambda();
                    } else {
                        mdl->output(time);
                    }
                    if (!(mdl->y.is_empty()))
                        lst.emplace(mdl);
                    ++it;
                } while (it != et && it->tn == Model <Time, Value>::tn);

                post(lst, ComposedModel <Time, Value>::last_output_list);

                std::for_each(lst.begin(), lst.end(),
                              [](const Model <Time, Value> *child)
                              {
                                  child->y.clear();
                              });
            }
        }
    };

template <typename Time, typename Value>
struct Engine
{
    typedef typename Time::type time_type;
    typedef Value value_type;
    typedef Model <Time, Value> model_type;
    vle::CommonPtr common;

    Engine()
        : common(std::make_shared <Common>())
    {}

    Engine(vle::CommonPtr common)
        : common(common)
    {}

    time_type pre(model_type& model, const time_type& time)
    {
        model.start(*common, time);

        return model.tn;
    }

    time_type run(model_type& model, const time_type& time)
    {
        model.output(time);
        model.transition(time);
        model.x.clear();

        return model.tn;
    }

    void post(model_type& model, const time_type& time)
    {
        (void)model;
        (void)time;
    }
};

}}

#endif
