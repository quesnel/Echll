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

#ifndef ORG_VLEPROJECT_KERNEL_PORT_HPP
#define ORG_VLEPROJECT_KERNEL_PORT_HPP

#include <vle/utils.hpp>
#include <algorithm>
#include <map>
#include <vector>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>

namespace vle {

class invalid_port : std::invalid_argument
{
public:
    invalid_port(const invalid_port &) = default;

    invalid_port(std::size_t port);

    invalid_port(const std::string &port);

    ~invalid_port() noexcept;
};

class invalid_port_size : std::invalid_argument
{
public:
    invalid_port_size();

    invalid_port_size(const invalid_port_size &) = default;

    ~invalid_port_size() noexcept;
};

template <typename Value>
class PortList
{
public:
    using value_type = Value;
    using element_type = std::vector <value_type>;
    using port_list_type = std::vector <element_type>;
    using const_iterator = typename port_list_type::const_iterator;
    using iterator = typename port_list_type::iterator;
    using reverse_iterator = typename port_list_type::reverse_iterator;
    using const_reverse_iterator = typename port_list_type::const_reverse_iterator;
    using const_reference = typename port_list_type::const_reference;
    using reference = typename port_list_type::reference;
    using size_type = typename port_list_type::size_type;

    PortList()
    {}

    PortList(size_type port_number)
        : ports(port_number)
    {}

    /**
     * @brief Merge all message to the list.
     * @details Insert all messages from @e lst into the list with the same id.
     *
     * @param lst The input list to merge.
     */
    void merge(const PortList &lst)
    {
        for (auto i = 0ul, e = lst.size(); i != e; ++i)
            for (const auto &value : lst[i])
                ports[i].push_back(value);
    }

    /**
     * @brief Merge specified message type to the list.
     * @details Insert specified messages from the port @e port_src from @e lst
     *     into the list on port @e port_dst.
     *
     * @param lst The input list to merge.
     * @param port_src The identifier of the @e lst output port to copy.
     * @param port_dst The identifier of the input port.
     */
    void merge(const PortList &lst, std::size_t port_src, std::size_t port_dst)
    {
        std::copy(lst[port_src].begin(),
                  lst[port_src].end(),
                  std::back_inserter(ports[port_dst]));
    }

    void add_ports(std::size_t number)
    {
        ports.resize(ports.size() + number);
    }

    template <class... Args>
    void emplace_back(std::size_t port_id, Args &&...args)
    {
        if (port_id >= ports.size())
            throw invalid_port(port_id);

        ports[port_id].emplace_back(std::forward <Args>(args)...);
    }

    const element_type &operator[](size_type i) const noexcept
    {
        return ports[i];
    }

    element_type &operator[](size_type i)
    {
        return ports[i];
    }

    const element_type &at(size_type i) const
    {
        if (i >= ports.size())
            throw invalid_port(i);

        return ports[i];
    }

    element_type &at(size_type i)
    {
        if (i >= ports.size())
            throw invalid_port(i);

        return ports[i];
    }

    void clear()
    {
        for (auto &port : ports)
            port.clear();
    }

    bool empty() const
    {
        for (const auto &port : ports)
            if (!port.empty())
                return false;

        return true;
    }

    constexpr std::size_t size() const
    {
        return ports.size();
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        (void)version;
        ar &ports;
    }

private:
    port_list_type ports;
};

template <typename Value>
class SparsePortList
{
public:
    using value_type = Value;
    using element_type = std::pair <std::size_t, value_type>;
    using sparse_port_list = std::vector <element_type>;
    using const_iterator = typename sparse_port_list::const_iterator;
    using iterator = typename sparse_port_list::iterator;
    using reverse_iterator = typename sparse_port_list::reverse_iterator;
    using const_reverse_iterator = typename
                                   sparse_port_list::const_reverse_iterator;
    using const_reference = typename sparse_port_list::const_reference;
    using reference = typename sparse_port_list::reference;
    using size_type = typename sparse_port_list::size_type;

    SparsePortList(std::size_t size)
        : m_size(size)
    {}

    template <class... Args>
    void emplace_back(std::size_t port_id, Args &&...args)
    {
        if (port_id >= m_size)
            throw invalid_port(port_id);

        m_list.emplace_back(port_id, std::forward <Args>(args)...);
    }

    /**
     * @brief Merge all message to the list.
     * @details Insert all messages from @e lst into the list with the same id.
     *     For example, if @e lst is: (0, "foo"), (1, "bar"), two messages "foo"
     *     on port 0 and "bar" on port 1 then these messages are copied into the
     *     list with the same port id
     *
     * @param lst The input list to merge.
     */
    void merge(const SparsePortList &lst)
    {
        m_list.insert(end(), lst.begin(), lst.end());
    }

    /**
     * @brief Merge specified message type to the list.
     * @details Insert specified messages from the port @e port_src from @e lst
     *     into the list on port @e port_dst. For example, if @e lst is: (0,
     *     "foo"), (1, "bar"), two messages "foo" on port 0 and "bar" on port 1
     *     and @e port_src = 0 and @e port_dst = 8, then the message (8, "bar")
     *     is inserted into the list.
     *
     * @param lst The input list to merge.
     * @param port_src The identifier of the @e lst output port to copy.
     * @param port_dst The identifier of the input port.
     */
    void merge(const SparsePortList &lst, std::size_t port_src,
               std::size_t port_dst)
    {
        for (const auto &elem : lst)
            if (elem.first == port_src)
                m_list.emplace_back(port_dst, elem.second);
    }

    reference front() noexcept { return m_list.front(); }
    const_reference front() const noexcept { return m_list.front(); }
    reference back() noexcept { return m_list.back(); }
    const_reference back() const noexcept { return m_list.back(); }

    iterator begin() noexcept { return m_list.begin(); }
    const_iterator begin() const noexcept { return m_list.begin(); }
    iterator end() noexcept { return m_list.end(); }
    const_iterator end() const noexcept { return m_list.end(); }

    reverse_iterator rbegin() noexcept { return m_list.rbegin(); }
    const_reverse_iterator rbegin() const noexcept { return m_list.rbegin(); }
    reverse_iterator rend() noexcept { return m_list.rend(); }
    const_reverse_iterator rend() const noexcept { return m_list.rend(); }

    const_iterator cbegin() const noexcept { return m_list.cbegin(); }
    const_iterator cend() const noexcept { return m_list.cend(); }
    const_reverse_iterator crbegin() const noexcept { return m_list.crbegin(); }
    const_reverse_iterator crend() const noexcept { return m_list.crend(); }

    bool empty() const noexcept { return m_list.empty(); }
    size_type size() const noexcept { return m_list.size(); }

    void clear() noexcept
    {
        m_list.clear();
    }

private:
    sparse_port_list m_list;
    std::size_t m_size;
};

}

#include <vle/detail/port-implementation.hpp>

#endif
