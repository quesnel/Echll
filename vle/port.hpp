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

    PortList();
    PortList(size_type port_number);

    /**
     * @brief Merge all message to the list.
     * @details Insert all messages from @e lst into the list with the same id.
     *
     * @param lst The input list to merge.
     */
    void merge(const PortList &lst);

    /**
     * @brief Merge specified message type to the list.
     * @details Insert specified messages from the port @e port_src from @e lst
     *     into the list on port @e port_dst.
     *
     * @param lst The input list to merge.
     * @param port_src The identifier of the @e lst output port to copy.
     * @param port_dst The identifier of the input port.
     */
    void merge(const PortList &lst, std::size_t port_src,
               std::size_t port_dst);


    void add_ports(std::size_t number);

    template <class... Args>
    void emplace_back(std::size_t port_id, Args &&...args);

    const element_type &operator[](size_type i) const noexcept;
    element_type &operator[](size_type i);
    const element_type &at(size_type i) const;
    element_type &at(size_type i);

    void clear();
    bool empty() const;
    constexpr std::size_t size() const;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version);

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

    SparsePortList(std::size_t size);

    template <class... Args>
    void emplace_back(std::size_t port_id, Args &&...args);

    /**
     * @brief Merge all message to the list.
     * @details Insert all messages from @e lst into the list with the same id.
     *     For example, if @e lst is: (0, "foo"), (1, "bar"), two messages
     *     "foo" on port 0 and "bar" on port 1 then these messages are copied
     *     into the list with the same port id
     *
     * @param lst The input list to merge.
     */
    void merge(const SparsePortList &lst);

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
               std::size_t port_dst);

    reference front() noexcept;
    const_reference front() const noexcept;
    reference back() noexcept;
    const_reference back() const noexcept;

    iterator begin() noexcept;
    const_iterator begin() const noexcept;
    iterator end() noexcept;
    const_iterator end() const noexcept;

    reverse_iterator rbegin() noexcept;
    const_reverse_iterator rbegin() const noexcept;
    reverse_iterator rend() noexcept;
    const_reverse_iterator rend() const noexcept;

    const_iterator cbegin() const noexcept;
    const_iterator cend() const noexcept;
    const_reverse_iterator crbegin() const noexcept;
    const_reverse_iterator crend() const noexcept;

    bool empty() const noexcept;
    size_type size() const noexcept;

    void clear() noexcept;

private:
    sparse_port_list m_list;
    std::size_t m_size;
};

}

#include <vle/detail/port-implementation.hpp>

#endif
