/*
 * Copyright (C) 2015 INRA
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

#ifndef ORG_VLEPROJECT_KERNEL_PORT_IMPLEMENTATION_HPP
#define ORG_VLEPROJECT_KERNEL_PORT_IMPLEMENTATION_HPP

#include <boost/format.hpp>

namespace vle {

invalid_port::invalid_port(std::size_t port)
    : std::invalid_argument(
          (boost::format("invalid_port: unknown port id %1%") % port).str())
{}

invalid_port::invalid_port(const std::string &port)
    : std::invalid_argument(
          (boost::format("invalid_port: unknown port %1%") % port).str())
{}

invalid_port::~invalid_port() noexcept
{}

invalid_port_size::invalid_port_size()
    : std::invalid_argument("invalid_port_size: copy failed")
{}

invalid_port_size::~invalid_port_size() noexcept
{}

template <typename Value>
PortList <Value>::PortList()
{
}

template <typename Value>
PortList <Value>::PortList(size_type port_number)
    : ports(port_number)
{}

template <typename Value>
void PortList <Value>::merge(const PortList &lst)
{
    for (std::size_t i = 0ul, e = lst.size(); i != e; ++i)
        for (const auto &value : lst[i])
            ports[i].push_back(value);
}

template <typename Value>
void PortList <Value>::merge(const PortList &lst, std::size_t port_src,
                             std::size_t port_dst)
{
    std::copy(lst[port_src].begin(),
              lst[port_src].end(),
              std::back_inserter(ports[port_dst]));
}

template <typename Value>
void PortList <Value>::add_ports(std::size_t number)
{
    ports.resize(ports.size() + number);
}

template <typename Value>
template <class... Args>
void PortList <Value>::emplace_back(std::size_t port_id, Args &&...args)
{
    if (port_id >= ports.size())
        throw invalid_port(port_id);

    ports[port_id].emplace_back(std::forward <Args>(args)...);
}

template <typename Value>
const typename PortList<Value>::element_type &
PortList <Value>::operator[](size_type i) const noexcept
{
    return ports[i];
}

template <typename Value>
typename PortList<Value>::element_type &
PortList <Value>::operator[](size_type i)
{
    return ports[i];
}

template <typename Value>
const typename PortList<Value>::element_type &
PortList <Value>::at(size_type i) const
{
    if (i >= ports.size())
        throw invalid_port(i);

    return ports[i];
}

template <typename Value>
typename PortList<Value>::element_type &
PortList <Value>::at(size_type i)
{
    if (i >= ports.size())
        throw invalid_port(i);

    return ports[i];
}

template <typename Value>
void PortList <Value>::clear()
{
    for (auto &port : ports)
        port.clear();
}

template <typename Value>
bool PortList <Value>::empty() const
{
    for (const auto &port : ports)
        if (!port.empty())
            return false;

    return true;
}

template <typename Value>
constexpr std::size_t PortList <Value>::size() const
{
    return ports.size();
}

template <typename Value>
template<class Archive>
void PortList <Value>::serialize(Archive &ar, const unsigned int version)
{
    (void)version;
    ar &ports;
}

template <typename Value>
SparsePortList<Value>::SparsePortList(std::size_t size)
    : m_size(size)
{}

template <typename Value>
template <class... Args>
void SparsePortList<Value>::emplace_back(std::size_t port_id, Args &&...args)
{
    if (port_id >= m_size)
        throw invalid_port(port_id);

    m_list.emplace_back(port_id, std::forward <Args>(args)...);
}

template <typename Value>
void SparsePortList<Value>::merge(const SparsePortList &lst)
{
    m_list.insert(end(), lst.begin(), lst.end());
}

template <typename Value>
void SparsePortList<Value>::merge(const SparsePortList &lst,
                                  std::size_t port_src,
                                  std::size_t port_dst)
{
    for (const auto &elem : lst)
        if (elem.first == port_src)
            m_list.emplace_back(port_dst, elem.second);
}

template <typename Value>
typename SparsePortList<Value>::reference
SparsePortList<Value>::front() noexcept
{
    return m_list.front();
}

template <typename Value>
typename SparsePortList<Value>::const_reference
SparsePortList<Value>::front() const noexcept
{
    return m_list.front();
}

template <typename Value>
typename SparsePortList<Value>::reference
SparsePortList<Value>::back() noexcept
{
    return m_list.back();
}

template <typename Value>
typename SparsePortList<Value>::const_reference
SparsePortList<Value>::back() const noexcept
{
    return m_list.back();
}

template <typename Value>
typename SparsePortList<Value>::iterator
SparsePortList<Value>::begin() noexcept
{
    return m_list.begin();
}

template <typename Value>
typename SparsePortList<Value>::const_iterator
SparsePortList<Value>::begin() const noexcept
{
    return m_list.begin();
}

template <typename Value>
typename SparsePortList<Value>::iterator
SparsePortList<Value>::end() noexcept
{
    return m_list.end();
}

template <typename Value>
typename SparsePortList<Value>::const_iterator
SparsePortList<Value>::end() const noexcept
{
    return m_list.end();
}

template <typename Value>
typename SparsePortList<Value>::reverse_iterator
SparsePortList<Value>::rbegin() noexcept
{
    return m_list.rbegin();
}

template <typename Value>
typename SparsePortList<Value>::const_reverse_iterator
SparsePortList<Value>::rbegin() const noexcept
{
    return m_list.rbegin();
}

template <typename Value>
typename SparsePortList<Value>::reverse_iterator
SparsePortList<Value>::rend() noexcept
{
    return m_list.rend();
}

template <typename Value>
typename SparsePortList<Value>::const_reverse_iterator
SparsePortList<Value>::rend() const noexcept
{
    return m_list.rend();
}

template <typename Value>
typename SparsePortList<Value>::const_iterator
SparsePortList<Value>::cbegin() const noexcept
{
    return m_list.cbegin();
}

template <typename Value>
typename SparsePortList<Value>::const_iterator
SparsePortList<Value>::cend() const noexcept
{
    return m_list.cend();
}

template <typename Value>
typename SparsePortList<Value>::const_reverse_iterator
SparsePortList<Value>::crbegin() const noexcept
{
    return m_list.crbegin();
}

template <typename Value>
typename SparsePortList<Value>::const_reverse_iterator
SparsePortList<Value>::crend() const noexcept
{
    return m_list.crend();
}

template <typename Value>
bool SparsePortList<Value>::empty() const noexcept
{
    return m_list.empty();
}

template <typename Value>
typename SparsePortList<Value>::size_type
SparsePortList<Value>::size() const noexcept
{
    return m_list.size();
}

template <typename Value>
void SparsePortList<Value>::clear() noexcept
{
    m_list.clear();
}

template <typename Value>
template<class Archive>
void SparsePortList<Value>::serialize(Archive &ar, const unsigned int version)
{
    (void)version;

    ar & m_size;
    ar & m_list;
}

}

#endif
