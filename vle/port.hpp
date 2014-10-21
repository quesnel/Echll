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

#ifndef __VLE_KERNEL_PORT_HPP__
#define __VLE_KERNEL_PORT_HPP__

#include <vle/utils.hpp>
#include <algorithm>
#include <map>
#include <vector>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/string.hpp>

namespace vle {

struct model_port_error : std::invalid_argument
{
    model_port_error(int port)
        : std::invalid_argument(std::to_string(port))
    {}

    model_port_error(const std::string &port)
        : std::invalid_argument(port)
    {}
};

template <typename Value>
struct PortList
{
    typedef std::vector <Value> Values;

    std::vector <Values> ports;
    std::map <std::string, int> accessor;
    bool empty;

    PortList()
        : empty(true)
    {}

    PortList(std::initializer_list < std::string > lst)
        : empty(true)
    {
        for (const auto& str : lst)
            add(str);
    }

    int index(const std::string& name) const
    {
        return accessor.at(name);
    }

    bool exists(const std::string& name) const
    {
        return accessor.find(name) != accessor.end();
    }

    int add()
    {
        ports.emplace_back();

        return ports.size() - 1;
    }

    int add(const std::string &name)
    {
        auto ret = accessor.emplace(name, ports.size());
        if (ret.second)
            ports.emplace_back();

        return ret.first->second;
    }

    const Values& operator[](int i) const
    {
        return ports[i];
    }

    Values& operator[](int i)
    {
        empty = false;

        return ports[i];
    }

    const Values& at(int i) const
    {
        if (i > ports.size())
            throw model_port_error(i);

        return ports[i];
    }

    Values& at(int i)
    {
        empty = false;

        if (i > ports.size())
            throw model_port_error(i);

        return ports[i];
    }

    const Values& operator[](const std::string &name) const
    {
        auto it = accessor.find(name);
        if (it == accessor.end())
            throw model_port_error(name);

        return ports[it->second];
    }

    Values& operator[](const std::string &name)
    {
        empty = false;

        auto it = accessor.find(name);
        if (it == accessor.end())
            throw model_port_error(name);

        return ports[it->second];
    }

    void clear()
    {
        empty = true;

        for (auto& port : ports)
            port.clear();
    }

    bool is_empty() const
    {
        for (const auto& port : ports)
            if (!port.empty())
                return false;

        return true;
    }

    std::size_t size() const
    {
        return ports.size();
    }

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        (void)version;

        ar & ports;
        ar & accessor;
        ar & empty;
    }
};

}

#endif
