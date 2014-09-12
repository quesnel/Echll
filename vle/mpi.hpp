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

#ifndef __VLE_KERNEL_MPI_HPP__
#define __VLE_KERNEL_MPI_HPP__

#include <vle/common.hpp>
#include <vle/time.hpp>
#include <vle/port.hpp>
#include <vle/utils.hpp>
#include <vle/heap.hpp>
#include <vle/dsde.hpp>
#include <boost/mpi/environment.hpp>
#include <boost/mpi/communicator.hpp>
#include <stdexcept>

namespace vle { namespace dsde {

enum SynchronousTags
{
    proxy_send_ended_simulation_tag,
    proxy_send_start_tag,
    proxy_recv_tn_tag,
    proxy_send_transition_tag,
    proxy_send_output_tag,
    proxy_recv_output_tag
};

template <typename Time, typename Value>
struct SynchronousProxyModel : Model <Time, Value>
{
    typedef typename Time::type time_type;
    typedef Value value_type;

    boost::mpi::environment* environment;
    boost::mpi::communicator* communicator;
    int rank;

    SynchronousProxyModel()
        : Model <Time, Value>()
          , environment(nullptr)
          , communicator(nullptr)
          , rank(-1)
    {}

    virtual ~SynchronousProxyModel()
    {
        if (communicator && (*communicator))
            communicator->send(rank, proxy_send_ended_simulation_tag);
    }

    virtual void start(const Common& common, const time_type& time) override
    {
        (void) common;

        communicator->send(rank, proxy_send_start_tag, time);

        time_type tmp;
        communicator->recv(rank, proxy_recv_tn_tag, tmp);

        Model <Time, Value>::tl = time;
        Model <Time, Value>::tn = time + tmp;
    }

    virtual void transition(const time_type& time)
    {
        communicator->send(rank, proxy_send_transition_tag, time);
        communicator->send(rank, proxy_send_transition_tag, Model <Time, Value>::x);

        time_type tn;
        communicator->recv(rank, proxy_recv_tn_tag, tn);

        Model <Time, Value>::tn = tn;
        Model <Time, Value>::tl = time;
        Model <Time, Value>::x.clear();
    }

    virtual void output(const time_type& time)
    {
        communicator->send(rank, proxy_send_output_tag, time);
        communicator->recv(rank, proxy_recv_output_tag, Model <Time, Value>::y);
    }
};

template <typename Time, typename Value>
struct SynchronousLogicalProcessor
{
    typedef typename Time::type time_type;
    typedef Value value_type;
    typedef Model <Time, Value> model_type;

    vle::CommonPtr common;

    boost::mpi::environment *environment;
    boost::mpi::communicator *communicator;
    int parent;

    SynchronousLogicalProcessor()
        : common(std::make_shared <Common>())
          , environment(nullptr)
          , communicator(nullptr)
          , parent(-1)
    {}

    SynchronousLogicalProcessor(vle::CommonPtr common)
        : common(common)
          , environment(nullptr)
          , communicator(nullptr)
          , parent(-1)
    {}

    /*
     * @e SynchronousLogicalProcessor is a children of an another
     * @e SynchronousLogicalProcessor. We wait for request form the parent.
     */
    void run(model_type& model)
    {
        assert(parent > -1);
        assert(environment);
        assert(communicator);

        time_type time;

        for (;;) {
            boost::mpi::status msg = communicator->probe();

            assert(msg.source() == parent);

            switch (msg.tag()) {
            case proxy_send_ended_simulation_tag:
                communicator->recv(parent, proxy_send_ended_simulation_tag);
                return;
            case proxy_send_start_tag:
                communicator->recv(parent, proxy_send_start_tag, time);
                model.start(*common, time);
                communicator->send(parent, proxy_recv_tn_tag, model.tn);
                break;
            case proxy_send_transition_tag:
                communicator->recv(parent, proxy_send_transition_tag, time);
                communicator->recv(parent, proxy_send_transition_tag, model.x);
                model.transition(time);
                communicator->send(parent, proxy_recv_tn_tag, model.tn);
                break;
            case proxy_send_output_tag:
                communicator->recv(parent, proxy_send_output_tag, time);
                model.output(time);
                communicator->send(parent, proxy_recv_output_tag, model.y);
                break;
            default:
                throw std::invalid_argument(
                    stringf("[SynchronousLP] unknown tag %d from process %d to"
                            " process %d", msg.tag(), msg.source(),
                            communicator->rank()));
            }
        }
    }
};

}}

#endif
