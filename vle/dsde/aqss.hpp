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

#ifndef __VLE_KERNEL_DSDE_AQSS_HPP__
#define __VLE_KERNEL_DSDE_AQSS_HPP__

#include <vle/dsde/dsde.hpp>

namespace vle { namespace dsde {

struct aqss_internal_error : std::logic_error
{
    explicit aqss_internal_error(const std::string &msg)
        : std::logic_error(msg)
    {}
};

namespace details {

template <typename Time, typename Value>
struct ArchiveItem
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    ArchiveItem(double value_, time_type date_)
        : value(value_), date(date_)
    {}

    double value;
    time_type date;
};

template <typename Time, typename Value>
struct Archive
{
    typedef std::deque <ArchiveItem <Time, Value>> container_type;
    typedef typename container_type::value_type value_type;
    typedef typename container_type::const_iterator const_iterator;
    typedef typename container_type::iterator iterator;
    typedef typename container_type::reference reference;
    typedef typename container_type::const_reference const_reference;

    reference front() { return lst.front(); }
    const_reference front() const { return lst.front(); }
    reference back() { return lst.back(); }
    const_reference back() const { return lst.back(); }
    reference operator[](int i) { return lst[i]; }
    const_reference operator[](int i) const { return lst[i]; }

    template <typename... _Args>
    void emplace_back(_Args&&... args)
    {
        lst.emplace_back(std::forward <_Args>(args)...);
    }

    void pop_front() { lst.pop_front(); }

    size_t size() const { return lst.size(); }
    bool empty() const { return lst.empty(); }
    void clear() { lst.clear(); }

    container_type lst;
};

template <typename Time, typename Value>
class Integrator : public AtomicModel <Time, Value>
{
public:
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    enum input { X_dot = 0, Quanta = 1 };
    enum output { I_out = 0 };

    Integrator(const Context &ctx)
        : AtomicModel <Time, Value>(ctx, {"X_dot", "Quanta"}, {"I_out"})
    {}

    virtual ~Integrator()
    {}

    virtual time_type init(const vle::Common& common, const time_type& time)
    {
        (void)time;

        boost::optional <double> val = vle::get<double>(common, "X_0");
        if (!val)
            val = 0.0;
        m_state = INIT;

        vle_dbg((Model <Time, Value>::context()),
                "Integrator initialised with %f\n", m_current_value);

        return Time::null();
    }

    virtual void lambda() const
    {
        switch (m_state) {
        case RUNNING:
            AtomicModel <Time, Value>::y[I_out].emplace_back(m_expected_value);
            break;
        case INIT:
            AtomicModel <Time, Value>::y[I_out].emplace_back(m_current_value);
            break;
        default:
            vle_dbg((AtomicModel <Time, Value>::context()),
                    "Integrator %p tries an output transition in state %d\n",
                    this, static_cast <int>(m_state));
            throw aqss_internal_error("integrator lambda error");
        }
    }

    virtual time_type delta(const time_type& e, const time_type& r,
                            const time_type& t)
    {
        if (Time::is_null(r))
            internal(e, r, t);

        if (not AtomicModel <Time, Value>::x.empty())
            external(e, r, t);

        return ta();
    }

    void external(const time_type& e, const time_type& r, const time_type& t)
    {
        (void)e;
        (void)r;

        if (not AtomicModel <Time, Value>::x[Quanta].empty()) {
            m_upthreshold = AtomicModel <Time, Value>::x[Quanta][0];
            m_downthreshold = AtomicModel <Time, Value>::x[Quanta][1];

            if (WAIT_FOR_QUANTA == m_state)
                m_state = RUNNING;
            else if (WAIT_FOR_BOTH == m_state)
                m_state = WAIT_FOR_X_DOT;
        }

        if (not AtomicModel <Time, Value>::x[X_dot].empty()) {
            archive.emplace_back(AtomicModel <Time, Value>::x[X_dot][0], t);

            if (WAIT_FOR_X_DOT == m_state)
                m_state = RUNNING;
            else if(WAIT_FOR_BOTH == m_state)
                m_state = WAIT_FOR_QUANTA;
        }

        if (RUNNING == m_state) {
            m_current_value = current_value(t);
            m_expected_value = expected_value(t);
        }
    }

    void internal(const time_type& e, const time_type& r, const time_type& t)
    {
        (void)e;
        (void)r;

        switch (m_state) {
        case RUNNING:
            {
                m_last_output_value = m_expected_value;
                m_last_output_date = t;
                double last_derivative_value = archive.back().value;
                archive.clear();
                archive.emplace_back(last_derivative_value, t);
                m_current_value = m_expected_value;
                m_state = WAIT_FOR_QUANTA;
            }
            break;
        case INIT:
            m_state = WAIT_FOR_BOTH;
            m_last_output_value = m_current_value;
            m_last_output_date = t;
            break;
        default:
            vle_dbg((AtomicModel <Time, Value>::context()),
                    "Integrator %p tries an internal transition in state %d\n",
                    this, static_cast <int>(m_state));
            throw aqss_internal_error("Integrator delta failure");
        }
    }

    time_type ta() const
    {
        switch (m_state) {
        case  RUNNING:
            {
                double current_derivative = archive.back().value;

                if(0 == current_derivative)
                    return Time::infinity();

                if (current_derivative > 0)
                    return (m_upthreshold - m_current_value)
                        / current_derivative;

                return (m_downthreshold - m_current_value)
                    / current_derivative;
            }
        default:
            return Time::infinity();
            break;
        }
    }

private:
    typedef enum { INIT, WAIT_FOR_QUANTA, WAIT_FOR_X_DOT,
                   WAIT_FOR_BOTH, RUNNING } State;
    State m_state;

    time_type m_last_output_date;

    double m_upthreshold;
    double m_downthreshold;

    double m_last_output_value;
    double m_current_value;
    double m_expected_value;

    bool m_got_quanta;
    bool m_got_x_dot;

    Archive <Time, Value> archive;

    double current_value(const time_type& time) const
    {
        double val = m_last_output_value;

        if (not archive.empty()) {
            for (size_t i = 0, e = archive.size() - 1; i !=e ;++i)
                val += (archive[i + 1].date - archive[i].date)
                    * archive[i].value;

            val += (time - archive.back().date) * archive.back().value;
        }

        return val;
    }

    double expected_value(const time_type& time) const
    {
        (void)time;

        double current_derivative = archive.back().value;

        if (0 == current_derivative)
            return m_current_value;
        else if (current_derivative > 0)
            return m_upthreshold;
        else
            return m_downthreshold;
    }
};

template <typename Time, typename Value>
class AdaptativeQuantifier : public AtomicModel <Time, Value>
{
public:
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    enum input { in = 0 };
    enum output { out = 0 };

    AdaptativeQuantifier(const Context &ctx)
        : AtomicModel <Time, Value>(ctx, {"in"}, {"out"})
    {}

    virtual ~AdaptativeQuantifier()
    {}

    virtual time_type init(const vle::Common &common, const time_type &time)
    {
        (void)time;

        auto adaptative = vle::get<bool>(common, "allow_offsets");
        m_adaptative = (adaptative) ? *adaptative : false;

        if (m_adaptative)
            m_adapt_state = POSSIBLE;
        else
            m_adapt_state = IMPOSSIBLE;

        auto zero_init_offset = vle::get<bool>(common, "zero_init_offset");
        m_zero_init_offset = (zero_init_offset) ? *zero_init_offset : false;

        auto quantum = vle::get<double>(common, "quantum");
        m_step_size = (quantum) ? *quantum : 0.1;

        if (m_step_size <= 0.0)
            throw std::invalid_argument("QSS quantifier: bad quantup");

        auto past_lenght = vle::get<int>(common, "archive_length");
        m_past_length = (past_lenght) ? *past_lenght : 3;

        if (m_past_length <= 2)
            throw std::invalid_argument(
                "QSS quantifier: bad archive length value (should at least 3)");

        vle_dbg((AtomicModel <Time, Value>::context()),
                "QSS quantifier (adaptative = %d, zero_init_offset %d, "
                "quantum = %f, past_lenght = %d)", m_adaptative,
                m_zero_init_offset, m_step_size, m_past_length);

        m_offset = 0;
        m_state = INIT;

        return Time::infinity();
    }

    virtual time_type delta(const time_type& e, const time_type& r,
                            const time_type& t)
    {
        if (Time::is_null(r))
            internal();

        if (not AtomicModel <Time, Value>::x.empty())
            external(e, r, t);

        return ta();
    }

    void external(const time_type& e, const time_type& r, const time_type& t)
    {
        (void)e;
        (void)r;

        double shifting_factor = 0.0;
        int cnt;

        for (size_t i = 0, e = AtomicModel <Time, Value>::x[0].size();
             i != e; ++i) {
            double val = AtomicModel <Time, Value>::x[0][i];

            if (INIT == m_state) {
                init_step_number_and_offset(val);
                update_thresholds();
                m_state = RESPONSE;
            } else {
                cnt = 0;

                while ((val >= m_upthreshold) || (val<=m_downthreshold)) {
                    cnt++;

                    if (val >= m_upthreshold)
                        m_step_number++;
                    else
                        m_step_number--;

                    switch (m_adapt_state) {
                    case IMPOSSIBLE:
                        update_thresholds();
                        break;
                    case POSSIBLE:
                        if (val >= m_upthreshold)
                            store_change(m_step_size, t);
                        else
                            store_change(-m_step_size, t);

                        shifting_factor = 0;
                        shifting_factor = shift_quanta();

                        // Bad shifting value (value should be strictly
                        // positive and less thant 1)
                        assert(0 <= shifting_factor && shifting_factor <= 1);

                        if ((0 != shifting_factor) && (1 != shifting_factor)) {
                            if (val >= m_upthreshold) {
                                update_thresholds(shifting_factor, DOWN);
                            } else {
                                update_thresholds(shifting_factor, UP);
                            }

                            vle_dbg((AtomicModel <Time, Value>::context()),
                                    " Quantifier  %p new quantas while treating"
                                    " new val %f at date %f\n quantizer"
                                    " interval :  [%f, %f]  , amplitude : %f"
                                    " (default amplitude : %f)\n Quantifier"
                                    " %p shifting : %f",  this, val, t,
                                    m_downthreshold, m_upthreshold,
                                    (m_upthreshold - m_downthreshold),
                                    (2 * m_step_size), this, shifting_factor);

                            m_adapt_state=DONE;
                        } else {
                            update_thresholds();
                        }
                        break;
                    case DONE: //equiv to reinit
                        init_step_number_and_offset(val);
                        m_adapt_state = POSSIBLE;
                        update_thresholds();
                        break;
                    }
                }

                if (cnt > 1) {
                    vle_dbg((AtomicModel <Time, Value>::context()), "Warning :"
                            " multiple quanta change at date: %f", t);
                }

                if(0 == cnt) {
                    vle_dbg((AtomicModel <Time, Value>::context()), "Warning :"
                            " useless ext transition call: no quanta change!"
                            " input val : %f (quantizer interval : %f,%f at"
                            " date: %f", val, m_downthreshold, m_upthreshold,
                            t);
                }
            }
        }
        m_state = RESPONSE;
    }

    void internal()
    {
        switch (m_state) {
        case INIT:
            break;
        case IDLE:
            break;
        case RESPONSE:
            m_state = IDLE;
            break;
        }
    }

    virtual void lambda() const
    {
        if (m_has_output_port)
            AtomicModel <Time, Value>::y[out] =
                {m_upthreshold, m_downthreshold};
    }

    time_type ta() const
    {
        switch (m_state) {
        case INIT:
        case IDLE:
            return Time::infinity();
        case RESPONSE:
            return Time::null();
        }

        throw aqss_internal_error("integrator lambda error");
    }

    // vv::Value* observation(const vd::ObservationEvent& /*event*/) const
    // {
    //     vv::Tuple* t = vv::Tuple::create();
    //     if(INIT!=m_state)
    //     {
    //         t->add(m_downthreshold);
    //         t->add(m_upthreshold);
    //         t->add(m_upthreshold-m_downthreshold);
    //         t->add(m_adapt_state);
    //     }
    //     return t;
    // }

private:
    typedef enum { INIT, IDLE, RESPONSE } State;
    State m_state;

    typedef enum { IMPOSSIBLE,POSSIBLE,DONE  } Adapt_State;
    Adapt_State m_adapt_state;

    typedef enum { UP, DOWN } Direction;

    bool m_adaptative;
    bool m_zero_init_offset;

    double m_offset;
    double m_step_size;

    long int m_step_number;
    double m_upthreshold;
    double m_downthreshold;

    Archive <Time, Value> archive;

    unsigned int m_past_length;

    std::string m_output_port_label;
    bool m_has_output_port;

    void update_thresholds()
    {
        m_upthreshold = m_offset + m_step_size * (m_step_number + 1);
        m_downthreshold = m_offset + m_step_size * (m_step_number - 1);
    }

    void update_thresholds(double factor)
    {
        m_upthreshold = m_offset + m_step_size * (m_step_number + (1 - factor));
        m_downthreshold = m_offset + m_step_size
            * (m_step_number - (1 - factor));
    }

    void update_thresholds(double factor, Direction d)
    {
        switch (d) {
        case UP:
            m_upthreshold = m_offset + m_step_size *
                (m_step_number + (1 - factor));
            m_downthreshold = m_offset + m_step_size * (m_step_number - 1);
            break;
        case DOWN:
            m_upthreshold = m_offset + m_step_size * (m_step_number + 1);
            m_downthreshold = m_offset + m_step_size *
                (m_step_number - (1 - factor));
            break;
        }
    }

    void init_step_number_and_offset(double value)
    {
        m_step_number = std::floor(value / m_step_size);

        if (m_zero_init_offset)
            m_offset = 0.0;
        else
            m_offset = value - m_step_number * m_step_size;
    }

    double shift_quanta()
    {
        double factor = 0;

        if (oscillating(m_past_length-1) &&
            archive.back().date - archive.front().date) {
            vle_dbg((AtomicModel <Time, Value>::context()),
                    "Oscillating, archive size=%zu (m_past_length = %u)",
                    archive.size(), m_past_length);
            double acc = 0;
            double local_estim;
            int cnt = 0;

            for (size_t i = 0; i < archive.size() - 2; ++i) {
                if (archive[i+2].date-archive[i].date) {
                    if ((archive.back().value * archive[i+1].value) > 0) {
                        local_estim = 1 - (archive[i+1].date - archive[i].date)
                            / (archive[i + 2].date - archive[i].date);
                    } else {
                        local_estim = (archive[i+1].date - archive[i].date)
                            / (archive[i + 2].date - archive[i].date);
                    }

                    vle_dbg((AtomicModel <Time, Value>::context()),
                            " Quantifier:  date 1 is  %f , date 2 is  %f,"
                            " date 3 is  %f", archive[i].date,
                            archive[i+1].date, archive[i+2].date);
                    vle_dbg((AtomicModel <Time, Value>::context()),
                            " Quantifier: delta time 1  %f ,delta time 2  %f",
                            archive[i+1].date - archive[i].date,
                            archive[i+2].date-archive[i+1].date);
                    vle_dbg((AtomicModel <Time, Value>::context()),
                            " Quantifier: local estim (%ld/%ld) : %f",
                            (i+1), (archive.size() - 2), local_estim);
                    acc+=local_estim;
                    cnt++;
                }
            }

            acc = acc / cnt;

            vle_dbg((AtomicModel <Time, Value>::context()),
                    " Quantifier: Global prospective offset: %f (%d values pooled)",
                    acc, cnt);
            factor = acc;
            archive.clear();
        } else {
            vle_dbg((AtomicModel <Time, Value>::context()),
                    " Quantifier: Not oscillating, archive size = %zu (m_past_length = %u)",
                    archive.size(), m_past_length);
        }

        return factor;
    }

    void store_change(double val, const time_type &time)
    {
        archive.emplace_back(val, time);

        while (archive.size() > m_past_length)
            archive.pop_front();
    }

    bool oscillating(size_t range) const
    {
        if ((range + 1) > archive.size())
            return false;

        for (size_t i = archive.size() - range; i < archive.size() - 1; ++i)
            if (0 < (archive[i].value * archive[i + 1].value))
                return false;

        return true;
    }

    bool monotonous(size_t range) const
    {
        if ((range + 1) > archive.size())
            return false;

        for (size_t i = 0; i < range; ++i)
            if (0 > (archive[i].value * archive[i + 1].value))
                return false;

        return true;
    }
};

template <typename Time, typename Value>
struct Variable : public AtomicModel <Time, Value>
{
    double oldvalue;
    std::vector <double> constants;
    std::vector <double> values;
    std::function <double(const std::vector <double>&, const std::vector <double>&)> compute;

    Variable(const vle::Context& ctx)
        : AtomicModel <Time, Value>(ctx)
        , oldvalue(std::numeric_limits <double>::infinity())
    {
    }

    virtual ~Variable()
    {
    }

    virtual double init(const vle::Common& common, const double& time)
    {
        (void)common;
        (void)time;

        return Time::infinity();
    }

    virtual void lambda() const
    {
        AtomicModel <Time, Value>::y[0] = { values.front() };
    }

    virtual double delta(const double& e, const double& r, const double& t)
    {
        (void)e;
        (void)r;
        (void)t;

        vle_dbg((AtomicModel <Time, Value>::context()), "compute delta at %f\n", t);

        /*
         * TODO by default, we take the first value in the variable
         * list.
         */
        for (std::size_t i = 0, e = AtomicModel <Time, Value>::x.size(); i != e; ++i)
            if (not AtomicModel <Time, Value>::x[i].empty())
                values[i] = AtomicModel <Time, Value>::x[i].front();

        double value = compute(values, constants);
        if (value != oldvalue) {
            values.front() = value;

            vle_dbg((AtomicModel <Time, Value>::context()), "returns %f\n", values.front());

            return Time::null();
        }

        return Time::infinity();
    }
};

} // namespace details

template <typename Time, typename Value>
struct QSS : CoupledModel <Time, Value>
{
    typedef typename Time::time_type time_type;
    typedef Value value_type;

    details::Integrator <Time, Value> m_integrator;
    details::AdaptativeQuantifier <Time, Value> m_quantifier;
    details::Variable <Time, Value> m_variable;

    QSS(const vle::Context &ctx)
        : CoupledModel <Time, Value>(ctx)
        , m_integrator(ctx)
        , m_quantifier(ctx)
        , m_variable(ctx)
    {}

    virtual ~QSS()
    {}

    virtual typename CoupledModel <Time, Value>::children_t
    children(const vle::Common&) override final
    {
        return { &m_integrator, &m_quantifier, &m_variable };
    }

    virtual void post(const UpdatedPort <Time, Value> &out,
                      UpdatedPort <Time, Value> &in)
        const override final
    {
        (void)out;
        (void)in;

        if (!m_quantifier.y.empty())
            copy_values(m_quantifier.y[0], m_integrator.x[0]);

        if (!m_integrator.y.empty()) {
            copy_values(m_integrator.y[0], m_quantifier.x[0]);
            copy_values(m_integrator.y[0], m_variable.x[0]);
        }

        if (!CoupledModel <Time, Value>::x.empty()) {
            copy_values(CoupledModel <Time, Value>::x, m_quantifier.x);
        }
    }
};

}}
#endif
