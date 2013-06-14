/*
 * Copyright (C) 2013 INRA
 * Copyright (C) 2013 ULCO
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

#ifndef VLE_2_MODELS_HPP
#define VLE_2_MODELS_HPP

struct double_infinity
{
    static constexpr double negative = -HUGE_VAL;
    static constexpr double positive = HUGE_VAL;
};

typedef vle::Time < double, double_infinity > MyTime;

struct MyValueNull
{
    bool is_null(const std::string& value) const
    {
        return value.empty();
    }
};

typedef vle::Value < std::string, MyValueNull > MyValue;

struct Dynamics
{
    vle::PortList < MyValue > x;
    vle::PortList < MyValue > y;

    Dynamics()
    {
    }

    Dynamics(std::initializer_list < std::string > lst_x,
             std::initializer_list < std::string > lst_y)
        : x(lst_x), y(lst_y)
    {
    }

    virtual ~Dynamics() {}

    virtual double start(double t) = 0;
    virtual double transition(double e) = 0;
    virtual void output() = 0;

    virtual std::string observation() const = 0;
};

struct ModelB : Dynamics
{
    int i;

    ModelB()
        : Dynamics({"in"}, {"out"})
    {}

    virtual ~ModelB()
    {}

    virtual double start(double t)
    {
        (void)t;

        i = 0;

        return 0.;
    }

    virtual double transition(double e)
    {
        (void)e;

        i++;

        return .1;
    }

    virtual void output()
    {}

    virtual std::string observation() const
    {
        return std::to_string(i);
    }
};

struct ModelA : Dynamics
{
    int i;

    ModelA()
        : Dynamics()
    {
        x.add("in");
        y.add("out");
    }

    virtual ~ModelA()
    {}

    virtual double start(double t)
    {
        (void)t;

        i = 0;

        return 0.;
    }

    virtual double transition(double e)
    {
        (void)e;

        i++;

        return 1.;
    }

    virtual void output()
    {}

    virtual std::string observation() const
    {
        return std::to_string(i);
    }
};

struct Counter : Dynamics
{
    int i;

    Counter()
        : Dynamics({"in"}, {}), i(0)
    {}

    virtual ~Counter()
    {}

    virtual double start(double t)
    {
        (void)t;

        return double_infinity::positive;
    }

    virtual double transition(double e)
    {
        (void)e;

        i += x[0].size();

        return double_infinity::positive;
    }

    virtual void output()
    {}

    virtual std::string observation() const
    {
        return std::to_string(i);
    }
};

struct Generator : Dynamics
{
    int i;
    std::mt19937 prng;
    std::normal_distribution < double > dist;

    Generator()
        : Dynamics({}, {"out"}), i(0), prng(1234), dist(0., 5.)
    {}

    virtual ~Generator()
    {}

    virtual double start(double t)
    {
        (void)t;

        return std::abs(dist(prng));
    }

    virtual double transition(double e)
    {
        (void)e;

        i++;

        return std::abs(dist(prng));
    }

    virtual void output()
    {
        y[0] = { std::string("msg"), std::string("msg2") };
    }

    virtual std::string observation() const
    {
        return std::to_string(i);
    }
};


#endif
