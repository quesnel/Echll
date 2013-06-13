/*
 * Copyright (C) 2013 INRA
 * Copyright (C) 2013 ULCO
 *
 * Gauthier Quesnel gauthier.quesnel@users.sourceforge.net
 * Éric Ramat ramat@lisic.univ-littoral.fr
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
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
