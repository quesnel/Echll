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

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#ifndef VLE_NDEBUG
#include <boost/format.hpp>
#include <cassert>
#include <iostream>
namespace vle {

    using boost::format;

}
#endif

namespace vle {

    template <typename Type, typename Infinity >
        struct Time
        {
            static constexpr Type negative_infinity = Infinity::negative;
            static constexpr Type infinity = Infinity::positive;
            typedef Type type;
        };

    template <typename Type, typename Null >
        struct Value
        {
            bool is_null(const Type &t) const
            {
                return Null::is_null(t);
            }

            typedef Type type;
        };

    template <typename Value >
        struct PortList
        {
            typedef std::vector <typename Value::type> Values;

            std::vector <Values> ports;
            std::map <std::string, int> accessor;
            bool empty;

            PortList()
                : empty(true)
            {
            }

            PortList(std::initializer_list < std::string > lst)
                : empty(true)
            {
                std::for_each(lst.begin(), lst.end(),
                              [this](const std::string& str)
                              {
                              add(str);
                              }
                             );
            }

            int add(const std::string &name)
            {
                accessor[name] = ports.size();
                ports.push_back(std::vector <typename Value::type>());
                return ports.size() - 1;
            }

            const Values& operator[](int i) const
            {
                return ports[i];
            }

            const Values& operator[](const std::string &name) const
            {
                return ports[accessor.find(name)->second];
            }

            Values& operator[](int i)
            {
                empty = false;
                return ports[i];
            }

            Values& operator[](const std::string &name)
            {
                empty = false;
                return ports[accessor.find(name)->second];
            }

            void clear()
            {
                empty = true;

                std::for_each(ports.begin(), ports.end(),
                              [](Values& v)
                              {
                              v.clear();
                              });
            }

            bool is_empty() const
            {
                return empty;
            }
        };

    template <typename Time, typename Value, typename Child >
        struct Synchronizer
        {
            Child child;

            typename Time::type run(typename Time::type begin,
                                    typename Time::type end)
            {
                child.start(begin);

                while (child.tn <end) {
                    child.output(child.tn);
                    child.transition(child.tn);
                }

                return child.tn;
            }
        };

    template <typename Time, typename Value, typename Child >
        struct SynchronizerLoop
        {
            Child child;
            typename Time::type begin;

            SynchronizerLoop(typename Time::type begin)
                : begin(begin)
            {
                child.start(begin);
            }

            bool loop(typename Time::type end)
            {
                if (child.tn < end) {
                    child.output(child.tn);
                    child.transition(child.tn);
                }

                return child.tn < end;
            }
        };

    template <typename Time, typename Value, typename ModelPtr >
        struct Simulator
        {
            ModelPtr model;
            typename Time::type tl, tn;

            Simulator(ModelPtr model)
                : model(model), tl(-888), tn(888)
            {
            }

            void start(typename Time::type t)
            {
                tl = t;
                tn = t + model->start(t);
            }

            void transition(typename Time::type time)
            {
                assert(tl <= time && time <= tn);

                if (time < tn and model->x.is_empty())
                    return;

                tn = time + model->transition(time - tl);
                tl = time;
            }

            void output(typename Time::type time)
            {
                if (time == tn)
                    model->output();
            }
        };

    template <typename Time, typename Value, typename GraphManager >
        struct NetworkSimulator
        {
            typedef Simulator <Time, Value, typename GraphManager::Model> Child;

            GraphManager gm;
            std::vector <Child> children;
            typename Time::type tl, tn;

            NetworkSimulator()
                : tl(-9), tn(+9)
            {
                std::for_each(gm.children.begin(),
                              gm.children.end(),
                              [this] (typename GraphManager::Model &mdl)
                              {
                              children.push_back(Child(mdl));
                              }
                             );
            }

            void start(typename Time::type t)
            {
                tl = t;
                tn = Time::infinity;

                std::for_each(children.begin(), children.end(),
                              [this, t](Child &child)
                              {
                              child.start(t);
                              if (child.tn < tn)
                              tn = child.tn;
                              });
            }

            void transition(typename Time::type time)
            {
                assert(tl <= time && time <= tn);

                typename Time::type next = Time::infinity;
                std::for_each(children.begin(), children.end(),
                              [this, time, &next](Child &child)
                              {
                              if (not child.model->x.is_empty() or time == tn) {
                              child.transition(time);

                              if (child.tn < next)
                              next = child.tn;

                              }
                              });

                tn = next;
            }

            void output(typename Time::type time)
            {
                if (time == tn) {
                    std::for_each(children.begin(), children.end(),
                                  [time](Child &child)
                                  {
                                  child.output(time);
                                  });

                    gm.put();
                }
            }
        };

    template <typename Time, typename Value, typename GraphManager >
        struct HierarchicalNetworkSimulator
        {
            typedef Simulator <Time, Value, typename GraphManager::Model> Child;

            GraphManager gm;
            std::vector <Child> children;
            typename Time::type tl, tn;

            HierarchicalNetworkSimulator()
                : tl(-9), tn(+9)
            {
                std::for_each(gm.children.begin(),
                              gm.children.end(),
                              [this] (typename GraphManager::Model &mdl)
                              {
                              children.push_back(Child(mdl));
                              }
                             );
            }

            void start(typename Time::type t)
            {
                tl = t;
                tn = Time::infinity;

                std::for_each(children.begin(), children.end(),
                              [this, t](Child &child)
                              {
                              child.start(t);
                              if (child.tn < tn)
                              tn = child.tn;
                              });
            }

            void transition(typename Time::type time)
            {
                assert(tl <= time && time <= tn);

                typename Time::type next = Time::infinity;
                std::for_each(children.begin(), children.end(),
                              [this, time, &next](Child &child)
                              {
                              if (not child.model->x.is_empty() or time == tn) {
                              child.transition(time);

                              if (child.tn < next)
                              next = child.tn;

                              }
                              });

                tn = next;
            }

            void output(typename Time::type time)
            {
                if (time == tn) {
                    std::for_each(children.begin(), children.end(),
                                  [time](Child &child)
                                  {
                                  child.output(time);
                                  });

                    gm.put();
                }
            }
        };
}
