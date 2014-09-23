## Getting Echll

The simplest way to get Echll is to download Echll, install dependencies,
compile and install:

For recent Debian and Ubuntu derivatives:

    apt-get install build-essential cmake g++ clang libboost-dev \
            libboost-serialization-dev libboost-mpi-dev libopenmpi-dev \
	    openmpi-bin

    cd Echll
    mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release ..
    make -j8
    sudo make install

To use clang replace the previous `cmake` command:

    CXX=clang++-libc++ cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release ..

## Writing model

Echll is a C++ template library. Majors template argument are for time and
value between models:

```c++
template <typename T>
struct Infinity
{
    typedef T time_type;
    static constexpr T negative = -std::numeric_limits<T>::infinity();
    static constexpr T positive = std::numeric_limits<T>::infinity();
};

typedef vle::Time <double, Infinity<double>> MyTime;
typedef std::string MyValue;
```

To develop an atomic model:

```c++
struct MyModel : vle::dsde::AtomicModel <MyTime, MyValue>
{
   /* TODO */
};
```
