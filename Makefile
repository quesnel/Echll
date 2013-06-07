all: target

CXXFLAGS=-pedantic -O0 -Werror -Wall -Wextra -fvisibility=hidden \
	 -std=gnu++11 -I.
CXX=g++-4.8
LIBS=
SRC := $(wildcard *.cpp)
OBJ := $(patsubst %.cpp,%.o,$(SRC))
DEP := $(patsubst %.cpp,%.deps,$(SRC))

-include $(DEP)

target: flatnetwork recursivenetwork

%.deps: %.cpp Makefile
	$(CXX) -MM $< >$@ $(CXXFLAGS)

%.o: %.cpp Makefile
	$(CXX) -o $@ -c $< $(CXXFLAGS)

flatnetwork: flatnetwork.o
	$(CXX) $(LIBS) -o $@ $^

recursivenetwork: recursivenetwork.o
	$(CXX) $(LIBS) -o $@ $^

clean:
	-@rm recursivenetwork flatnetwork *.o *.deps

test: flatnetwork recursivenetwork
	./flatnetwork -o %stderr > /dev/null
	./recursivenetwork -o %stderr > /dev/null
