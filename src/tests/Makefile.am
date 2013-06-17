MAINTAIRNERCLEANFILES = Makefile.in

AM_CPPFLAGS = -I$(top_srcdir)/src/lib

if VLE_ENABLE_TESTS
TESTS = flatnetwork recursivenetwork
noinst_PROGRAMS  = flatnetwork recursivenetwork
check_PROGRAMS = flatnetwork recursivenetwork
flatnetwork_SOURCES = flatnetwork.cpp models.hpp catch.hpp
recursivenetwork_SOURCES = recursivenetwork.cpp models.hpp catch.hpp
endif

clean_local:
	rm -fr *.gcno *.gcda
