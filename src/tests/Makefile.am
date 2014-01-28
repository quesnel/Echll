MAINTAIRNERCLEANFILES = Makefile.in

AM_CPPFLAGS = -include $(top_builddir)/config.h \
	      -DSYSCONFDIR=\""$(sysconfdir)"\" \
	      -DLIBEXECDIR=\""$(libexecdir)"\" \
	      -I${top_srcdir}/src/lib

if VLE_ENABLE_TESTS
#TESTS = flatnetwork recursivenetwork
#noinst_PROGRAMS  = flatnetwork recursivenetwork
#check_PROGRAMS = flatnetwork recursivenetwork
TESTS = utils recursivenetwork engines
noinst_PROGRAMS  = utils recursivenetwork engines
check_PROGRAMS = utils recursivenetwork engines
#flatnetwork_SOURCES = flatnetwork.cpp models.hpp catch.hpp
recursivenetwork_SOURCES = recursivenetwork.cpp models.hpp catch.hpp
utils_SOURCES = utils.cpp catch.hpp
utils_LDADD = $(top_srcdir)/src/lib/libvle-2.0.la
engines_SOURCES = engines.cpp
endif

clean_local:
	rm -fr *.gcno *.gcda
