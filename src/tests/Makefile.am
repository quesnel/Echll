MAINTAIRNERCLEANFILES = Makefile.in

AM_CPPFLAGS = -include $(top_builddir)/config.h \
	      -DSYSCONFDIR=\""$(sysconfdir)"\" \
	      -DLIBEXECDIR=\""$(libexecdir)"\" \
	      -I${top_srcdir}/src/lib

if VLE_ENABLE_TESTS
TESTS = utils dsde-engine devs-engine dtss-engine
noinst_PROGRAMS  = utils dsde-engine devs-engine dtss-engine
check_PROGRAMS = utils dsde-engine devs-engine dtss-engine

dsde_engine_SOURCES = dsde-engine.cpp catch.hpp
dsde_engine_CXXFLAGS = $(COVERAGE_CXXFLAGS)
dsde_engine_LDFLAGS = $(COVERAGE_LDFLAGS)

devs_engine_SOURCES = devs-engine.cpp catch.hpp
devs_engine_CXXFLAGS = $(COVERAGE_CXXFLAGS)
devs_engine_LDFLAGS = $(COVERAGE_LDFLAGS)

dtss_engine_SOURCES = dtss-engine.cpp catch.hpp
dtss_engine_CXXFLAGS = $(COVERAGE_CXXFLAGS)
dtss_engine_LDFLAGS = $(COVERAGE_LDFLAGS)

utils_SOURCES = utils.cpp catch.hpp
utils_CXXFLAGS = $(COVERAGE_CXXFLAGS)
utils_LDADD = $(top_builddir)/src/lib/libvle-2.0.la
utils_LDFLAGS = $(COVERAGE_LDFLAGS)
endif

clean_local:
	rm -fr *.gcno *.gcda
