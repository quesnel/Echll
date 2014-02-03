MAINTAINERCLEANFILES = Makefile.in

AM_CPPFLAGS = -include $(top_builddir)/config.h \
              -DSYSCONFDIR=\""$(sysconfdir)"\" \
              -DLIBEXECDIR=\""$(libexecdir)"\" \
              -I${top_srcdir}/src/lib

lib_LTLIBRARIES = libvle-2.0.la

vleincludedir = $(includedir)/vle-2.0/vle
vleinclude_HEADERS = vle/dbg.hpp \
		     vle/devs.hpp \
		     vle/dsde.hpp \
		     vle/dtss.hpp \
		     vle/environment.hpp \
		     vle/export.hpp \
		     vle/heap.hpp \
		     vle/port.hpp \
		     vle/time.hpp \
		     vle/utils.hpp \
		     vle/vle.hpp

libvle_2_0_la_SOURCES = environment.cpp \
			i18n.hpp \
			path.cpp \
			path.hpp \
			vle/dbg.hpp \
			vle/devs.hpp \
			vle/dsde.hpp \
			vle/dtss.hpp \
			vle/environment.hpp \
			vle/export.hpp \
			vle/heap.hpp \
			vle/port.hpp \
			vle/time.hpp \
		        vle/utils.hpp \
			vle/vle.hpp

libvle_2_0_la_CXXFLAGS = $(COVERAGE_CXXFLAGS)
libvle_2_0_la_LDFLAGS = $(COVERAGE_LDFLAGS)
