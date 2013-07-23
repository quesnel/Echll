MAINTAINERCLEANFILES = Makefile.in

AM_CPPFLAGS = -include $(top_builddir)/config.h \
              -DSYSCONFDIR=\""$(sysconfdir)"\" \
              -DLIBEXECDIR=\""$(libexecdir)"\" \
              -I${top_srcdir}/src/lib

lib_LTLIBRARIES = libvle-2.0.la

vleincludedir = $(includedir)/vle-$(v_maj).$(v_min)/vle
vleinclude_HEADERS = vle/dbg.hpp \
		     vle/environment.hpp \
		     vle/export.hpp \
		     vle/heap.hpp \
		     vle/path.hpp \
		     vle/port.hpp \
		     vle/time.hpp \
		     vle/value.hpp \
		     vle/vle.hpp

libvle_2_0_la_SOURCES = environment.cpp \
			i18n.hpp \
			path.cpp \
			vle/dbg.hpp \
			vle/environment.hpp \
			vle/export.hpp \
			vle/heap.hpp \
			vle/path.hpp \
			vle/port.hpp \
			vle/time.hpp \
			vle/value.hpp \
			vle/vle.hpp
