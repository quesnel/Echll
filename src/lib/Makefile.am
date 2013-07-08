MAINTAINERCLEANFILES = Makefile.in

AM_CPPFLAGS = -include $(top_builddir)/config.h \
	      -DSYSCONFDIR=\""$(sysconfdir)"\" \
	      -DLIBEXECDIR=\""$(libexecdir)"\" \
	      -I${top_srcdir}/src/lib

lib_LTLIBRARIES = libvle-2.0.la

vleincludedir = $(includedir)/vle-$(v_maj).$(v_min)/vle
vleinclude_HEADERS = vle/dsde-classic.hpp \
		     vle/dsde-heap.hpp \
		     vle/environment.hpp \
		     vle/export.hpp \
		     vle/path.hpp \
		     vle/vle.hpp

libvle_2_0_la_SOURCES = vle/dsde-classic.hpp \
			vle/dsde-heap.hpp \
			environment.cpp \
			vle/environment.hpp \
			vle/export.hpp \
			path.cpp \
			vle/path.hpp \
			vle/vle.hpp
