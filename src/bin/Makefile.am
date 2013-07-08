MAINTAINERCLEANFILES = Makefile.in

AM_CPPFLAGS = -include $(top_builddir)/config.h \
	      -DSYSCONFDIR=\""$(sysconfdir)"\" \
	      -DLIBEXECDIR=\""$(libexecdir)"\" \
	      -DLOCALE_DIR=\"@LOCALE_DIR@\" \
	      -I${top_srcdir}/src/lib

bin_PROGRAMS = vle

vle_SOURCES = vle.cpp

vle_LDADD = $(top_builddir)/src/lib/libvle-2.0.la
