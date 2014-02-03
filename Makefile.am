ACLOCAL_AMFLAGS = -I m4

AM_MAKEFLAGS = --no-print-directory

SUBDIRS = src

if HAVE_PO
SUBDIRS += po
endif

MAINTAINERCLEANFILES = ABOUT-NLS \
		       Makefile.in \
		       aclocal.m4 \
		       config.guess \
		       config.h.in \
		       config.h.in~ \
		       config.rpath \
		       config.sub \
		       configure \
		       depcomp \
		       install-sh \
		       ltmain.sh \
		       missing \
		       mkinstalldirs \
		       $(PACKAGE_TARNAME)-$(PACKAGE_VERSION).tar.gz \
		       $(PACKAGE_TARNAME)-$(PACKAGE_VERSION).tar.bz2 \
		       $(PACKAGE_TARNAME)-$(PACKAGE_VERSION)-doc.tar.bz2 \
		       boost.m4 \
		       codeset.m4 \
		       gettext.m4 \
		       glibc21.m4 \
		       glibc2.m4 \
		       iconv.m4 \
		       intdiv0.m4 \
		       intldir.m4 \
		       intl.m4 \
		       intlmacosx.m4 \
		       intmax.m4 \
		       inttypes_h.m4 \
		       inttypes-pri.m4 \
		       lcmessage.m4 \
		       lib-ld.m4 \
		       lib-link.m4 \
		       lib-prefix.m4 \
		       libtool.m4 \
		       lock.m4 \
		       longlong.m4 \
		       lt~obsolete.m4 \
		       ltoptions.m4 \
		       ltsugar.m4 \
		       ltversion.m4 \
		       nls.m4 \
		       po.m4 \
		       printf-posix.m4 \
		       progtest.m4 \
		       size_max.m4 \
		       stdint_h.m4 \
		       uintmax_t.m4 \
		       visibility.m4 \
		       wchar_t.m4 \
		       wint_t.m4 \
		       xsize.m4

EXTRA_DIST = AUTHORS \
	     COPYING \
	     README \
	     autogen.sh \
	     vle-2.0.pc.in \
	     m4/ax_cxx_compile_stdcxx_11.m4 \
	     m4/boost.m4 \
	     m4/gcov.m4 \
	     m4/vle_compiler_flag.m4 \
	     m4/vle_doxygen.m4 \
	     m4/vle_tests.m4

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = vle-2.0.pc

if HAVE_PO
MAINTAINERCLEANFILES += po/boldquot.sed \
			po/en@boldquot.header \
			po/en@quot.header \
			po/insert-header.sin \
			po/Makefile.in.in \
			po/Makevars.template \
			po/quot.sed \
			po/remove-potcdate.sin \
			po/Rules-quot
endif

if HAVE_GCOV
.PHONY: clean-gcda
clean-gcda:
	@echo Removing old coverage results
	-find -name '*.gcda' -print | xargs -r rm

.PHONY: coverage-html generate-coverage-html clean-coverage-html
coverage-html: clean-gcda
	-$(MAKE) $(AM_MAKEFLAGS) -k check
	$(MAKE) $(AM_MAKEFLAGS) generate-coverage-html

generate-coverage-html:
	@echo Collecting coverage data
	$(LCOV) --directory $(top_builddir) --capture --output-file coverage.info --no-checksum --compat-libtool
	LANG=C $(GENHTML) --prefix $(top_builddir) --output-directory coveragereport --title "Code Coverage" --legend --show-details coverage.info

clean-coverage-html: clean-gcda
	-$(LCOV) --directory $(top_builddir) -z
	-rm -rf coverage.info coveragereport

clean-local: clean-coverage-html

endif # HAVE_GCOV
