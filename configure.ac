##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
m4_define([v_maj], [1])
m4_define([v_min], [999])
m4_define([v_mic], [999])
m4_define([v_rev], m4_esyscmd([(git rev-list --count HEAD 2>/dev/null || echo 0) | tr -d '\n']))dnl
##--   When released, remove the dnl on the below line
dnl m4_undefine([v_rev])
##--   When doing snapshots - change soname. remove dnl on below line
m4_define([relname], [ver-pre-svn-07])
m4_define([v_rel], [-release relname])
##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
m4_ifdef([v_rev], [m4_define([v_ver], [v_maj.v_min.v_mic.v_rev])], [m4_define([v_ver], [v_maj.v_min.v_mic])])
m4_define([lt_cur], m4_eval(v_maj + v_min))
m4_define([lt_rev], v_mic)
m4_define([lt_age], v_min)
##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##

AC_INIT([VLE], [v_ver], [quesnel@users.sourceforge.net])
AC_PREREQ([2.52])
AC_CONFIG_SRCDIR([configure.ac])
AC_CONFIG_MACRO_DIR([m4])

AC_CONFIG_HEADERS([config.h])
AH_TOP([
#ifndef VLE_VFL_CONFIG_H__
#define VLE_VFL_CONFIG_H__
])
AH_BOTTOM([
#endif /* VLE_VFL_CONFIG_H__ */
])

AM_INIT_AUTOMAKE([foreign 1.11 -Wall -Wno-portability silent-rules tar-pax no-dist-gzip dist-xz subdir-objects])
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

AC_GNU_SOURCE

AC_LIBTOOL_WIN32_DLL
AC_PROG_LIBTOOL

##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
m4_ifdef([v_rev], , [m4_define([v_rev], [0])])
m4_ifdef([v_rel], , [m4_define([v_rel], [])])
AC_DEFINE_UNQUOTED(VERS_MAJ, [v_maj], [Major version])
AC_DEFINE_UNQUOTED(VERS_MIN, [v_min], [Minor version])
AC_DEFINE_UNQUOTED(VERS_MIC, [v_mic], [Micro version])
AC_DEFINE_UNQUOTED(VERS_REV, [v_rev], [Revison])
version_info="lt_cur:lt_rev:lt_age"
release_info="v_rel"
AC_SUBST(version_info)
AC_SUBST(release_info)
##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##--##
VMAJ=v_maj
AC_SUBST(VMAJ)

AM_GNU_GETTEXT_VERSION(0.17)

m4_ifdef([AM_GNU_GETTEXT], [
AM_GNU_GETTEXT([external])
po_makefile_in=po/Makefile.in
have_po="yes"
],[
have_po="no"
])
AC_SUBST(LTLIBINTL)
LOCALE_DIR="${localedir}"
AC_SUBST(LOCALE_DIR)

if test "x${POSUB}" = "x" ; then
   have_po="no"
fi

AM_CONDITIONAL([HAVE_PO], [test "x${have_po}" = "xyes"])

### Default options with respect to host
AC_CANONICAL_BUILD
AC_CANONICAL_HOST

# dependencies and options
AC_LANG([C++])
AC_PROG_CXX
AX_CXX_COMPILE_STDCXX_11([noext], [mandatory])

# doxygen program for documentation building
VLE_CHECK_DOXYGEN([build_doc="yes"], [build_doc="no"])

m4_ifdef([v_mic],
   [
    VLE_COMPILER_FLAG([-Wall])
    VLE_COMPILER_FLAG([-Wextra])
    VLE_COMPILER_FLAG([-std=c++11])
   ])

### Checks for compiler characteristics
AC_PROG_CC_STDC
AC_C_CONST
AC_C_BIGENDIAN
AC_HEADER_STDC
AC_CHECK_HEADERS([ctime cstring cstdio cstdlib])

BOOST_REQUIRE

AX_PTHREAD([entable_pthread="ok"], [AC_MSG_ERROR([
	    The pthread library and header file
	    required to build vle. Stopping...
	    Check 'config.log' for more information.])])

AC_ARG_ENABLE([debug],
	      [AS_HELP_STRING([--enable-debug],
			      [enable debug code (default is no)])],
			      [debug="$withval"], [debug=no])

AS_IF([test "x$debug" = xno],
      [AC_DEFINE([VLE_NDEBUG], [1], [Define to disable VLE debug features])])

VLE_CHECK_TESTS([enable_tests="yes"], [enable_tests="no"])

# Check coverage
m4_include([m4/gcov.m4])
AC_TDD_GCOV
AC_SUBST(COVERAGE_CFLAGS)
AC_SUBST(COVERAGE_CXXFLAGS)
AC_SUBST(COVERAGE_LDFLAGS)

AC_CONFIG_FILES([Makefile
		 vle-2.0.pc
		 src/Makefile
		 src/lib/Makefile
		 src/bin/Makefile
		 src/tests/Makefile
		 $po_makefile_in
		 ])

AC_OUTPUT

echo
echo "$PACKAGE $VERSION"
echo
echo "Debug Build............: $debug"
echo "Tests..................: ${enable_tests}"
echo "Coverage...............: ${enable_gcov}"
echo "Documentation..........: ${build_doc}"
echo
echo "Compilation............: make (or gmake)"
echo "  CPPFLAGS.............: $CPPFLAGS"
echo "  CFLAGS...............: $CFLAGS"
echo "  CXXFLAGS.............: $CXXFLAGS"
echo "  LDFLAGS..............: $LDFLAGS"
echo
echo "Installation...........: make install (as root if needed, with 'su' or 'sudo')"
echo "  prefix...............: $prefix"
echo
