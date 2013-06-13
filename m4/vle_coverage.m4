dnl Copyright (C) 2008 Vincent Torri <vtorri at univ-evry dot fr>
dnl That code is public domain and can be freely used or copied.

dnl Macro that check if coverage support is wanted and, if yes, if
dnl lcov is available.

dnl Usage: VLE_CHECK_COVERAGE(tests [, ACTION-IF-FOUND [, ACTION-IF-NOT-FOUND]])
dnl The parameter 'tests' is used if a dependency is needed. If set to "yes",
dnl the dependency is available.
dnl Defines VLE_COVERAGE_CFLAGS and VLE_COVERAGE_LIBS variables
dnl Defines the automake conditionnal VLE_ENABLE_COVERAGE

AC_DEFUN([VLE_CHECK_COVERAGE],
[

dnl configure option

AC_ARG_ENABLE([coverage],
   [AC_HELP_STRING([--enable-coverage], [enable coverage profiling instrumentation @<:@default=disabled@:>@])],
   [
    if test "x${enableval}" = "xyes" ; then
       _vle_enable_coverage="yes"
    else
       _vle_enable_coverage="no"
    fi
   ],
   [_vle_enable_coverage="no"])

AC_MSG_CHECKING([whether to use profiling instrumentation])
AC_MSG_RESULT([$_vle_enable_coverage])

dnl lcov check

if test "x$_vle_enable_coverage" = "xyes" && test ! "x$1" = "xyes" ; then
   AC_MSG_WARN([Coverage report requested but tests not being built, disable profiling instrumentation.])
   AC_MSG_WARN([Run configure with --enable-tests])
   _vle_enable_coverage="no"
fi

if test "x$_vle_enable_coverage" = "xyes" ; then
   AC_CHECK_PROG(have_lcov, [lcov], [yes], [no])
   if test "x$have_lcov" = "xyes" ; then
      VLE_COVERAGE_CFLAGS="-fprofile-arcs -ftest-coverage"
      VLE_COVERAGE_LIBS="-lgcov"
# remove any optimisation flag and force debug symbols
      VLE_DEBUG_CFLAGS="-g -O0 -DDEBUG"
   else
      AC_MSG_WARN([lcov is not found, disable profiling instrumentation])
      _vle_enable_coverage="no"
   fi
fi

dnl Substitution
AC_SUBST(VLE_COVERAGE_CFLAGS)
AC_SUBST(VLE_COVERAGE_LIBS)

AM_CONDITIONAL(VLE_ENABLE_COVERAGE, test "x${_vle_enable_coverage}" = "xyes")

AS_IF([test "x$_vle_enable_coverage" = "xyes"], [$2], [$3])
])

dnl End of vle_coverage.m4
