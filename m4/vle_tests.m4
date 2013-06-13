dnl Copyright (C) 2008 Vincent Torri <vtorri at univ-evry dot fr>
dnl That code is public domain and can be freely used or copied.

dnl Macro that check if tests programs are wanted and if yes, if
dnl the Check library is available.

dnl Usage: VLE_CHECK_TESTS([ACTION-IF-FOUND [, ACTION-IF-NOT-FOUND]])
dnl Define the automake conditionnal VLE_ENABLE_TESTS

AC_DEFUN([VLE_CHECK_TESTS],
[

dnl configure option

AC_ARG_ENABLE([tests],
   [AC_HELP_STRING([--enable-tests], [enable tests @<:@default=disabled@:>@])],
   [
    if test "x${enableval}" = "xyes" ; then
       _vle_enable_tests="yes"
    else
       _vle_enable_tests="no"
    fi
   ],
   [_vle_enable_tests="yes"])

AM_CONDITIONAL(VLE_ENABLE_TESTS, test "x${_vle_enable_tests}" = "xyes")

AS_IF([test "x$_vle_enable_tests" = "xyes"], [$1], [$2])
])

dnl End of vle_tests.m4
