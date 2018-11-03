
# enable versioning
AC_DEFUN([AC_VERSIONING],
[dnl
 AC_ARG_ENABLE([versioning],
 [ --enable-versioning Turn-on versioning of libraries],
 [case "${enableval}" in
   yes) versioning=yes ;;
   no) versioning=no ;;
   *) AC_MSG_ERROR([bad value ${enableval} for --enable-versioning]) ;;
 esac], [versioning=no])
 AM_CONDITIONAL([LIB_VERSIONING], [test x$versioning = xyes])
])

# check for doxygen
AC_DEFUN([AC_CHECK_DOXYGEN],
[dnl

 AC_ARG_WITH([doxygen],
             [AC_HELP_STRING([--with-doxygen@<:@=VALUE@:>@],
                    [using doxygen (VALUE = value of doxygen)])])
 AM_CONDITIONAL([DOXYGEN], [test $with_doxygen])
 AC_SUBST([DOXYGEN])
])

# check for C++17
AX_REQUIRE_DEFINED([AX_CXX_COMPILE_STDCXX])
AC_DEFUN([AX_CXX_COMPILE_STDCXX_17], [AX_CXX_COMPILE_STDCXX([17], [$1], [$2])])

# check verbosity
AC_DEFUN([AC_CHECK_VERBOSITY],
[dnl

 AC_ARG_WITH([verbosity],
             [AC_HELP_STRING([--with-verbosity@<:@=VALUE@:>@],
                    [using verbosity (VALUE = value of verbosity)])])
 AM_CONDITIONAL([VERBOSITY], [test $with_verbosity])
 AC_SUBST([VERBOSITY])
])

# enable static compilation of binaries located in tools
AC_DEFUN([AC_STATIC_TOOLS],
[dnl
 AC_ARG_ENABLE([static-tools],
 [ --enable-static-tools Turn-on compilation of binaries in tools in static mode],
 [case "${enableval}" in
   yes) statictools=yes ;;
   no) statictools=no ;;
   *) AC_MSG_ERROR([bad value ${enableval} for --enable-static-tools]) ;;
 esac], [statictools=no])
 AM_CONDITIONAL([STATIC_TOOLS], [test x$statictools = xyes])
 DISTCHECK_CONFIGURE_FLAGS="$DISTCHECK_CONFIGURE_FLAGS --enable-static-tools=$statictools "
])

# check for eigen3
AC_DEFUN([AC_CHECK_LIB_EIGEN3],
[dnl
 AC_REQUIRE([AC_PROG_CXX])
 AC_LANG_PUSH([C++])

 AC_ARG_WITH([eigen3],
             [AC_HELP_STRING([--with-eigen3@<:@=DIR@:>@],
                    [using eigen3 (DIR = prefix for eigen3 installation)])])
 EIGEN3_CXXFLAGS=''
 EIGEN3_LDFLAGS=''
 if test -n "$with_eigen3"; then
   EIGEN3_CXXFLAGS="-I${with_eigen3}"
   EIGEN3_LDFLAGS=""
 fi
 CXXFLAGS="$CXXFLAGS $EIGEN3_CXXFLAGS"
 LDFLAGS="$LDFLAGS $EIGEN3_LDFLAGS"


 if test -n "$with_eigen3"; then
   DISTCHECK_CONFIGURE_FLAGS="$DISTCHECK_CONFIGURE_FLAGS --with-eigen3=$with_eigen3 "
 fi
 AC_CHECK_HEADER([eigen3/Eigen/Dense],[], [echo eigen3 needed
			   exit -1])
 AC_SUBST([EIGEN3_CXXFLAGS])
 AC_SUBST([EIGEN3_LDFLAGS])

 AC_LANG_POP([C++])
])

# check for boost
AC_DEFUN([AC_CHECK_LIB_BOOST],
[dnl
 AC_REQUIRE([AC_PROG_CXX])
 AC_LANG_PUSH([C++])

 AC_ARG_WITH([boost],
             [AC_HELP_STRING([--with-boost@<:@=DIR@:>@],
                    [using boost (DIR = prefix for boost installation)])])
 BOOST_CXXFLAGS=''
 BOOST_LDFLAGS=''
 if test -n "$with_boost"; then
   BOOST_CXXFLAGS="-I${with_boost}"
   BOOST_LDFLAGS=""
 fi
 CXXFLAGS="$CXXFLAGS $BOOST_CXXFLAGS"
 LDFLAGS="$LDFLAGS $BOOST_LDFLAGS"


 if test -n "$with_boost"; then
   DISTCHECK_CONFIGURE_FLAGS="$DISTCHECK_CONFIGURE_FLAGS --with-boost=$with_boost "
 fi
 AC_CHECK_HEADER([boost/version.hpp],[], [echo boost needed
			   exit -1])
 AC_SUBST([BOOST_CXXFLAGS])
 AC_SUBST([BOOST_LDFLAGS])

 AC_LANG_POP([C++])
])

# check for openmp
AC_DEFUN([AC_CHECK_OPENMP],
[dnl
 AC_REQUIRE([AC_PROG_CXX])
 AC_LANG_PUSH([C++])

 AC_ARG_WITH([openmp],
             [AC_HELP_STRING([--with-openmp@<:@=FLAGS@:>@],
                    [using openmp (FLAGS = compiler specific flag to enagle openmp)])])
 OPENMP_CXXFLAGS=''
 if test -n "$with_openmp"; then
   OPENMP_CXXFLAGS="${with_openmp}"
   OPENMP_LDFLAGS="${with_openmp}"
   DISTCHECK_CONFIGURE_FLAGS="$DISTCHECK_CONFIGURE_FLAGS --with-openmp=$with_openmp "

 else
   OPENMP_CXXFLAGS="-fopenmp"
   OPENMP_LDFLAGS="-fopenmp"
 fi

 if test "x$with_openmp" != xno; then
   old_CXXFLAGS=$CXXFLAGS
   old_LDFLAGS=$LDFLAGS
   CXXFLAGS="$CXXFLAGS $OPENMP_CXXFLAGS"
   LDFLAGS="$LDFLAGS $OPENMP_LDFLAGS"
   AC_CHECK_LIB([gomp], [exit], [], [])
   AC_TRY_COMPILE([# include <omp.h>],
                  [omp_lock_t *_lock = 0;
                 omp_set_lock(_lock);],
                [],
                [echo you should get an openmp compiler
                 CXXFLAGS="$old_CXXFLAGS -DNOPENMP"
                 AC_SUBST([NOPENMP])])
 fi

 AC_LANG_POP([C++])
])
