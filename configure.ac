# dnl Copyright (C) 2018  Zhijin Li

# dnl All rights reserved.

# dnl Redistribution and use in source and binary forms, with or without
# dnl modification, are permitted provided that the following conditions are
# dnl met:

# dnl     * Redistributions of source code must retain the above copyright
# dnl notice, this list of conditions and the following disclaimer.
# dnl     * Redistributions in binary form must reproduce the above
# dnl copyright notice, this list of conditions and the following disclaimer
# dnl in the documentation and/or other materials provided with the
# dnl distribution.

# dnl THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# dnl "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# dnl LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# dnl A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# dnl OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# dnl SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# dnl LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# dnl DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# dnl THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# dnl (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# dnl OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## ---------------------------------------------------------------------------
##
## File: configure.ac for pasta
##
## Created by Zhijin Li
## E-mail:   <jonathan.zj.lee@gmail.com>
##
## Started on  Fri Nov  2 16:57:16 2018 Zhijin Li
## Last update Fri Nov  9 23:06:12 2018 Zhijin Li
## ---------------------------------------------------------------------------


AC_PREREQ(2.59)
AC_INIT([Pasta Library], 0.1.0, jonathan.zj.lee@gmail.com, pasta)
AC_CONFIG_HEADER([libs/pasta/pasta/build_info.hh])

PASTA_API_VERSION_CURRENT=0
PASTA_API_VERSION_REV=1
PASTA_API_VERSION_AGE=0

AC_CONFIG_AUX_DIR([m4])
AC_CONFIG_SRCDIR([m4/pasta.m4])
AC_CONFIG_SRCDIR([m4/ax_cxx_compile_stdcxx.m4])
AM_INIT_AUTOMAKE([subdir-objects tar-pax foreign])
AC_CONFIG_MACRO_DIR([m4])

for var in CC CFLAGS  CPPFLAGS CXX CXXCPP LDFLAGS LIBS CPP CXXFLAGS; do
  eval isset=\${$var+set}
  if test "$isset" = 'set' ; then
    eval val=$`echo $var`
    DISTCHECK_CONFIGURE_FLAGS="${DISTCHECK_CONFIGURE_FLAGS}'${var}=${val}' "
  fi
done

# Require standard version C++17.
AX_CXX_COMPILE_STDCXX_17([noext],[mandatory])

# Compiler flags.
CXXFLAGS="-O3 -W -fomit-frame-pointer   \
          -Wall -Wextra -fopenmp -msse2 \
          -Wno-deprecated-declarations  \
          -Wno-parentheses              \
          -DPASTA_BUILD"

# Checks for programs.
CXXFLAGS_SAVE="$CXXFLAGS"
AC_PROG_CXX
CXXFLAGS="$CXXFLAGS_SAVE"
AC_PROG_CC
AC_PROG_LIBTOOL

AC_SUBST([PASTA_API_VERSION_CURRENT])
AC_SUBST([PASTA_API_VERSION_AGE])
AC_SUBST([PASTA_API_VERSION_REV])

AC_VERSIONING
AC_CHECK_OPENMP
AC_CHECK_LIB_BOOST
AC_CHECK_LIB_EIGEN3
AC_STATIC_TOOLS

AC_SUBST(DISTCHECK_CONFIGURE_FLAGS)
AC_SUBST(PROJS_PATH)

# Checks for header files.
AC_HEADER_DIRENT

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_CONST
AC_C_INLINE

# Checks for library functions.
AC_FUNC_CLOSEDIR_VOID
AC_CHECK_FUNCS([pow sqrt])

AC_CONFIG_FILES([Makefile
                 libs/Makefile
                 libs/pasta/Makefile
                 Doxyfile
                 check/Makefile
                 check/libs/Makefile
                 check/libs/pasta/Makefile
                 check/libs/pasta/core/Makefile
                 check/libs/pasta/sanity/Makefile
		 ])

AC_OUTPUT
