# Copyright (C) 2018  Zhijin Li

# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

#     * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following disclaimer
# in the documentation and/or other materials provided with the
# distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## ---------------------------------------------------------------------------
##
## File: Makefile.am<github/pasta> for pasta
##
## Created by Zhijin Li
## E-mail:   <jonathan.zj.lee@gmail.com>
##
## Started on  Fri Nov  2 17:00:10 2018 Zhijin Li
## Last update Sat Nov  3 14:14:44 2018 Zhijin Li
## ---------------------------------------------------------------------------


SUBDIRS = libs check

ACLOCAL_AMFLAGS = -I m4

EXTRA_DIST = README.md doc bootstrap.sh

BIN_NAME=${PACKAGE}-${VERSION}-${host_cpu}-`echo ${host_vendor} | \
	sed s/unknown/${host_os}/g`
BIN_TARBALL=${BIN_NAME}.tar.gz
TMP_BUILD=${abs_builddir}/.tmp-bin-build/

dist-hook:
	chmod u+w $(distdir)/doc
	rm -rf `find $(distdir)/doc ! -path . -and ! -path ..		\
		-type d -name ".*"`
	rm -rf `find $(distdir)/doc -type f ! \( -name "*.cc" 		\
		-or -name "*.xml" -or -name "*.css" -or -name "*.md" \)`

doc: 	Doxyfile 			  \
	@top_srcdir@/README.md		  \
	@top_srcdir@/libs/pasta/pasta/*	  \
	@top_srcdir@/libs/pasta/pasta/*/* \
	@top_srcdir@/doc/*		  \
	@top_srcdir@/doc/examples/*
	doxygen

dist-bin:
	rm -rf ${TMP_BUILD}{/*,}
	${MAKE} DESTDIR=${TMP_BUILD} install-strip
	cd ${TMP_BUILD}/${prefix} && find . \( -type f -or -type f \) \
	! -name "*.la" -print0 | xargs -0t tar -czvf                  \
	${abs_builddir}/${BIN_TARBALL}

clean-local:
	rm -rf doc

.PHONY:
	doc
