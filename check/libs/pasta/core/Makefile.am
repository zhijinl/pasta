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
## File: Makefile.am<core> for pasta
##
## Created by Zhijin Li
## E-mail:   <jonathan.zj.lee@gmail.com>
##
## Started on  Fri Nov  2 16:18:24 2018 Zhijin Li
## Last update Sat Nov  3 11:50:09 2018 Zhijin Li
## ---------------------------------------------------------------------------


check_PROGRAMS=foo
include .tests.lst

MAKE_TEST_LIST=$(top_srcdir)/check/make_test_list.sh

LIBPASTAPATH=${top_srcdir}/libs/pasta

AM_CPPFLAGS = -I$(top_srcdir)/libs/pasta
AM_LDFLAGS = -lstdc++fs

sanity_%.cc: $(srcdir)/.tests.lst

test_%: %
	@$(top_srcdir)/check/run_unit_tests.sh $< ${VERBOSE}

$(srcdir)/.tests.lst: $(top_srcdir)/configure $(LIBPASTAPATH)/pasta/*.hh $(srcdir)/*.cc
	$(MAKE_TEST_LIST) core $(srcdir) > $@

check-local: ${TEST}
