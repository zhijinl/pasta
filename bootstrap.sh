#!/bin/bash
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
## File: bootstrap.sh for pasta
##
## Created by Zhijin Li
## E-mail:   <jonathan.zj.lee@gmail.com>
##
## Started on  Fri Nov  2 16:02:50 2018 Zhijin Li
## Last update Sat Nov  3 14:18:42 2018 Zhijin Li
## ---------------------------------------------------------------------------


# Init autotools
function init_autotools()
{
    cd $SRC_DIR
    echo > libs/pasta/.hdrs.lst
    echo > check/libs/pasta/core/.tests.lst
    echo > check/libs/pasta/sanity/.tests.lst

    sleep 1
    autoreconf -fvis
}


# Launch configure
function configure()
{
    cd $BUILD_DIR
    $SRC_DIR/configure --prefix=$1 "${CONFIGURE_ARGS[@]}"
}


# Show usage.
function show_usage()
{
    echo -------------------
    echo usage:
    echo "$0 [install-prefix] <extra flags (see README.md)>"
    echo -------------------
    exit 1
}


SRC_DIR=`dirname $0`
BUILD_DIR=`pwd`

if [ $# -lt 1 ]
then
    show_usage
fi

LOCAL_DIR=$1
shift 1
CONFIGURE_ARGS=("$@")

init_autotools
configure $LOCAL_DIR
