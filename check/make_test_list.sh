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
## File: make_test_list.sh for pasta
##
## Created by Zhijin Li
## E-mail:   <jonathan.zj.lee@gmail.com>
##
## Started on  Fri Nov  2 16:23:36 2018 Zhijin Li
## Last update Sat Nov  3 11:46:10 2018 Zhijin Li
## ---------------------------------------------------------------------------


test_type=$1

bins=""
test_list=""
src_list=

if [ "$test_type" = "core" ]; then

    for src in `\ls $2/*.cc | grep -v \~`
    do
        src_list="$src_list `basename $src`"
    done

elif [ "$test_type" = "sanity" ]; then

    for hdr in `\ls $2/*.hh`
    do
        hdr_name=`basename $hdr`
        test_name=sanity_`echo $hdr_name | sed s/\\.hh//g`
        if [ ! -f $test_name.cc ] ;
        then
	    echo "#include <$3$hdr_name>" > $test_name.cc
	    echo "int main() {}" >> $test_name.cc
        fi
    done

    for src in `\ls *.cc | grep -v \~`
    do
        src_list="$src_list `basename $src`"
    done

fi

src_list=`echo $src_list |tr " " "\n"`

for src in `echo $src_list | sort -u`
do
    src_path=`basename $src`
    test_name=`echo $src_path | sed s/\\\\.cc//g`
    bins="$bins $test_name"
    test_list="$test_list test_$test_name"
    echo `echo -n $test_name| sed s/"-"/_/g`"_SOURCES="$test_name.cc
done

echo "check_PROGRAMS=$bins"
echo "TEST=$test_list"
