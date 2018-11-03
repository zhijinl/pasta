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
## File: run_unit_tests.sh for pasta
##
## Created by Zhijin Li
## E-mail:   <jonathan.zj.lee@gmail.com>
##
## Started on  Fri Nov  2 16:04:35 2018 Zhijin Li
## Last update Sat Nov  3 00:38:47 2018 Zhijin Li
## ---------------------------------------------------------------------------


COLOR_FAILED='\033[1;31m'
COLOR_PASSED='\033[1;32m'
END='\033[m'

test_list=$1
verbose=$2
test_output=0

if [ -z $test_list ]
then
    test_list=`\ls *.cc | sed s/'\.cc'//g`
fi

for unit_test in $test_list
do
  echo -n "running unit-test $unit_test ..."
  log=$unit_test.log

  if [ "$verbose" = 1 ]
      then
      \time -p ./$unit_test
      status=`echo $?`
  else
      \time ./$unit_test > $log 2>&1
      status=`echo $?`
      time_output=`tail -n 1 $log`
      IFS=' ' read -ra time_tokens <<< "$time_output"
      time_elapsed=${time_tokens[0]}
  fi

  if [ $status = 0 ]
      then
      echo -e "("$time_elapsed"s)""$COLOR_PASSED"" PASSED ""$END"
      rm $log
  else
      echo -e "("$time_elapsed"s)" $COLOR_FAILED FAILED $END
      test_output=1
  fi

done

exit $test_output
