/*
 * Copyright (C) 2018  Zhijin Li
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *     * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following disclaimer
 * in the documentation and/or other materials provided with the
 * distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// ---------------------------------------------------------------------------
//
// File: meta.cc for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Sat Nov  3 23:17:42 2018 Zhijin Li
// Last update Sat Nov  3 23:30:39 2018 Zhijin Li
// ---------------------------------------------------------------------------


# include "pasta/core.hh"

int main()
{

  /// Test for is_parentable_v
  auto parentable1 = pasta::is_parentable_v<Eigen::Vector3f,int>;
  auto parentable2 = pasta::is_parentable_v<Eigen::Matrix3f,int,int>;

  auto unparentable1 = pasta::is_parentable_v<std::vector<int>,int>;
  auto unparentable2 = pasta::is_parentable_v<std::vector<int>,int,int>;

  if( !parentable1 || !parentable2 || unparentable1 || unparentable2 )
  {
    std::cerr << "is_parentable_v test failed.\n";

    std::clog << "parentable1 is: " << parentable1 << '\n';
    std::clog << "parentable2 is: " << parentable2 << '\n';

    std::clog << "unparentable1 is: " << unparentable1 << '\n';
    std::clog << "unparentable2 is: " << unparentable2 << '\n';

    return 1;
  }


  /// Test for is_bracketable_v
  auto unbracketable = pasta::is_bracketable_v<int,int>;
  auto bracketable = pasta::is_bracketable_v<std::vector<int>,int>;

  if( !bracketable || unbracketable )
  {
    std::cerr << "is_bracketable_v test failed.\n";

    std::clog << "unbracketable is: " << unbracketable << '\n';
    std::clog << "bracketable is: " << bracketable << '\n';

    return 1;
  }

  return 0;
}
