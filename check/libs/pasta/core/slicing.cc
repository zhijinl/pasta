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
// File: slicing.cc for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 17:38:33 2018 Zhijin Li
// Last update Wed Nov  7 19:16:26 2018 Zhijin Li
// ---------------------------------------------------------------------------


# include "pasta/core.hh"
# include "pasta/utilities.hh"


int main()
{
  /// Defs.
  using dtype = float;
  constexpr int dim = 3;


  /// Create vectors for slicing:
  /// note -> in-place slicing works only for
  /// dynamic vectors (Eigen).
  ///
  Eigen::Matrix<int,1,-1> int_vec(10);
  std::vector<int> int_vec_stl(10);

  std::iota(int_vec.data(), int_vec.data()+int_vec.size(), 0);
  std::iota(int_vec_stl.begin(), int_vec_stl.end(), 0);

  Eigen::Matrix<int,1,-1> int_vec2(int_vec);
  std::vector<int> int_vec_stl2(int_vec_stl);


  /// Define slicing vectors.
  std::vector<short> logical_stl(10);
  Eigen::Matrix<bool,10,1> logical_eig;
  for(int n = 0; n < 10; ++n)
    if(n%2 == 0)
    {
      logical_stl[n] = 42;
      logical_eig(n) = false;
    } else
    {
      logical_stl[n] = 0;
      logical_eig(n) = true;
    }

  pasta::utils::logical_slice(int_vec, logical_stl, 42);
  pasta::utils::logical_slice(int_vec_stl, logical_stl, 42);

  pasta::utils::logical_slice(int_vec2, logical_eig);
  pasta::utils::logical_slice(int_vec_stl2, logical_eig);


  /// Check correctness.
  if( int_vec.size() != 5 || int_vec_stl.size() != 5 ||
      int_vec2.size() != 5 || int_vec_stl2.size() != 5 )
  {
    std::cerr << "logical slice went wrong: size mismatch.\n";
    return 1;
  }
  for(int n = 0; n < 5; ++n)
  {
    if( int_vec(n)%2 != 0 ||  int_vec_stl[n]%2 != 0 )
    {
      std::cerr << "logical slice (42) went wrong: value mismatch.\n";
      return 1;
    }

    if( int_vec2(n)%2 == 0 ||  int_vec_stl2[n]%2 == 0 )
    {
      std::cerr << "logical slice (true) went wrong: value mismatch.\n";
      return 1;
    }
  }

  return 0;
}
