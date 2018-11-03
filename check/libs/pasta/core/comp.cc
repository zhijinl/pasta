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
// File: comp.cc for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 17:43:39 2018 Zhijin Li
// Last update Sat Nov  3 14:28:01 2018 Zhijin Li
// ---------------------------------------------------------------------------


# include "pasta/core.hh"
# include "pasta/utilities.hh"


int main()
{

  /// Roll some vectors.
  using dtype = double;
  int n_samples = 1e5;


  /// Random vectors.
  Eigen::Matrix<dtype,Eigen::Dynamic,1> rnd_vec =
    Eigen::Matrix<dtype,Eigen::Dynamic,1>::Random(n_samples);

  std::vector<dtype> rnd_vec_stl;
  rnd_vec_stl.reserve(n_samples);
  for(int n = 0; n < n_samples; ++n) rnd_vec_stl.push_back(rnd_vec(n));
  for(int n = 0; n < rnd_vec.size(); ++n)
  {
    if( rnd_vec_stl[n] != rnd_vec(n) )
    {
      std::cerr << "vector copy failed.\n";
      return 1;
    }
  }


  /// Int vectors with ones.
  Eigen::Matrix<int,1,-1> int_vec = Eigen::Matrix<int,1,-1>::Ones(n_samples);

  std::vector<int> int_vec_stl(n_samples);
  std::fill(int_vec_stl.begin(), int_vec_stl.end(), 1);


  /// Test cumsum results.
  auto cum_int = pasta::utils::cumsum(int_vec);
  auto cum_int_stl = pasta::utils::cumsum(int_vec_stl);

  for(int i = 0; i < cum_int.size(); ++i)
  {
    if( cum_int(i) != i+1 || cum_int_stl[i] != i+1 )
    {
      std::cerr << "cumsum results went wrong..\n";
      return 1;
    }
  }


  /// Do ascendant & descendant indexing.
  auto indexing_a = pasta::utils::ascend_indexing(rnd_vec);
  auto indexing_b = pasta::utils::descend_indexing(rnd_vec);
  auto indexing_a2 = pasta::utils::ascend_indexing(rnd_vec_stl);
  auto indexing_b2 = pasta::utils::descend_indexing(rnd_vec_stl);


  /// Check ordering correctness of ascendant indexing.
  if( std::adjacent_find(indexing_a.data(),
                         indexing_a.data()+indexing_a.size(),
                         [&rnd_vec](int prev, int next)
                         { return rnd_vec(prev) > rnd_vec(next); })
      != indexing_a.data()+indexing_a.size() )
  {
    std::cerr << "ascend indexing order failure.\n";
    return 1;
  }
  if( std::adjacent_find(indexing_a2.data(),
                         indexing_a2.data()+indexing_a.size(),
                         [&rnd_vec](int prev, int next)
                         { return rnd_vec(prev) > rnd_vec(next); })
      != indexing_a2.data()+indexing_a2.size() )
  {
    std::cerr << "ascend indexing order failure.\n";
    return 1;
  }


  /// Check ordering correctness of descendant indexing.
  if( std::adjacent_find(indexing_b.data(),
                         indexing_b.data()+indexing_b.size(),
                         [&rnd_vec](int prev, int next)
                         { return rnd_vec(prev) < rnd_vec(next); })
      != indexing_b.data()+indexing_b.size() )
  {
    std::cerr << "descend indexing order failure.\n";
    return 1;
  }
  if( std::adjacent_find(indexing_b2.data(),
                         indexing_b2.data()+indexing_b.size(),
                         [&rnd_vec](int prev, int next)
                         { return rnd_vec(prev) < rnd_vec(next); })
      != indexing_b2.data()+indexing_b2.size() )
  {
    std::cerr << "descend indexing order failure.\n";
    return 1;
  }


  /// Check propagation of types.
  Eigen::Matrix<int,1,10> fixed_int_vec = Eigen::Matrix<int,1,10>::Ones();
  auto __tmp_cum = pasta::utils::cumsum(fixed_int_vec);
  auto __tmp_asc = pasta::utils::ascend_indexing(fixed_int_vec);
  auto __tmp_des = pasta::utils::descend_indexing(fixed_int_vec);

  if( pasta::eigen_cols_v<decltype(__tmp_cum)>() != 10 ||
      pasta::eigen_cols_v<decltype(__tmp_asc)>() != 10 ||
      pasta::eigen_cols_v<decltype(__tmp_des)>() != 10 )
  {
    std::cerr << "something went wrong with fixed size propagation..\n";
    return 1;
  }

  return 0;
}
