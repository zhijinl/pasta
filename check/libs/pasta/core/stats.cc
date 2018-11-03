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
// File: stats.cc for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 17:32:22 2018 Zhijin Li
// Last update Sat Nov  3 00:06:14 2018 Zhijin Li
// ---------------------------------------------------------------------------


# include "pasta/core.hh"
# include "pasta/statistics.hh"


int main()
{

  /// Defs.
  using dtype = float;
  constexpr int size = 100000;
  using Vec = Eigen::Matrix<dtype,Eigen::Dynamic,1>;


  /// Create Eigen & STL vectors.
  pasta::rnd::Gaussian<dtype> grv(1.0, 2.0);
  pasta::rnd::Gaussian<dtype> grv2(2.0, 5.0);

  Vec vec_eigen(size);
  Eigen::Matrix<dtype,2,-1> mat_eigen(2,size);
  std::vector<dtype> vec_stl(size);

  for(auto &i:vec_stl) i = grv.draw();
  for(int i = 0; i < size; ++i) vec_eigen(i) = vec_stl[i];
  for(int i = 0; i < size; ++i)
  {
    mat_eigen(0,i) = grv.draw();
    mat_eigen(1,i) = grv2.draw();
  }

  for(auto i = 0; i < vec_eigen.size(); ++i)
  {
    if( !pasta::utils::f_equal(vec_stl[i],vec_eigen(i)) )
    {
      std::cerr << "vector value equality failed." << std::endl;
      return 1;
    }
  }


  /// Check mean & var computation.
  if( std::fabs(vec_eigen.mean()-pasta::stats::mean<dtype>(vec_stl)) ||
      std::fabs(pasta::stats::mean<dtype>(vec_stl) - 1.0) > 0.1)
  {
    std::cerr << "STL mean and Eigen mean mismatch.\n";
    return 1;
  }

  if( std::fabs(pasta::stats::var<dtype>(vec_eigen)-
                pasta::stats::var<dtype>(vec_stl)) > 0.1)
  {
    std::cerr << "STL var and Eigen var mismatch.\n";
    return 1;
  }


  /// Check mean & covariance computation. For matrix.
  Eigen::Matrix<dtype,2,1> truth_mean{1.0, 2.0};
  Eigen::Matrix<dtype,2,2> truth_cov;
  truth_cov << 4.0, 0.0,
    0.0, 25.0;

  if( (truth_mean-pasta::stats::mean<dtype>(mat_eigen)).norm() > 0.1)
  {
    std::cerr << "Eigen matrix mean went wrong.\n";
    return 1;
  }

  if( (truth_cov-pasta::stats::var<dtype>(mat_eigen)).squaredNorm() > 0.5)
  {
    std::cerr << "Eigen covariance matrix went wrong.\n";
    return 1;
  }



  /// Compyte Kernel smoothing density of Gaussian samples.
  dtype stdev = 10;
  int n_samples = 100000;

  Vec gauss_vec(n_samples);
  pasta::rnd::Gaussian<dtype> gauss_rv(0,stdev);
  gauss_rv.draw(gauss_vec);


  auto domain =
    pasta::utils::make_discrete_domain<dtype>(gauss_vec, 100, 1.2);

  pasta::stats::GaussianKS<dtype,1> gauss_ks;
  auto ks_den = gauss_ks.estimate(pasta::pdf,gauss_vec,domain);

  dtype mse = 0.0;
  for(auto n = 0; n < ks_den.n_elem(); ++n)
  {
    auto err = gauss_rv(ks_den.point_at(n)) - ks_den.value_at(n);
    mse += err*err;
  }
  mse /= ks_den.n_elem();


  if( mse > 1e-4 )
  {
    std::cerr << "ks_density failed precision test.\n";
    return 1;
  }


  return 0;
}
