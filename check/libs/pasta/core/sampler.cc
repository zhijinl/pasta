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
// File: sampler.cc for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 17:24:48 2018 Zhijin Li
// Last update Fri Nov  2 17:24:49 2018 Zhijin Li
// ---------------------------------------------------------------------------


#include "pasta/core.hh"
#include "pasta/statistics.hh"


int main()
{
  using dtype = float;
  using Vec = Eigen::Matrix<dtype,-1,1>;


  /// Sampling from a Gaussian distribution.
  dtype mean = 5.0;
  dtype stdv = 10.0;
  pasta::rnd::Gaussian<dtype> gauss_rv(mean,stdv);

  constexpr int n_samples = 5e3;
  Vec samples(n_samples);
  Vec inv_samples(n_samples);
  Vec rej_samples(n_samples);

  gauss_rv.draw(samples);


  /// Constructed inverse & reject sampler.
  pasta::stats::InverseSampler<dtype,1> inv_smplr;
  pasta::stats::RejectSampler<dtype,1> rej_smplr;
  inv_smplr.bind(samples);
  rej_smplr.bind(samples);


  /// Do sampling from empirical estimation.
  inv_smplr.draw(inv_samples);
  rej_smplr.draw(rej_samples);


  auto inv_mean = inv_samples.mean();
  auto inv_stdv = std::sqrt(pasta::stats::var<dtype>(inv_samples));
  auto rej_mean = rej_samples.mean();
  auto rej_stdv = std::sqrt(pasta::stats::var<dtype>(rej_samples));

  std::cout << "Inverse Sampler mean: " << inv_mean << '\n';
  std::cout << "Inverse Sampler stdv: " << inv_stdv << '\n';

  std::cout << "Inverse Sampler mean: " << rej_mean << '\n';
  std::cout << "Inverse Sampler stdv: " << rej_stdv << '\n';

  /// Check sampling corectness.
  if( std::fabs(inv_mean - mean) > mean/2.0 ||
      std::fabs(inv_stdv - stdv) > stdv/4.0 )
  {
    std::cerr << "Inverse sampling robustness test failed.\n";
  }
  if( std::fabs(rej_mean - mean) > mean/2.0 ||
      std::fabs(rej_stdv - stdv) > stdv/4.0 )
  {
    std::cerr << "Rejection sampling robustness test failed.\n";
  }

  return 0;
}
