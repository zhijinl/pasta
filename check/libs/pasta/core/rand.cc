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
// File: rand.cc for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 17:31:11 2018 Zhijin Li
// Last update Wed Nov  7 18:34:31 2018 Zhijin Li
// ---------------------------------------------------------------------------


#define PST_USE_SHARED_RND_ENGINE

# include <chrono>
# include <algorithm>
# include "pasta/core.hh"
# include "pasta/statistics.hh"
# include "pasta/distributions.hh"


template<typename T> using Vector = Eigen::Matrix<T,Eigen::Dynamic,1>;


int main()
{
  using dtype = double;
  dtype threshold = 0.3;


  /// Define r.vs.
  pasta::rnd::Bernoulli       bg(0.3);
  pasta::rnd::Poisson<int>    pg(1.3);
  pasta::rnd::Gaussian<dtype> gg(0.0, 1.0);
  pasta::rnd::RUniform<dtype>    ug(0.0, 1.0);

  int n_samples = 1000;
  Vector<dtype> gauss_samples(n_samples);
  Vector<bool>  berno_samples(n_samples);
  Vector<int>   poiss_samples(n_samples);
  Vector<dtype> runif_samples(n_samples);


  /// Reset global seed & start sampling.
  pasta::utils::reset_shared_engine(42);
  gg.draw(gauss_samples);
  bg.draw(berno_samples);
  pg.draw(poiss_samples);
  ug.draw(runif_samples);


  /// Precision test: Bernoulli.
  dtype mean_bg = berno_samples.template cast<dtype>().sum()
    /berno_samples.size();
  if( std::fabs(mean_bg-0.3) > threshold )
  {
    std::cerr << "Bernoulli sampling went wrong: precision test failed.\n";
    return 1;
  }


  /// Precision test: Poisson.
  dtype mean_pg = poiss_samples.template cast<dtype>().mean();
  dtype var_pg = pasta::stats::var<dtype>(poiss_samples.template cast<dtype>());
  if( std::fabs(mean_pg-1.3) > threshold ||
      std::fabs(var_pg-1.3) > threshold )
  {
    std::cerr << "Poisson sampling went wrong: precision test failed.\n";
    return 1;
  }


  /// Precision test: Gaussian.
  dtype mean_gg = gauss_samples.mean();
  dtype var_gg = pasta::stats::var<dtype>(gauss_samples);
  if( std::fabs(mean_gg) > threshold ||
      std::fabs(var_gg-1.0) > threshold )
  {
    std::cerr << "Gaussian sampling went wrong: precision test failed.\n";
    return 1;
  }


  /// Precision test: Real Uniform.
  dtype mean_ug = runif_samples.mean();
  dtype var_ug = pasta::stats::var<dtype>(runif_samples);
  if( std::fabs(mean_ug-0.5) > threshold ||
      std::fabs(var_ug-1.0/12) > threshold )
  {
    std::cerr << "Real Uniform sampling went wrong: precision test failed.\n";
    return 1;
  }


  /// Duplicates test: Gaussian.
  if( std::unique(gauss_samples.data(),
                  gauss_samples.data()+gauss_samples.size()) !=
      gauss_samples.data()+gauss_samples.size() )
  {
    std::cerr << "Gaussian sampling went wrong: duplicates test failed.\n";
    return 1;
  }


  /// Duplicates test: Real Uniform.
  if( std::unique(runif_samples.data(),
                  runif_samples.data()+runif_samples.size()) !=
      runif_samples.data()+runif_samples.size() )
  {
    std::cerr << "Real Uniform sampling went wrong: duplicates test failed.\n";
    return 1;
  }


  /// Test random engine reset.
  pasta::utils::reset_shared_engine(42);
  gg.reset_state();

  Vector<dtype> gauss_samples2(n_samples);
  gg.draw(gauss_samples2);

  for(int i = 0; i < n_samples; ++i)
  {
    if( gauss_samples(i) != gauss_samples2(i) )
    {
      std::cerr << "Random engine reset went wrong. Values mismatch.\n";
      return 1;
    }
  }

  return 0;
}
