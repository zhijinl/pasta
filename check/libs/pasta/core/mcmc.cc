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
// File: mcmc.cc for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 17:30:32 2018 Zhijin Li
// Last update Fri Nov  2 17:30:40 2018 Zhijin Li
// ---------------------------------------------------------------------------


# include "pasta/core.hh"
# include "pasta/statistics.hh"
# include "pasta/distributions.hh"


struct TargetDistr
{
  template<typename Value> Value operator()(Value val) const
  {
    return std::exp(-0.5*(val-5)*(val-5)/100) + 2*std::exp(-0.5*(val-50)*(val-50)/100);
  }
};


int main()
{

  /// Definitions.
  using dtype = double;
  using distr = TargetDistr;
  using propo = pasta::rnd::Gaussian<dtype>;

  distr target_distr = distr{};
  propo propos_distr = propo{0,10};

  pasta::stats::MHSampler<dtype,1,distr> mh_sampler(target_distr, propos_distr);


  int n_samples = 5e3;
  Eigen::Matrix<dtype,-1,1> vec(n_samples);


  /// Run Metropolis-Hasting Markov chain Monte Carlo.
  mh_sampler.draw(vec, propos_distr.draw());


  /// Test for MSE: sample pdf vs analytical pdf.
  pasta::stats::GaussianKS<dtype,1> ks_estimator;
  auto emp_pdf = ks_estimator.estimate(pasta::pdf, vec);

  dtype mse = 0.0;
  dtype constant = 75.1;
  for(auto n = 0; n < emp_pdf.n_elem(); ++n)
  {
    auto err = target_distr(emp_pdf.point_at(n))/constant
      - emp_pdf.value_at(n);
    mse += err*err;
  }
  mse /= emp_pdf.n_elem();

  if( mse > 1e-4 || vec.mean() > 45 || vec.mean() < 25)
  {
    std::cerr << "MH-MCMC went wrong.\n";
    return 1;
  }


  return 0;
}
