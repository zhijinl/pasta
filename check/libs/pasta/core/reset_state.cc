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
// File: reset_state.cc for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 17:28:02 2018 Zhijin Li
// Last update Mon Nov  5 23:52:25 2018 Zhijin Li
// ---------------------------------------------------------------------------


#define PST_USE_SHARED_ENGINE

# include "pasta/core.hh"
# include "pasta/utilities.hh"
# include "pasta/statistics.hh"
# include "pasta/distributions.hh"


struct TargetDistr
{
  template<typename Value> Value operator()(Value val) const
  {
    return std::exp(-0.5*(val-5)*(val-5)/100) +
      2*std::exp(-0.5*(val-50)*(val-50)/100);
  }
};

int main()
{

  using dtype = float;

  /// Test rest_state for distributions.
  pasta::rnd::Gaussian<float>  grv(1.0, 3.0);
  pasta::rnd::Bernoulli brv(0.7);
  pasta::rnd::Poisson<int>   prv(42.0);
  pasta::rnd::RUniform<float>  urv(2.0, 7.0);

  pasta::utils::reset_shared_engine(42);

  auto grv_val = grv.draw();
  auto brv_val = brv.draw();
  auto prv_val = prv.draw();
  auto urv_val = urv.draw();

  for( int n = 0; n < 100; ++n )
  {
    pasta::utils::reset_shared_engine(42);
    grv.reset_state();
    brv.reset_state();
    prv.reset_state();
    urv.reset_state();

    auto new_grv_val = grv.draw();
    auto new_brv_val = brv.draw();
    auto new_prv_val = prv.draw();
    auto new_urv_val = urv.draw();

    if( grv_val != new_grv_val ||
        brv_val != new_brv_val ||
        prv_val != new_prv_val ||
        urv_val != new_urv_val )
    {
      std::cerr << "distribution reset_state failed.\n"
                << "EXPECT: "
                << grv_val << ' '
                << grv_val << ' '
                << grv_val << ' '
                << grv_val << "\n"
                << "GOT: "
                << new_grv_val << ' '
                << new_grv_val << ' '
                << new_grv_val << ' '
                << new_grv_val << "\n";
      return 1;
    }
  }

  /// Check reset_state for Metroplis-Hasting Monte Carlo sampler.
  pasta::utils::reset_shared_engine(76);

  auto target_distr = TargetDistr{};
  auto propos_distr = pasta::rnd::Gaussian<dtype>{0,10};

  int n_samples = 5e3;
  Eigen::Matrix<dtype,-1,1> vec(n_samples);
  pasta::stats::MHSampler<dtype,1,TargetDistr>
    mh_sampler(target_distr, propos_distr);

  mh_sampler.draw(vec, propos_distr.draw());

  pasta::utils::reset_shared_engine(76);
  mh_sampler.reset_state();
  propos_distr.reset_state();

  Eigen::Matrix<dtype,-1,1> vec2(n_samples);
  mh_sampler.draw(vec2, propos_distr.draw());

  for(int n = 0; n < n_samples; ++n)
  {
    if( vec(n) != vec2(n) )
    {
      std::cerr << "MH sampler sample value mismatch.\n"
                << "EXPECT: " << vec(n) << " | GOT: "
                << vec2(n) << '\n';
      return 1;
    }
  }

  return 0;
}
