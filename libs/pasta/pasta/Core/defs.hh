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
// File: defs.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Thu Nov  1 23:04:35 2018 Zhijin Li
// Last update Sat Nov  3 15:55:52 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_DEFS_HH
# define PASTA_DEFS_HH

# include <cmath>
# include <vector>
# include <cassert>
# include <iostream>

# include "eigen3/Eigen/Dense"


namespace pasta
{

  /// @ingroup group_const
  ///
  /// @brief Constant expression for mathematical @f$ \pi @f$.
  ///
  constexpr double pst_pi =
    3.1415926535897931159979634685442;


  /// @ingroup group_const
  ///
  /// @brief Constant expression for @f$ \frac{1}{\sqrt{2\pi}} @f$.
  ///
  constexpr double one_over_sqrt2pi =
    0.398942280401432702863218082711682654917240143;


  ///@{
  /// @ingroup group_const
  ///
  /// @brief Constant expression for @f$ \frac{1}{\sqrt{2\pi}^{d}} @f$.
  /// with @f$ d @f$ being the desired dimension.
  ///
  template<int __d> constexpr double one_over_sqrt2pi_pow()
  { return std::pow(one_over_sqrt2pi,__d); }

  template<> constexpr double one_over_sqrt2pi_pow<1>()
  { return 0.398942280401432702863218082711682654917240143; }
  template<> constexpr double one_over_sqrt2pi_pow<2>()
  { return 0.159154943091895345608222100963757839053869247; }
  template<> constexpr double one_over_sqrt2pi_pow<3>()
  { return 0.063493635934240982843235201471543405205011368; }
  template<> constexpr double one_over_sqrt2pi_pow<4>()
  { return 0.025330295910584450791436239569520694203674793; }
  template<> constexpr double one_over_sqrt2pi_pow<5>()
  { return 0.010105326013811645816109496820445201592519879; }
  ///@}


  /// @ingroup group_const
  ///
  /// @brief Constant expression for @f$ \sqrt{2\pi} @f$.
  ///
  constexpr double sqrt2pi =
    2.506628274631000241612355239340104162693023681;


  ///@{
  /// @ingroup group_const
  ///
  /// @brief Constant expression for @f$ \sqrt{2\pi}^{d} @f$.
  /// with @f$ d @f$ being the desired dimension.
  ///
  template<int __d> constexpr double sqrt2pi_pow()
  { return std::pow(sqrt2pi,__d); }

  template<> constexpr double sqrt2pi_pow<1>()
  { return 2.506628274631000241612355239340104162693023681; }
  template<> constexpr double sqrt2pi_pow<2>()
  { return 6.283185307179585343817507236963137984275817871; }
  template<> constexpr double sqrt2pi_pow<3>()
  { return 15.74960994572241546052282501477748155593872070; }
  template<> constexpr double sqrt2pi_pow<4>()
  { return 39.47841760435741775836504530161619186401367188; }
  template<> constexpr double sqrt2pi_pow<5>()
  { return 98.95771780477254253582941601052880287170410156; }
  ///@}


  /// @ingroup group_const
  ///
  /// @brief `pasta` class identity collection.
  ///
  enum class pst_id_list
    {
     PASTA_SCALAR,
     PASTA_VECTOR,
     PASTA_MATRIX,
     PASTA_DYNAMIC_MATRIX,
     PASTA_RANDOM_VAR,
     PASTA_UNKNOWN,
     PASTA_KERNEL,
     PASTA_SAMPLER,
     PASTA_DOMAIN
    };


  /// @ingroup group_const
  ///
  /// @brief Constant tag for un-normalized function.
  ///
  struct unnorm_den_tag_t {}; constexpr unnorm_den_tag_t unnorm_den{};

  /// @ingroup group_const
  ///
  /// @brief Constant tag for inverse cumulative distribution function.
  ///
  struct icdf_tag_t {};  constexpr icdf_tag_t   icdf{};

  /// @ingroup group_const
  ///
  /// @brief Constant tag for probability distribution function.
  ///
  struct pdf_tag_t {};   constexpr pdf_tag_t     pdf{};

  /// @ingroup group_const
  ///
  /// @brief Constant tag for cumulative distribution function.
  ///
  struct cdf_tag_t {};   constexpr cdf_tag_t     cdf{};


  /// @ingroup group_const
  ///
  /// @brief Constant tags for Metropolis-Hasting MCMC sampling types.
  ///
  enum class mcmc { random_walk, independent, general };

  /// @ingroup group_const
  ///
  /// @brief Constant tags for Metropolis-Hasting MCMC sampling types.
  ///
  enum class sampler { inverse, reject, mh_mcmc };


  /// @defgroup group_traits pasta Property Traits
  ///
  /// @brief Properties traits for types used in `pasta` classes.
  ///

  /// @ingroup group_traits
  namespace traits
  {
    /// @ingroup group_traits
    ///
    /// @brief `pasta` default state type traits property.
    ///
    template<typename T> struct specs
    { static const pst_id_list pst_id = pst_id_list::PASTA_UNKNOWN; };
  }


  /// @ingroup group_const
  ///
  /// @brief Tags to specify to not perform **Bessel correction**
  /// for variance and covariance estimation.
  ///
  struct uncorrected_t {};
  constexpr uncorrected_t uncorrected;
}


#endif
