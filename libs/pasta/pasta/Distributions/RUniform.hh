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
// File: RUniform.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:48:39 2018 Zhijin Li
// Last update Tue Nov  6 23:22:50 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_RUNIFORM_HH
# define PASTA_RUNIFORM_HH

# include "distrbase.hh"


namespace pasta
{
  // Fwd decl.
  namespace rnd { template<typename T> class RUniform; }

  /// @ingroup group_traits
  namespace traits
  {

    /// @ingroup group_traits
    ///
    /// @brief Type traits properties for the `pasta::rnd::RUniform`
    /// class.
    ///
    template<typename T> struct specs<rnd::RUniform<T> >
    {
      static const pst_id_list pst_id = pst_id_list::PASTA_RANDOM_VAR;
      static const int dim        =                     1;
      static const bool is_symmetric =               true;
      typedef T                                   scalr_t;
      typedef T                                   value_t;
      typedef T                                   locat_t;
      typedef std::uniform_real_distribution<T>   distr_t;
      typedef typename distr_t::param_type        param_t;
    };
  }

  /// @ingroup group_stats
  namespace rnd
  {

    /// @ingroup group_stats
    ///
    /// @brief Class for Uniform real distribution.
    ///
    /// @param T: the scalar value, result type of each sampling.
    ///
    template<typename T> class RUniform
      : public abstract::distrbase<RUniform<T> >
    {
      static_assert(std::is_floating_point_v<T>,
                    "ERROR: RUNIFORM EXPECTS FLOATING POINT ARG.");
    public:

      using exact_t =               RUniform<T>;
      using specs_t =    traits::specs<exact_t>;
      using scalr_t = typename specs_t::scalr_t;
      using value_t = typename specs_t::value_t;
      using locat_t = typename specs_t::locat_t;
      using distr_t = typename specs_t::distr_t;
      using param_t = typename specs_t::param_t;

      using parnt_t = abstract::distrbase<exact_t>;
      friend parnt_t;

      using parnt_t::draw;
      using parnt_t::distr_max;
      using parnt_t::operator();
      using parnt_t::reset_state;
      using parnt_t::reset_param;

      /// Ctor.
      ///
      /// @param lower: lower bound of the distribution (included).
      /// @param upper: upper bound of the distribution (excluded).
      ///
      RUniform(scalr_t lower, scalr_t upper);

      /// Ctor.
      ///
      /// @param lower: lower bound of the distribution (included).
      /// @param upper: upper bound of the distribution (excluded).
      /// @param seed: random seed.
      ///
      RUniform(scalr_t lower, scalr_t upper, unsigned seed);

      /// @brief Default ctor.
      RUniform() = default;

      /// @brief Default copy ctor.
      RUniform(const RUniform &rhs) = default;

      /// @brief Default Move ctor.
      RUniform(RUniform &&rhs) = default;

      /// @brief Default copy assignment operator.
      exact_t& operator=(const RUniform &rhs) = default;

      /// @brief Default Move assignment operator.
      exact_t& operator=(RUniform &&rhs) = default;

      /// Access lower bound.
      scalr_t min() const { return _distribution.min(); };

      /// Access upper bound.
      scalr_t max() const { return _distribution.max(); };

    protected:

      /// Sample a Uniform real r.v.
      value_t draw_impl() const;

      /// Return the max value of the uniform pdf.
      scalr_t max_distr_val_impl() const;

      /// Return the value at specified index of the uniform pdf.
      ///
      /// @param location: input index where the pdf will be evaluated.
      ///
      scalr_t distr_val_at_impl(locat_t location) const;

    private:
      mutable distr_t _distribution;
    };

  }
}


# include "RUniform.hxx"
#endif //!PASTA_RUNIFORM_HH
