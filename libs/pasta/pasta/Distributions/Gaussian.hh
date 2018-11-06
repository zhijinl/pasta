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
// File: Gaussian.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:52:54 2018 Zhijin Li
// Last update Tue Nov  6 23:27:13 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_GAUSSIAN_HH
# define PASTA_GAUSSIAN_HH

# include "distrbase.hh"


namespace pasta
{
  // Fwd decl.
  namespace rnd { template<typename T> class Gaussian; }

  /// @ingroup group_traits
  namespace traits
  {

    /// @ingroup group_stats
    ///
    /// @brief Type traits properties for the `pasta::rnd::Gaussian`
    /// class.
    ///
    template<typename T> struct specs<rnd::Gaussian<T> >
    {
      static const pst_id_list pst_id    = pst_id_list::PASTA_RANDOM_VAR;
      static const int dim           =                     1;
      static const bool is_symmetric =                  true;
      typedef T                                      scalr_t;
      typedef T                                      value_t;
      typedef T                                      locat_t;
      typedef std::normal_distribution<T>            distr_t;
      typedef typename distr_t::param_type           param_t;
    };
  }

  /// @ingroup group_stats
  namespace rnd
  {

    /// @ingroup group_stats
    ///
    /// @brief Class for Gaussian distribution.
    ///
    /// @param T: the scalar value, result type of each sampling.
    ///
    template<typename T> class Gaussian
      : public abstract::distrbase<Gaussian<T> >
    {
    public:

      using exact_t =            Gaussian<T>;
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
      /// @param mu: mean of the Gaussian distribution.
      /// @param sigma: stddev of Gaussian distribution.
      ///
      Gaussian(scalr_t mu, scalr_t sigma);

      /// Ctor with specified random seed.
      ///
      /// @param mu: mean of the Gaussian distribution.
      /// @param sigma: stddev of Gaussian distribution.
      /// @param seed: random seed.
      ///
      Gaussian(scalr_t mu, scalr_t sigma, unsigned seed);

      /// Dflt ctor.
      Gaussian() = default;

      /// Dflt copy ctor.
      Gaussian(const Gaussian &rhs) = default;

      /// Dflt Move ctor.
      Gaussian(Gaussian &&rhs) = default;

      /// Dflt copy assignment operator.
      exact_t& operator=(const Gaussian &rhs) = default;

      /// Dflt Move assignment operator.
      exact_t& operator=(Gaussian &&rhs) = default;

      /// Access the mean.
      scalr_t mu() const { return _distribution.mean(); }

      /// Access the sigma: standard deviation.
      scalr_t sigma() const { return _distribution.stddev(); }

    private:

      /// Sample a Gaussian r.v.
      value_t draw_impl() const;

      /// Return the max value of the Gaussian pdf.
      scalr_t max_distr_val_impl() const;

      /// Return the value at specified index of the Gaussian pdf.
      ///
      /// @param location: input index where the pdf will be evaluated.
      ///
      scalr_t distr_val_at_impl(locat_t location) const;

      mutable distr_t _distribution;
    };

  }
}


# include "Gaussian.hxx"
#endif //!PASTA_GAUSSIAN_HH
