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
// File: Poisson.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:55:36 2018 Zhijin Li
// Last update Sat Nov  3 21:01:15 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_POISSON_HH
# define PASTA_POISSON_HH

# include "distrbase.hh"


namespace pasta
{
  // Fwd decl.
  namespace rnd { template<typename T> class Poisson; }

  /// @ingroup group_traits
  namespace traits
  {

    /// @ingroup group_traits
    ///
    /// @brief Type traits properties for the `pasta::rnd::Poisson`
    /// class.
    ///
    template<typename T> struct specs<rnd::Poisson<T> >
    {
      static const pst_id_list pst_id    = pst_id_list::PASTA_RANDOM_VAR;
      static const int dim           =                     1;
      static const bool is_symmetric =                 false;
      typedef T                                      value_t;
      typedef double                                 scalr_t;
      typedef T                                      locat_t;
      typedef std::poisson_distribution<T>           distr_t;
      typedef typename distr_t::param_type           param_t;
    };
  }

  namespace rnd
  {

    /// @ingroup group_stats
    ///
    /// @brief Class for Poisson distribution.
    ///
    /// @param T: the scalar value, result type of each sampling.
    /// In this case must be an integral type.
    ///
    template<typename T> class Poisson
      : public abstract::distrbase<Poisson<T> >
    {
      static_assert(std::is_integral<T>::value,
                    "ERROR: POISSON DISTR RETURN INTEGRAL TYPES.");
    public:

      using exact_t =             Poisson<T>;
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
      template<typename PT,
               std::enable_if_t<std::is_arithmetic_v<PT> >* = nullptr>
      explicit Poisson(PT mean);

      /// Default ctor.
      Poisson() = default;

      /// Default copy ctor.
      Poisson(const Poisson &rhs) = default;

      /// Default move ctor.
      Poisson(Poisson &&rhs) = default;

      /// Default copy assignment operator.
      exact_t& operator=(const Poisson &rhs) = default;

      /// Default move assignment operator.
      exact_t& operator=(Poisson &&rhs) = default;

      /// Access mean / variance.
      ///
      /// @return The mean value for current Poisson distribution.
      ///
      scalr_t mean() const { return _distribution.mean(); }

    private:

      /// Sample a Poisson r.v.
      ///
      /// @return The sampled value.
      ///
      value_t draw_impl() const;

      /// Return the max value of the Poisson pmf.
      ///
      /// @return The maximum of Poisson pmf, i.e. value at
      /// `floor(mean)`.
      ///
      scalr_t max_distr_val_impl() const;

      /// Return the value at specified index of the Poisson pmf.
      ///
      /// @param location: input index where the pmf will be evaluated.
      /// @return The value of Poisson pmf at `location`.
      ///
      scalr_t distr_val_at_impl(locat_t location) const;

      mutable distr_t _distribution;
    };

  }
}


# include "Poisson.hxx"
#endif //!PASTA_POISSON_HH
