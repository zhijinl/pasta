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
// File: Bernoulli.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:53:37 2018 Zhijin Li
// Last update Wed Nov  7 21:17:04 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_BERNOULLI_HH
# define PASTA_BERNOULLI_HH

# include "distrbase.hh"


namespace pasta
{
  // Fwd decl.
  namespace rnd { class Bernoulli; }

  /// @ingroup group_traits
  namespace traits
  {

    /// @ingroup group_traits
    ///
    /// @brief Type traits properties for the `pasta::rnd::Bernoulli`
    /// class.
    ///
    template<> struct specs<rnd::Bernoulli>
    {
      static const pst_id_list pst_id    =   pst_id_list::PASTA_RANDOM_VAR;
      static const int dim           =                       1;
      static const bool is_symmetric =                   false;
      typedef double                                   scalr_t;
      typedef std::bernoulli_distribution::result_type value_t;
      typedef std::bernoulli_distribution::result_type locat_t;
      typedef std::bernoulli_distribution              distr_t;
      typedef typename distr_t::param_type             param_t;
    };

  }

  /// @ingroup group_stats
  namespace rnd
  {

    /// @ingroup group_stats
    ///
    /// @brief Class for Bernoulli distribution.
    ///
    class Bernoulli: public abstract::distrbase<Bernoulli>
    {
    public:

      using exact_t =              Bernoulli;
      using specs_t =    traits::specs<exact_t>;
      using scalr_t = typename specs_t::scalr_t;
      using value_t = typename specs_t::value_t;
      using locat_t = typename specs_t::locat_t;
      using distr_t = typename specs_t::distr_t;
      using param_t = typename specs_t::param_t;

      using parnt_t = abstract::distrbase<exact_t>;
      friend parnt_t;

      using engine_t = typename parnt_t::engine_t;
      using seed_t = typename parnt_t::seed_t;

      using parnt_t::draw;
      using parnt_t::distr_max;
      using parnt_t::operator();
      using parnt_t::reset_seed;
      using parnt_t::reset_state;
      using parnt_t::reset_param;
      using parnt_t::reset_state_with_seed;

      /// Ctor
      ///
      /// @param pr: the success probability.
      ///
      explicit Bernoulli(scalr_t pr);

      /// Ctor with specified random seed.
      ///
      /// @param pr: the success probability.
      /// @param seed: random seed.
      ///
#ifdef PST_USE_SHARED_RND_ENGINE
      template<typename Seed>
      Bernoulli(scalr_t pr, Seed seed);
#else
      Bernoulli(scalr_t pr, seed_t seed);
#endif

      /// Default ctor.
      Bernoulli() = default;

      /// Default copy ctor.
      Bernoulli(const Bernoulli &rhs) = default;

      /// Default move ctor.
      Bernoulli(Bernoulli &&rhs) = default;

      /// Default copy assignment operator.
      exact_t& operator=(const Bernoulli &rhs) = default;

      /// Default move assignment operator.
      exact_t& operator=(Bernoulli &&rhs) = default;

      /// Access the distribution param.
      ///
      /// @return The success rate.
      ///
      scalr_t success_rate() const { return _distribution.p(); }

    private:

      /// Sample a Uniform read r.v.
      value_t draw_impl() const;

      /// Return the max value of the Bernoulli pmf.
      ///
      /// @return The maximum of current Bernoulli pmf, i.e. max of
      /// success & failure rate.
      ///
      scalr_t max_distr_val_impl() const;

      /// Return the value at specified index of the Bernoulli pmf.
      ///
      /// @param location: input index where the pmf will be evaluated.
      /// @return The Bernoulli pmf evaluated at `location`.
      ///
      scalr_t distr_val_at_impl(locat_t location) const;

      mutable distr_t _distribution;
    };

  }
}


# include "Bernoulli.hxx"
#endif //!PASTA_BERNOULLI_HH
