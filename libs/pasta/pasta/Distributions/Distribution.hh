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
// File: Distribution.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 16:00:15 2018 Zhijin Li
// Last update Sat Nov  3 20:33:57 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_DISTRIBUTION_HH
# define PASTA_DISTRIBUTION_HH

# include "pasta/core.hh"

namespace pasta
{

  // Fwd decl.
  namespace rnd
  {
    template<typename,int,typename,typename> class Distribution;
  }

  /// @ingroup group_traits
  namespace traits
  {

    /// @ingroup group_traits
    ///
    /// @brief Type traits properties for the `pasta::rnd::Distribution`
    /// class.
    ///
    template<typename ValueType, int Dim, typename Func, typename Sampler>
    struct specs<rnd::Distribution<ValueType,Dim,Func> >
    {
      static const pst_id_list pst_id    =    pst_id_list::PASTA_RANDOM_VAR;
      static const int dim           =                      Dim;
      static const bool is_symmetric =                    false;
      typedef double                                    scalr_t;
      typedef point_dispatch_t<ValueType,Dim,Func> value_t;
      typedef point_dispatch_t<ValueType,Dim,Func> locat_t;
      typedef Func                                 distr_t;
      typedef Sampler                                   smplr_t;
    };
  }

  /// @ingroup group_stats
  namespace rnd
  {

    /// @ingroup group_stats
    ///
    /// @brief User-defined multivariate distribution.
    ///
    /// A distribution is a function **sums / integrates to 1**
    ///
    /// @param ValueType: scalar type used for sample values.
    /// @param Dim: dimension of the distribution.
    /// @Func: a functor or lambda describing the distribution
    /// function: a **pdf or pmf.**
    ///
    template<typename ValueType, int Dim, typename Func>
    class Distribution:
      public abstract::distrbase<Distribution<ValueType,Dim,Func> >
    {
    public:

      using exact_t = Distribution<ValueType,Dim,Func>;
      using specs_t =                traits::specs<exact_t>;
      using scalr_t =             typename specs_t::scalr_t;
      using value_t =             typename specs_t::value_t;
      using locat_t =             typename specs_t::locat_t;
      using distr_t =             typename specs_t::distr_t;

      /// @brief Default constructor.
      Distribution() = default;

      /// @breif Ctor.
      ///
      /// Taking `lvalue` ref of the functional distribution.
      ///
      explicit Distribution(const Func &distr):
        _distribution(distr) {};

      /// @breif Ctor.
      ///
      /// Taking `rvalue` ref of the functional distribution.
      ///
      explicit Distribution(Func &&distr):
        _distribution(std::move(distr)) {};

      /// @brief Default copy ctor.
      Distribution(const Distribution &) = default;

      /// @brief Default Move ctor.
      Distribution(Distribution &&) = default;

      /// @brief Default copy assignment operator.
      exact_t& operator=(const Distribution &) = default;

      /// @brief Default Move assignment operator.
      exact_t& operator=(Distribution &&) = default;

    private:

      /// Sample a realization following current distribution.
      value_t draw_impl() const;

      /// Return the max value of the uniform pdf.
      scalr_t max_distr_val_impl() const;

      /// Return the value at specified location of the current distr.
      ///
      /// @param location: input index where the distr will be evaluated.
      /// For Dim == 1, it is a scalr value. For Dim > 1, it must be an
      /// **Eigen fixed size column vector expression**.
      ///
      template<typename Location,
               typename = std::enable_if_t<dim_dispatch_v<Location>()==Dim> >
      scalr_t distr_val_at_impl(Location &&location) const;

      distr_t _distribution; //!< The current distribution function.
      scalr_t _distr_maxima; //!< Maximum value of the distribution.
    };

  }
}
