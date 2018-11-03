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
// File: Densities.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:59:05 2018 Zhijin Li
// Last update Fri Nov  2 20:17:35 2018 Zhijin Li
// ---------------------------------------------------------------------------


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
      static const pst_id_list pst_id    = pst_id_list::PASTA_RANDOM_VAR;
      static const int dim           =                       Dim;
      static const bool is_symmetric =                     false;
      typedef double                                     scalr_t;
      typedef point_dispatch_t<ValueType,Dim,Func>       value_t;
      typedef point_dispatch_t<ValueType,Dim,Func>       locat_t;
      typedef Func                                       distr_t;
      typedef Sampler                                    smplr_t;
    };
  }


  namespace rnd
  {

    template<typename ValueType, int Dim, typename Func, sampler smplr>
    class PDF:
      public abstract::distrbase<PDF<ValueType,Dim,Func,smplr> >
    {
    };

    template<typename ValueType, int Dim, typename Func, sampler smplr>
    class UnnormalizedDensity:
      public abstract::distrbase<UnnormalizedDensity<ValueType,Dim,Func,smplr> >
    {
    };

    // template<typename ValueType, int Dim, typename Func, sampler smplr>
    // class CDF:
    //   public abstract::distrbase<CDF<ValueType,Dim,Func,smplr> >
    // {
    // };

    template<typename ValueType, int Dim, typename Func, sampler smplr>
    class InverseCDF:
      public abstract::distrbase<InverseCDF<ValueType,Dim,Func,smplr> >
    {
    };

  }
}
