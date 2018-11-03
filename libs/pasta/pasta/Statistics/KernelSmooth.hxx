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
// File: KernelSmooth.hxx for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:29:37 2018 Zhijin Li
// Last update Sat Nov  3 22:33:03 2018 Zhijin Li
// ---------------------------------------------------------------------------


namespace pasta
{
  namespace stats
  {

    // =====================================================================
    template<typename Kernel, int Dim>
    template<typename Tag, typename Data, typename Dom, typename Bandw,
             typename>
    auto KernelSmooth<Kernel,Dim>::
    estimate(Tag tag, Data &&data, Dom &&domain, Bandw &&band_width)
      const -> estim_t<Data>
    {
      _kernel.reset(std::forward<Bandw>(band_width));
      return estimate_impl(std::forward<Data>(data),
                           std::forward<Dom>(domain), tag);
    }

    // =====================================================================
    template<typename Kernel, int Dim>
    template<typename Tag, typename Data, typename Dom,
             enable_if_all_t<is_eigen_v<Data>,is_pst_domain_v<Dom>()>*>
    auto KernelSmooth<Kernel,Dim>::
    estimate(Tag tag, Data &&data, Dom &&domain) const -> estim_t<Data>
    {
      _kernel.reset(kern::silverman_bw<value_t>(data));
      return estimate_impl(data, std::forward<Dom>(domain), tag);
    }

    // =====================================================================
    template<typename Kernel, int Dim>
    template<typename Tag, typename Data, typename Bandw,
             enable_if_all_t<is_eigen_v<Data>, !is_pst_domain_v<Bandw>()>*>
    auto KernelSmooth<Kernel,Dim>::
    estimate(Tag tag, Data &&data, Bandw &&band_width)
      const -> estim_t<Data>
    {
      _kernel.reset(std::forward<Bandw>(band_width));
      return estimate_impl
        (data, utils::make_discrete_domain<value_t>
         (data, utils::make_const_pt<value_t,Dim>(0.1), 1.2), tag);
    }

    // =====================================================================
    template<typename Kernel, int Dim>
    template<typename Tag, typename Data, typename>
    auto KernelSmooth<Kernel,Dim>::estimate(Tag tag, Data &&data) const
      -> estim_t<Data>
    {
      _kernel.reset(kern::silverman_bw<value_t>(data));
      return estimate_impl
        (data, utils::make_discrete_domain<value_t>
         (data, utils::make_const_pt<value_t,Dim>(0.1), 1.2), tag);
    }

    // =====================================================================
    template<typename Kernel, int Dim>
    template<typename Data, typename Dom,
             std::enable_if_t<is_pst_domain_v<Dom>()>*>
    auto KernelSmooth<Kernel,Dim>::
    estimate_impl(Data &&data, Dom &&domain, cdf_tag_t)
      const -> estim_t<Data>
    {
      auto __cdf = estimate_impl(data, std::forward<Dom>(domain), pdf);
      __cdf.stats() = utils::cumsum(__cdf.stats())*
        domain.elem_volume();
      return __cdf;
    }

    // =====================================================================
    template<typename Kernel, int Dim>
    template<typename Data, typename Dom,
             std::enable_if_t<is_pst_domain_v<Dom>()>*>
    auto KernelSmooth<Kernel,Dim>::
    estimate_impl(Data &&data, Dom &&domain, icdf_tag_t)
      const -> estim_t<Data>
    {
      assert(false && "this function has not been implemented yet ...");
    }

    // =====================================================================
    template<typename Kernel, int Dim>
    template<typename Data, typename Dom,
             std::enable_if_t<is_pst_domain_v<Dom>()>*>
    auto KernelSmooth<Kernel,Dim>::
    estimate_impl(Data &&data, Dom &&domain, pdf_tag_t)
      const -> estim_t<Data>
    {
      using __locat_t = typename traits::specs<std::decay_t<Dom> >::locat_t;
      estim_t<Data> __result(domain.card());

      int __count = 0;
      domain.traverse
        ([this, &data, &__count, &__result] (const __locat_t &__loc)
         {
           __result.point_at(__count) = __loc;
           __result.value_at(__count) = _kernel(data, __loc);
           ++__count;
         });
      __result.stats() /= utils::n_elem(data);
      return __result;
    }

  } //!stats
} //!pasta
