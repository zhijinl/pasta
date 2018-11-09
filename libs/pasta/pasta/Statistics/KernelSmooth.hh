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
// File: KernelSmooth.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:29:11 2018 Zhijin Li
// Last update Fri Nov  9 23:04:13 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_KERNELSMOOTH_HH
# define PASTA_KERNELSMOOTH_HH

# include "Kernels.hh"
# include "../Domains/DiscreteDom.hh"
# include "EmpiricalStats.hh"


namespace pasta
{
  // Fwd decl.
  namespace stats { template<typename Kernel, int Dim> class KernelSmooth; }

  /// @ingroup group_traits
  namespace traits
  {
    /// @ingroup group_traits
    ///
    /// @brief Type traits properties for the
    /// `pasta::stats::KernelSmooth` class.
    ///
    template<typename Kernel, int Dim>
    struct specs<stats::KernelSmooth<Kernel,Dim> >
    {
      static constexpr int dim =                   Dim;
      typedef Kernel                           kernl_t;
      typedef  typename specs<Kernel>::value_t value_t;
      typedef  typename specs<Kernel>::bandw_t bandw_t;
    };
  }

  namespace stats
  {

    /// @ingroup group_stats
    ///
    /// @brief Kernel smooth estimator for intensity function, probability
    /// density function, cumulative distribution, and inverse cumulative
    /// distribution function.
    ///
    /// Kernel smooth technique is widely used for non-parametric estimations.
    /// It is a generalization of histogram approximation of a probability
    /// distribution. It KS, a kernel, which is itself a probability distr
    /// function, is used at each location of estimation, to take into
    /// account other (near-by) data points.
    ///
    /// KS works best with uni-modal distributions. This implementation is
    /// valid for any arbitrary dimension, but it is generally recommanded
    /// to use KS with dimension <= 3, due to the **curse of dimensionality**.
    ///
    /// `KernelSmooth` object can be constructed as a constant expression
    /// object. Some convenient helpers are defined for some specified kernel
    /// type.
    ///
    /// @param Kernel: the type of kernel. Various are avalaible.
    /// @param Dim: dimension of the data points.
    ///
    template<typename Kernel, int Dim> class KernelSmooth
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      using exact_t        =  KernelSmooth<Kernel,Dim>;
      using specs_t        =    traits::specs<exact_t>;
      using kernl_t        = typename specs_t::kernl_t;
      using value_t        = typename specs_t::value_t;
      using bandw_t        = typename specs_t::bandw_t;

      static constexpr int dim =          specs_t::dim;

      template<typename Input>
      using estim_t = EmpiricalStats<value_dispatch_t<Input>,dim,value_t,1>;

      /// @brief Default ctor.
      ///
      /// This construct an empty estimator with uninitialized kernel.
      ///
      KernelSmooth() = default;

      /// @brief Const access the internal kernel.
      ///
      /// @return Const reference to internal kernel.
      ///
      const kernl_t& kernel() const { return _kernel; };

      /// @brief Non-const access the internal kernel.
      ///
      /// @return Non-const reference to internal kernel.
      ///
      kernl_t& kernel() { return _kernel; };

      /// @brief Run a kernel smooth estimation with given tag, data,
      /// domain and band width.
      ///
      /// @param tag: tag indicating with property to estimate. Available
      /// choices are pdf, cdf, icdf and inten.
      /// @param data: the input data.
      /// - When it's a dynamic Eigen vector, it can be **either row or
      ///   col ordered**.
      /// - When it's a matrix, it**must be column ordered**, i.e. each
      ///   column represents a data point.
      /// @param domain: the domain indicating where estimates are going
      /// to be evaluated. A `pasta::dom::DiscreteDom` object.
      /// @param band_width: band width used for estimation.
      /// - A scalar when Dim == 1
      /// - An Eigen Dim x Dim matrix when Dim > 1.
      /// @return The estimates: an Eigen Dynamic x (Dim+1) matrix. The 1st
      /// col is the estimates, the rest are corresponding locations.
      ///
      template<typename Tag, typename Data, typename Dom, typename Bandw,
               typename = enable_if_all_t<is_eigen_v<Data>,
                                          is_pst_domain_v<Dom> > >
      estim_t<Data>
      estimate(Tag tag, Data &&data, Dom &&domain, Bandw &&band_width)
        const;

      /// @brief Run a kernel smooth estimation with given tag, data,
      /// domain and default band width.
      ///
      /// @note The default band_with is computed using the Silverman's
      /// rule-of-thumb.
      ///
      /// @param tag: tag indicating with property to estimate. Available
      /// choices are pdf, cdf, icdf and inten.
      /// @param data: the input data.
      /// - When it's a dynamic Eigen vector, it can be **either row or
      ///   col ordered**.
      /// - When it's a matrix, it**must be column ordered**, i.e. each
      ///   column represents a data point.
      /// @param domain: the domain indicating where estimates are going
      /// to be evaluated. A `pasta::dom::DiscreteDom` object.
      /// @return The estimates: an Eigen Dynamic x (Dim+1) matrix. The 1st
      /// col is the estimates, the rest are corresponding locations.
      ///
      /// @sa `pasta::kern::silverman_bw`.
      ///
      template<typename Tag, typename Data, typename Dom,
               enable_if_all_t<is_eigen_v<Data>,
                               is_pst_domain_v<Dom> >* = nullptr>
      estim_t<Data> estimate(Tag tag, Data &&data, Dom &&domain) const;

      /// @brief Run a kernel smooth estimation with given tag, data,
      /// band width and default domain.
      ///
      /// @note The default domain is computed empirical from the data upper
      /// and lower bound, scaled by 1.2 in all dim to ensure full coverage.
      ///
      /// @param tag: tag indicating with property to estimate. Available
      /// choices are pdf, cdf, icdf and inten.
      /// @param data: the input data.
      /// - When it's a dynamic Eigen vector, it can be **either row or
      ///   col ordered**.
      /// - When it's a matrix, it**must be column ordered**, i.e. each
      ///   column represents a data point.
      /// @param band_width: band width used for estimation.
      /// - A scalar when Dim == 1
      /// - An Eigen Dim x Dim matrix when Dim > 1.
      /// @return The estimates: an Eigen Dynamic x (Dim+1) matrix. The 1st
      /// col is the estimates, the rest are corresponding locations.
      ///
      template<typename Tag, typename Data, typename Bandw,
               enable_if_all_t<is_eigen_v<Data>,
                               !is_pst_domain_v<Bandw> >* = nullptr>
      estim_t<Data> estimate(Tag tag, Data &&data, Bandw &&band_width) const;

      /// @brief Run a kernel smooth estimation with given tag, data,
      /// default domain and band width.
      ///
      /// @param tag: tag indicating with property to estimate. Available
      /// choices are pdf, cdf, icdf and inten.
      /// @param data: the input data.
      /// - When it's a dynamic Eigen vector, it can be **either row or
      ///   col ordered**.
      /// - When it's a matrix, it**must be column ordered**, i.e. each
      ///   column represents a data point.
      /// @param domain: the domain indicating where estimates are going
      /// to be evaluated. A `pasta::dom::DiscreteDom` object.
      /// @param band_width: band width used for estimation.
      /// - A scalar when Dim == 1
      /// - An Eigen Dim x Dim matrix when Dim > 1.
      /// @return The estimates: an Eigen Dynamic x (Dim+1) matrix. The 1st
      /// col is the estimates, the rest are corresponded estim locations.
      ///
      template<typename Tag, typename Data,
               typename = std::enable_if_t<is_eigen_v<Data> > >
      estim_t<Data> estimate(Tag tag, Data &&data) const;

    private:

      /// @brief Kernel smooth estimation of the cumulative distribution
      /// function.
      ///
      /// @param data: the input data.
      /// @param domain: the domain indicating where estimates are going
      /// to be evaluated. A `pasta::dom::DiscreteDom` object.
      /// @param band_width: band width used for estimation.
      /// - A scalar when Dim == 1
      /// - An Eigen Dim x Dim matrix when Dim > 1.
      /// @param tag: tag of type cdf_tag_t.
      /// @return The estimates. An Eigen column matrix. The First column
      /// contains estimated value. The rest of the columns represent
      /// locations.
      ///
      template<typename Data, typename Dom,
               std::enable_if_t<is_pst_domain_v<Dom> >* = nullptr>
      estim_t<Data>
      estimate_impl(Data &&data, Dom &&domain, cdf_tag_t tag) const;

      /// @brief Kernel smooth estimation of the inverse cumulative
      /// distribution function.
      ///
      /// @param data: the input data.
      /// @param pos: position where the pdf will be evaluated. A scalar if
      /// Dim == 1, an Eigen Dim x 1 expression if not.
      /// @param tag: tag of type icdf_tag_t.
      /// @return The estimates. An Eigen column matrix. The First column
      /// contains estimated value. The rest of the columns represent
      /// locations.
      ///
      template<typename Data, typename Dom,
               std::enable_if_t<is_pst_domain_v<Dom> >* = nullptr>
      estim_t<Data>
      estimate_impl(Data &&data, Dom &&domain, icdf_tag_t tag) const;

      /// @brief Kernel smooth estimation of the probability distribution
      /// function.
      ///
      /// @param data: the input data.
      /// @param domain: the domain indicating where estimates are going
      /// to be evaluated. A `pasta::dom::DiscreteDom` object.
      /// @param band_width: band width used for estimation.
      /// - A scalar when Dim == 1
      /// - An Eigen Dim x Dim matrix when Dim > 1.
      /// @param tag: tag of type pdf_tag_t.
      /// @return The estimates. An Eigen column matrix. The First column
      /// contains estimated value. The rest of the columns represent
      /// locations.
      ///
      template<typename Data, typename Dom,
               std::enable_if_t<is_pst_domain_v<Dom> >* = nullptr>
      estim_t<Data>
      estimate_impl(Data &&data, Dom &&domain, pdf_tag_t tag) const;

      mutable kernl_t _kernel;
    };

    /// Helper for Gaussian kernel smooth estimator.
    template<typename Value, int Dim>
    using GaussianKS = KernelSmooth<kern::GaussKernel<Value,Dim>,Dim>;

  } //!stats
} //!pasta


# include "KernelSmooth.hxx"
#endif //!PASTA_KERNELSMOOTH_HH
