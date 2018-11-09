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
// File: StatsOps.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:30:25 2018 Zhijin Li
// Last update Fri Nov  9 23:18:51 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_STATSOPS_HH
# define PASTA_STATSOPS_HH

# include "pasta/core.hh"
# include "EmpiricalStats.hh"
# include "../Utilities/computations.hh"


namespace pasta
{
  /// @ingroup group_stats
  namespace stats
  {

    ///@{
    /// @ingroup group_stats
    ///
    /// @brief Comp mean of an STL vec or Eigen structure.
    ///
    /// @param vec: the input STL vector or Eigen structure.
    /// @return The computed mean value or column vector.
    ///
    template<typename Scalar, typename VT,
             std::enable_if_t<!is_eigen_v<VT> >* = nullptr>
    inline Scalar mean(VT &&);

    template<typename Scalar, typename VT,
             enable_if_all_t<is_eigen_v<VT>,
                             dim_dispatch_v<VT> == 1>* = nullptr>
    inline Scalar mean(VT &&);

    template<typename Scalar, typename MT,
             enable_if_all_t<is_eigen_v<MT>,
                             dim_dispatch_v<MT> != 1>* = nullptr>
    inline auto mean(MT &&) -> Eigen::Matrix<Scalar,dim_dispatch_v<MT>,1>;
    ///@}

    ///@{
    /// @ingroup group_stats
    ///
    /// @brief Comp (co)variance of an Eigen structure or STL vector.
    ///
    /// If the input is an STL containers, it is treated as a coumn
    /// vector and a scalar variance value is computed. Same as when
    /// the input is an Eigen vector. When an Eigen matrix is given
    /// as input, the covariance matrix is estimated. In this case,
    /// the input matrix is assumed to be a storage of observation
    /// vectors in columns.
    ///
    /// @note The **Bessel correction** is applied by default. This
    /// means that the division factor is `n-1` instead of `n`, where
    /// n is the sample size. The correction can be deactivated by
    /// specifying a second `uncorrected` argument to the function.
    /// When the correction is deactivated, it is recommanded to use
    /// the third overload, which takes a second argument as the true
    /// mean value / vector, and use it to compute the variance /
    /// covariance.
    ///
    /// @param data: the input Eigen structure or STL vector.
    /// @return Computed variance value or covariance matrix.
    ///
    template<typename Scalar, typename VT,
             std::enable_if_t<!is_eigen_v<VT> >* = nullptr>
    inline Scalar var(VT &&);

    template<typename Scalar, typename VT,
             std::enable_if_t<!is_eigen_v<VT> >* = nullptr>
    inline Scalar var(VT &&, uncorrected_t corr);

    template<typename Scalar, typename VT,
             std::enable_if_t<!is_eigen_v<VT> >* = nullptr>
    inline Scalar var(VT &&, Scalar true_mean);


    template<typename Scalar, typename VT,
             enable_if_all_t<is_eigen_v<VT>,
                             dim_dispatch_v<VT> == 1>* = nullptr>
    inline Scalar var(VT &&);

    template<typename Scalar, typename VT,
             enable_if_all_t<is_eigen_v<VT>,
                             dim_dispatch_v<VT> == 1>* = nullptr>
    inline Scalar var(VT &&, uncorrected_t corr);

    template<typename Scalar, typename VT,
             enable_if_all_t<is_eigen_v<VT>,
                             dim_dispatch_v<VT> == 1>* = nullptr>
    inline Scalar var(VT &&, Scalar true_mean);


    template<typename Scalar, typename MT,
             enable_if_all_t<is_eigen_v<MT>,
                             dim_dispatch_v<MT> != 1>* = nullptr>
    inline auto var(MT &&)
      -> Eigen::Matrix<Scalar, dim_dispatch_v<MT>, dim_dispatch_v<MT> >;

    template<typename Scalar, typename MT,
             enable_if_all_t<is_eigen_v<MT>,
                             dim_dispatch_v<MT> !=1 >* = nullptr>
    inline auto var(MT &&, uncorrected_t corr)
      -> Eigen::Matrix<Scalar, dim_dispatch_v<MT>, dim_dispatch_v<MT> >;

    template<typename Scalar, typename MT, typename VT,
             enable_if_all_t<is_eigen_v<MT>,
                             is_eigen_v<VT>,
                             dim_dispatch_v<VT> == 1,
                             dim_dispatch_v<MT> != 1>* = nullptr>
    inline auto var(MT &&, VT &&true_mean)
      -> Eigen::Matrix<Scalar, dim_dispatch_v<MT>, dim_dispatch_v<MT> >;
    ///@}

    /// @ingroup group_stats
    ///
    /// @brief Compute histogram from an eigen vector.
    ///
    /// @param vec: the input Eigen vector. Must be colmajored.
    /// @param bin_width: bin width.
    /// @param bord_ext: # of bins to be extended to each side of histogram.
    /// @return: the computed histogram of type `EmpiricalStats`.
    ///
    template<typename Inp, typename = std::enable_if_t<is_eigen_v<Inp> > >
    inline auto comp_histogram(Inp &&input, float bin_width, int bord_ext=0)
      -> stats::EmpiricalStats<eigen_val_t<Inp>,dim_dispatch_v<Inp>,int,1>;

  }
}


# include "StatsOps.hxx"
#endif
