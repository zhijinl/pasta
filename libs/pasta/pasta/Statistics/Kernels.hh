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
// File: Kernels.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:29:55 2018 Zhijin Li
// Last update Sat Nov 10 00:19:16 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_KERNELS_HH
# define PASTA_KERNELS_HH

# include "pasta/core.hh"
# include "StatsOps.hh"
# include "../Domains/DiscreteDom.hh"


namespace pasta
{
  namespace kern
  {
    // Fwd decl.
    template<typename,int> class GaussKernel;
    template<typename,int> class EpanechKernel;

    /// @ingroup group_stats
    ///@{
    /// @brief Compute the data-driven rule of thumb bandwidth for kernel
    /// smooth estimations using Silverman's method.
    ///
    /// @note The Silverman bandwidths are empirical values that works the
    /// best when the input data is Gaussian distributed.
    ///
    /// @param Value: template argument for the return band width val type.
    /// **must be a floating-point value type**.
    /// @param data: the input data points vector or matrix. When a matrix
    /// of Eigen is given, it is assumed column majored with each column as
    /// denoting a data point. When a vector is given, it can have any order.
    /// @return The computed Rule-of-Thumb bandwidth value.
    ///
    /// @sa [Silverman]
    /// (http://www.stat.ufl.edu/~rrandles/sta6934/smhandout.pdf)
    ///
    template<typename Value, typename Data,
             std::enable_if_t<is_eigen_vec_v<Data> >* = nullptr>
    Value silverman_bw(Data &&data);

    template<typename Value, typename Data,
             std::enable_if_t<is_eigen_mat_v<Data> >* = nullptr>
    auto silverman_bw(Data &&data)
      -> Eigen::Matrix<Value,dim_dispatch_v<Data>,dim_dispatch_v<Data> >;
    ///@}

  }

  /// @ingroup group_traits
  namespace traits
  {
    /// @ingroup group_traits
    ///
    /// @brief Type traits for `pasta::kern::GaussKernel` class.
    ///
    template<typename Value, int Dim>
    struct specs<kern::GaussKernel<Value,Dim> >
    {
      static const pst_id_list pst_id = pst_id_list::PASTA_KERNEL;
      static const int dim =                                 Dim;
      typedef Value                                      value_t;
      typedef bandwidth_dispatch_t<Value,Dim>            bandw_t;
    };

    /// @ingroup group_traits
    ///
    /// @brief Type traits for `pasta::kern::EpanechKernel` class.
    ///
    template<typename Value, int Dim>
    struct specs<kern::EpanechKernel<Value,Dim> >
    {
      static const pst_id_list pst_id = pst_id_list::PASTA_KERNEL;
      static const int dim =                                 Dim;
      typedef Value                                      value_t;
      typedef bandwidth_dispatch_t<Value,Dim>            bandw_t;
    };

  }

  namespace kern
  {
    ///@{
    /// @ingroup group_stats
    ///
    /// @brief N-dimensional Gaussian kernel.
    ///
    /// Class containing constrcution & evaluation methods for Gaussian kernel
    /// of arbitrary dimension. The analytical formula for a @f$ d @f$
    /// -dimensional Gaussian kernel is:
    ///
    ///   @f[ \frac{1}{\sqrt{2\pi}^d |\Sigma|^{-\frac{1}{2}}}
    ///       exp( -\frac{1}{2} (x-\mu)^T \Sigma^{-1} (x-\mu) ) @f]
    ///
    /// @note A note on the bandwidth:
    /// * In 1D, the bandwidth is defined to play the role of the Gaussian
    ///   **standard deviation**, i.e. the @f$ \sigma @f$ in:
    ///
    ///   @f[ \frac{1}{\sqrt{2\pi}{\sigma}}
    ///       exp( - \frac{(x-\mu)^2}{\sigma^2} ) @f]
    ///
    /// * In multi-dimensional cases, due to the multi-normal definition
    ///   convetion, the so-called **bandwidth matrix** often refers to the
    ///   **covariance matrix** of the Gaussian. i.e. the @f$ \Sigma @f$ in:
    ///
    ///   @f[ \frac{1}{\sqrt{2\pi}^d |\Sigma|^{-\frac{1}{2}}}
    ///       exp( -\frac{1}{2} (x-\mu)^T \Sigma^{-1} (x-\mu) ) @f]
    ///
    /// @note Kernel computation is implemented using a **cut-off** value, for
    /// the sake of efficiency. A parameter `eps` is available for the ctor
    /// and the `reset` method, specifying the limit of the kernel value of a
    /// point, below which the point is **no longer used for the smoothing**.
    ///
    /// @param Value: type of the value used for kernel value computed at one
    /// position. **Use floating point** for precision & to avoid unexpected
    /// results.
    /// @param Dim: dimension of the kernel.
    ///
    template<typename Value, int Dim> class GaussKernel
    {
    public:

      using exact_t =    GaussKernel<Value,Dim>;
      using specs_t =    traits::specs<exact_t>;
      using value_t = typename specs_t::value_t;
      using bandw_t = typename specs_t::bandw_t;

      static constexpr int dim = specs_t::dim;

      /// @brief Default ctor.
      GaussKernel() = default;

      /// @brief Ctor.
      ///
      /// Initialize using bandwidth and precision.
      ///
      /// @note The precision variable controlls the level of exactness of
      /// bandwidth value computation. Each precision corresponds to a cut-
      /// off value, determining the cut-off distance from which a point will
      /// not be considered for smoothing.
      ///
      /// @param bw_mat: input bandwidth.
      /// @param eps: the desired precision, default to 1e-8.
      ///
      template<typename Bandw, std::enable_if_t<is_eigen_v<Bandw> >* = nullptr>
      GaussKernel(Bandw &&bw_mat, value_t eps=1e-8);

      /// @brief Compute Kernel smoothed sum from input data points at a location.
      ///
      /// Comp smoothed value at pos with band width bw: i.e. the sum of smoothed
      /// values, normalized by the bandwidth.
      ///
      /// @warning The result is not normalized by number of samples in data vec.
      /// This is because normalization is not always desire in some smoothing
      /// application(s): f.e. intensity estimation for point patterns. When used
      /// with probability density estimation, user need to manually apply
      /// normalization by number of samples. The KernelSmooth class does that
      /// internally.
      ///
      /// @param vec: the input Eigen vector containing 1D data pts. If it can be
      /// a dynamic row or col ordered vector. Or a **row ordered fix sized vec**.
      /// @param pos: the position where kernel smoothing will be computed. In
      /// this 1D case it is arithmetic.
      /// @param bw: the scalar band width use for smoothing.
      /// @return The computed smoothed value at position pos.
      ///
      template<typename Matrix, typename Point,
               enable_if_all_t<is_eigen_mat_v<Matrix>,
                               is_eigen_v<Point> >* = nullptr>
      value_t operator()(Matrix &&data, Point &&pos) const;

      /// @brief Compute Kernel smoothed sum from input data points at a location,
      /// with weights.
      ///
      /// Comp smoothed value at pos with band width bw: i.e. the sum of smoothed
      /// values, normalized by the bandwidth. Each input data points is weighted
      /// by a factor specified by the weight vector.
      ///
      /// @warning The result is not normalized by number of samples in data vec.
      /// This is because normalization is not always desire in some smoothing
      /// application(s): f.e. intensity estimation for point patterns. When used
      /// with probability density estimation, user need to manually apply
      /// normalization by number of samples. The KernelSmooth class does that
      /// internally.
      ///
      /// @param data: the input Eigen vector or matrix (colmajor) containing data
      /// pts. When it's a vector it **can be of any ordering**. When it's a matrix,
      /// **it is assumed colume ordered**.
      /// @param weights: an Eigen vector of weights.
      /// @param pos: the position where kernel smoothing will be computed.
      /// @param bw: the band width.
      /// @return The computed smoothed value at position pos.
      ///
      template<typename Matrix, typename Weights, typename Point,
               enable_if_all_t<is_eigen_mat_v<Matrix>,
                               is_eigen_vec_v<Weights>,
                               is_eigen_v<Point> >* = nullptr>
      value_t operator()(Matrix &&data, Weights &&weights, Point &&pos) const;

      /// @brief reset the bandwidth value.
      ///
      /// @param bandwidth: the new bandwidth.
      /// @param eps: the desired precision, default to 1e-8.
      ///
      template<typename Bandw> void reset(Bandw &&bandwidth, value_t eps=1e-8);

      /// @brief Evaluate cutoff value.
      ///
      /// @param bandwidth: the input bandwidth.
      /// @param eps: the desired precision.
      /// @return The cutoff value corresponding to input precision.
      ///
      template<typename Bandw>
      value_t eval_cutoff(Bandw &&bandwidth, value_t eps) const;

    private:

      /// @brief An internal function evaluated unormalized kernel value at a
      /// position.
      ///
      /// @param pos: the input position, scalar when Dim == 1, an Eigen Dim x
      /// Dim square matrix when Dim > 1.
      /// @return Unormalized value of the kernel evaluated at position `pos`.
      ///
      template<typename Point, enable_if_all_t<is_eigen_v<Point> >* = nullptr>
      value_t operator()(Point pos) const;

      value_t _inv_sqrtdet; //!< Inverse of sqrt of bandwidth determinant.
      bandw_t _inv_sqrtbdw; //!< Inverse of mat sqrt of bandwidth.
      value_t _cut_off_val; //!< Cutoff value to speedup computation.
    };

    // Specialization for 1D.
    template<typename Value> class GaussKernel<Value,1>
    {
    public:

      using exact_t =      GaussKernel<Value,1>;
      using specs_t =    traits::specs<exact_t>;
      using value_t = typename specs_t::value_t;
      using bandw_t = typename specs_t::bandw_t;

      static constexpr int dim = specs_t::dim;

      GaussKernel() = default;

      GaussKernel(value_t bandwidth, value_t eps=1e-8):
        _inv_bw( 1.0/bandwidth ), _cutoff( eval_cutoff(bandwidth, eps) ) {};

      template<typename Vec>
      value_t operator()(Vec &&data, value_t pos) const;

      template<typename Vec, typename Weights>
      value_t operator()(Vec &&data, Weights &&weights, value_t pos) const;

      void reset(value_t bandwidth, value_t eps=1e-8);

      value_t eval_cutoff(value_t bandwidth, value_t eps) const;

    private:

      value_t operator()(value_t pos) const;

      bandw_t _inv_bw;
      value_t _cutoff;
    };
    ///@}


    /// @ingroup group_stats
    ///
    /// @brief N-dimensional Epanechnikov kernel.
    ///
    /// Class containing constrcution & evaluation methods for Epanechnikov kernel
    /// of arbitrary dimension.
    ///
    /// @param Value: type of the value used for kernel value computed at one
    /// position. **Use floating point** for precision & to avoid unexpected
    /// results.
    ///
    template<typename Value, int Dim> class EpanechKernel
    {
    public:

      using value_t = typename traits::specs<EpanechKernel<Value,Dim> >::value_t;

      /// Ctor.
      constexpr EpanechKernel() {};

      /// @brief Simply evaluate the value of the kernel function at given
      /// position
      ///
      /// @param pos: the input position, arithmetic.
      /// @return Value of the kernel evaluated at position pos.
      ///
      template<typename Point, enable_if_all_t<is_eigen_v<Point> >* = nullptr>
      value_t operator()(Point pos) const;

      /// @brief Compute Kernel smoothed sum from input data points at a location.
      ///
      /// Comp smoothed sum at pos with band width bw: i.e. the sum of smoothed
      /// values, normalized by the bandwidth.
      ///
      /// @warning The result is not normalized by number of samples in data vec.
      /// This is because normalization is not always desire in some smoothing
      /// application(s): f.e. intensity estimation for point patterns. When used
      /// with probability density estimation, user need to manually apply
      /// normalization by number of samples. The KerSmooth class does that
      /// internally.
      ///
      /// @param data: the input Eigen vector or matrix (colmajor) containing data
      /// pts. When it's a vector it **can be of any ordering**. When it's a matrix,
      /// **it is assumed colume ordered**.
      /// @param pos: the position where kernel smoothing will be computed. In
      /// this 1D case it is arithmetic.
      /// @param bw: the scalar band width use for smoothing.
      /// @return The computed smoothed sum at position pos.
      ///
      template<typename Matrix, typename Point,
               enable_if_all_t<is_eigen_mat_v<Matrix>,
                               is_eigen_v<Point> >* = nullptr>
      value_t operator()(Matrix &&data, Point &&pos) const;

      /// @brief Compute Kernel smoothed sum from input data points at a location,
      /// with weights.
      ///
      /// Comp smoothed value at pos with band width bw: i.e. the sum of smoothed
      /// values, normalized by the bandwidth. Each input data points is weighted
      /// by a factor specified by the weight vector.
      ///
      /// @warning The result is not normalized by number of samples in data vec.
      /// This is because normalization is not always desire in some smoothing
      /// application(s): f.e. intensity estimation for point patterns. When used
      /// with probability density estimation, user need to manually apply
      /// normalization by number of samples. The KernelSmooth class does that
      /// internally.
      ///
      /// @param data: the input Eigen vector or matrix (colmajor) containing data
      /// pts. When it's a vector it **can be of any ordering**. When it's a matrix,
      /// **it is assumed colume ordered**.
      /// @param weights: an Eigen vector of weights.
      /// @param pos: the position where kernel smoothing will be computed.
      /// @param bw: the band width.
      /// @return The computed smoothed value at position pos.
      ///
      template<typename Matrix, typename Weights, typename Point,
               enable_if_all_t<is_eigen_mat_v<Matrix>,
                               is_eigen_vec_v<Weights>,
                               is_eigen_v<Point> >* = nullptr>
      value_t operator()(Matrix &&data, Weights &&weights,
                         Point &&pos) const;

    };

  } //!kern
} //!pasta


# include "Kernels.hxx"
#endif //!PASTA_KERNELS_HH
