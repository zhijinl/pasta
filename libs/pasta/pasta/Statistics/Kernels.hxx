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
// File: Kernels.hxx for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:30:08 2018 Zhijin Li
// Last update Sat Nov  3 22:32:53 2018 Zhijin Li
// ---------------------------------------------------------------------------


namespace pasta
{
  namespace kern
  {

    // =====================================================================
    template<typename Value, typename Data,
             std::enable_if_t<is_eigen_vec_v<Data>()>*>
    Value silverman_bw(Data &&data)
    {
      return 1.06 * std::sqrt(stats::var<Value>(data)) *
        std::pow(data.size(),-0.2);
    }

    // =====================================================================
    template<typename Value, typename Data,
             std::enable_if_t<is_eigen_mat_v<Data>()>*>
    auto silverman_bw(Data &&data)
      -> Eigen::Matrix<Value,dim_dispatch_v<Data>(),dim_dispatch_v<Data>()>
    {
      constexpr int __dim = dim_dispatch_v<Data>();
      using __matrx_t = Eigen::Matrix<Value,__dim,__dim>;
      Value __coeff = std::pow(4.0/(__dim+2)/data.cols(),2.0/(__dim+4));

      return __matrx_t{ Eigen::LDLT<__matrx_t>
          ( stats::var<Value>(data) ).vectorD().asDiagonal() * __coeff };
    }

    // =====================================================================
    template<typename Value, int Dim>
    template<typename Bandw, std::enable_if_t<is_eigen_v<Bandw> >*>
    GaussKernel<Value,Dim>::GaussKernel(Bandw &&bw_mat, value_t eps):
      _inv_sqrtbdw( utils::sqrt_mat(bw_mat) ),
      _inv_sqrtdet( std::pow(bw_mat.determinant(),-0.5) ),
      _cut_off_val( eval_cutoff(bw_mat, eps) ) {}

    // =====================================================================
    template<typename Value, int Dim>
    template<typename Matrix, typename Point,
             enable_if_all_t<is_eigen_mat_v<Matrix>(), is_eigen_v<Point> >*>
    auto GaussKernel<Value,Dim>::operator()(Matrix &&data, Point &&pos) const
      -> value_t
    {
      static_assert( dim_dispatch_v<Matrix>() == dim_dispatch_v<Point>(),
                     "POINT AND DATA VEC DIMENSION MISMATCH." );

      Value __result = 0.0;
#pragma omp parallel for reduction (+:__result)
      for(int __n = 0; __n < data.cols(); ++__n)
      {
        Eigen::Matrix<Value,Dim,1> __curr = _inv_sqrtbdw*(data.col(__n)-pos);
        if( __curr.squaredNorm() < _cut_off_val )
          __result += (*this)(__curr);
      }
      return __result*one_over_sqrt2pi_pow<Dim>()*_inv_sqrtdet;
    }

    // =====================================================================
    template<typename Value, int Dim>
    template<typename Matrix, typename Weights, typename Point,
             enable_if_all_t<is_eigen_mat_v<Matrix>(),
                             is_eigen_vec_v<Weights>(),
                             is_eigen_v<Point> >*>
    auto GaussKernel<Value,Dim>::operator()(Matrix &&data,
                                            Weights &&weights,
                                            Point &&pos) const -> value_t
    {
      static_assert( dim_dispatch_v<Matrix>() == dim_dispatch_v<Point>(),
                     "POINT AND DATA VEC DIMENSION MISMATCH." );
      // TODO: NEED TO PROPERLY IMPLEMENT WEIGHTING SCHEME.
      Value __result = 0.0;
#pragma omp parallel for reduction (+:__result)
      for(int __n = 0; __n < data.cols(); ++__n)
      {
        Eigen::Matrix<Value,Dim,1> __curr = _inv_sqrtbdw*(data.col(__n)-pos);
        if( __curr.squaredNorm() > _cut_off_val )
          __result += (*this)(__curr)*weights(__n);
      }
      return __result*one_over_sqrt2pi_pow<Dim>()*_inv_sqrtdet;
    }

    // =====================================================================
    template<typename Value, int Dim> template<typename Bandw>
    void GaussKernel<Value,Dim>::reset(Bandw &&bandwidth, value_t eps)
    {
      _inv_sqrtbdw = utils::sqrt_mat(bandwidth);
      _inv_sqrtdet = std::pow(bandwidth.determinant(),-0.5);
      _cut_off_val = eval_cutoff(bandwidth, eps);
    }

    // =====================================================================
    template<typename Value, int Dim> template<typename Bandw>
    auto GaussKernel<Value,Dim>::eval_cutoff(Bandw &&bandwidth, value_t eps)
      const -> value_t
    {
      return -2*std::log(eps*sqrt2pi_pow<Dim>()*std::sqrt
                         (bandwidth.determinant()));
    }

    // =====================================================================
    template<typename Value, int Dim>
    template<typename Point, enable_if_all_t<is_eigen_v<Point> >*>
    auto GaussKernel<Value,Dim>::operator()(Point pos) const -> value_t
    {
      return std::exp(-0.5 * pos.squaredNorm());
    }

    // =====================================================================
    template<typename Value> template<typename Vec>
    auto GaussKernel<Value,1>::operator()(Vec &&data, value_t pos) const
      -> value_t
    {
      static_assert( (dim_dispatch_v<Vec>()==1 ), "EXPECT A VECTOR." );

      Value __result = 0.0;
#pragma omp parallel for reduction (+:__result)
      for(int __n = 0; __n < data.size(); ++__n)
        if( std::abs(data(__n)-pos) < _cutoff )
          __result += (*this)((data(__n)-pos)*_inv_bw);

      return __result*one_over_sqrt2pi*_inv_bw;
    }

    // =====================================================================
    template<typename Value>  template<typename Vec, typename Weights>
    auto GaussKernel<Value,1>::operator()(Vec &&data,
                                          Weights &&weights,
                                          value_t pos) const -> value_t
    {
      static_assert( (dim_dispatch_v<Vec>()==1 ), "EXPECT A VECTOR." );
      // TODO: NEED TO PROPERLY IMPLEMENT WEIGHTING SCHEME.
      // TODO: CHECK OUT THE ELEMINATING BRANCH PREDICTION OPTMIZATION!!
      Value __result = 0.0;
#pragma omp parallel for reduction (+:__result)
      for(int __n = 0; __n < data.size(); ++__n)
        if( std::abs(data(__n)-pos) < _cutoff )
          __result += (*this)((data(__n)-pos)*_inv_bw)*weights(__n);

      return __result*one_over_sqrt2pi*_inv_bw;
    }

    // =====================================================================
    template<typename Value>
    auto GaussKernel<Value,1>::eval_cutoff(value_t bandwidth, value_t eps)
      const -> value_t
    {
      return std::sqrt(-2*std::log(eps*sqrt2pi*bandwidth))*bandwidth;
    }

    // =====================================================================
    template<typename Value>
    void GaussKernel<Value,1>::reset(value_t bandwidth, value_t eps)
    {
      _inv_bw = 1.0/bandwidth;
      _cutoff = eval_cutoff(bandwidth, eps);
    }

    // =====================================================================
    template<typename Value>
    auto GaussKernel<Value,1>::operator()(value_t pos) const -> value_t
    {
      return std::exp(-0.5 * pos * pos);
    }

  } //!kern
} //!pasta
