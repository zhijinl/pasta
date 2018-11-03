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
// File: StatsOps.hxx for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:30:36 2018 Zhijin Li
// Last update Sat Nov  3 22:34:40 2018 Zhijin Li
// ---------------------------------------------------------------------------


namespace pasta
{
  namespace stats
  {

    // =====================================================================
    template<typename Scalar, typename VT,
             std::enable_if_t<!is_eigen_v<VT> >*>
    inline Scalar mean(VT &&vec)
    {
      return utils::ref_as_eigen(vec).mean();
    }

    // =====================================================================
    template<typename Scalar, typename VT,
             enable_if_all_t<is_eigen_v<VT>, dim_dispatch_v<VT>()==1>*>
    inline Scalar mean(VT &&vec)
    {
      return vec.mean();
    }

    // =====================================================================
    template<typename Scalar, typename MT,
             enable_if_all_t<is_eigen_v<MT>, dim_dispatch_v<MT>()!=1>*>
    inline auto mean(MT &&mat)
      -> Eigen::Matrix<Scalar,dim_dispatch_v<MT>(),1>
    {
      return mat.rowwise().mean().template cast<Scalar>();
    }

    // =====================================================================
    template<typename Scalar, typename VT,
             std::enable_if_t<!is_eigen_v<VT> >*>
    inline Scalar var(VT &&vec)
    {
      auto __vec_eigen = utils::ref_as_eigen(vec);
      // auto __factor = 1.0/static_cast<Scalar>(__vec_eigen.size()-1);

      return (__vec_eigen.array() - __vec_eigen.mean()).square().sum()/
        static_cast<Scalar>(__vec_eigen.size()-1);

      // return (__vec_eigen.array()*__vec_eigen.array()).sum()*__factor -
      //   __vec_eigen.mean()* __vec_eigen.mean()*__vec_eigen.size()*__factor;
    }

    // =====================================================================
    template<typename Scalar, typename VT,
             std::enable_if_t<!is_eigen_v<VT> >*>
    inline Scalar var(VT &&vec, uncorrected_t)
    {
      auto __vec_eigen = utils::ref_as_eigen(vec);
      // return (__vec_eigen.array()*__vec_eigen.array()).sum()/
      //   static_cast<Scalar>(__vec_eigen.size()) - __vec_eigen.mean()*
      //   __vec_eigen.mean();
      return (__vec_eigen.array() - __vec_eigen.mean()).square().sum()/
        static_cast<Scalar>(__vec_eigen.size());
    }

    // =====================================================================
    template<typename Scalar, typename VT,
             std::enable_if_t<!is_eigen_v<VT> >*>
    inline Scalar var(VT &&vec, Scalar true_mean)
    {
      auto __vec_eigen = utils::ref_as_eigen(vec);
      return (__vec_eigen.array()*__vec_eigen.array()).sum()/
        static_cast<Scalar>(__vec_eigen.size()) - true_mean*true_mean;
    }

    // =====================================================================
    template<typename Scalar, typename VT,
             enable_if_all_t<is_eigen_v<VT>, dim_dispatch_v<VT>()==1>*>
    inline Scalar var(VT &&vec)
    {
      auto __factor = 1.0/static_cast<Scalar>(vec.size()-1);
      return (vec.array()*vec.array()).sum()*__factor -
        vec.mean()*vec.mean()*__factor*vec.size();
    }

    // =====================================================================
    template<typename Scalar, typename VT,
             enable_if_all_t<is_eigen_v<VT>, dim_dispatch_v<VT>()==1>*>
    inline Scalar var(VT &&vec, uncorrected_t)
    {
      return (vec.array()*vec.array()).sum()/static_cast<Scalar>(vec.size())
        -vec.mean()*vec.mean();
    }

    // =====================================================================
    template<typename Scalar, typename VT,
             enable_if_all_t<is_eigen_v<VT>, dim_dispatch_v<VT>()==1>*>
    inline Scalar var(VT &&vec, Scalar true_mean)
    {
      return (vec.array()*vec.array()).sum()/static_cast<Scalar>(vec.size())
        -true_mean*true_mean;
    }

    // =====================================================================
    template<typename Scalar, typename MT,
             enable_if_all_t<is_eigen_v<MT>, dim_dispatch_v<MT>()!=1>*>
    inline auto var(MT &&data)
      -> Eigen::Matrix<Scalar,dim_dispatch_v<MT>(),dim_dispatch_v<MT>()>
    {
      auto __mean = mean<Scalar>(data);
      auto __factor = 1.0/static_cast<Scalar>(data.cols()-1);
      return (data*data.transpose()).template cast<Scalar>()*__factor
        - (__mean*__mean.transpose())*__factor*data.cols();
    }

    // =====================================================================
    template<typename Scalar, typename MT,
             enable_if_all_t<is_eigen_v<MT>, dim_dispatch_v<MT>()!=1>*>
    inline auto var(MT &&data, uncorrected_t)
      -> Eigen::Matrix<Scalar,dim_dispatch_v<MT>(),dim_dispatch_v<MT>()>
    {
      auto __mean = mean<Scalar>(data);
      return (data*data.transpose()).template cast<Scalar>()/(data.cols())
        - (__mean*__mean.transpose());
    }

    // =====================================================================
    template<typename Scalar, typename MT, typename VT,
             enable_if_all_t<is_eigen_v<MT>,
                             is_eigen_v<VT>,
                             dim_dispatch_v<VT>()==1,
                             dim_dispatch_v<MT>()!=1>*>
    inline auto var(MT &&data, VT &&true_mean)
      -> Eigen::Matrix<Scalar,dim_dispatch_v<MT>(),dim_dispatch_v<MT>()>
    {
      return (data*data.transpose()).template cast<Scalar>()/(data.cols())
        - (true_mean * true_mean.transpose());
    }


  }
}
