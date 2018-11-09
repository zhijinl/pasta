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
// File: computations.hxx for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:24:07 2018 Zhijin Li
// Last update Sat Nov 10 00:14:37 2018 Zhijin Li
// ---------------------------------------------------------------------------


namespace pasta
{
  namespace utils
  {

    // =====================================================================
    template<typename... TPS>
    constexpr inline auto min(TPS &&...args) -> std::common_type_t<TPS...>
    {
      return std::min({std::forward<TPS>(args)...});
    }

    // =====================================================================
    template<typename... TPS>
    constexpr inline auto max(TPS &&...args) -> std::common_type_t<TPS...>
    {
      return std::max({std::forward<TPS>(args)...});
    }

    // =====================================================================
    template<typename T>
    inline auto f_equal(T lhs, T rhs, int ulp)
      -> std::enable_if_t<!std::numeric_limits<T>::is_integer,bool>
    {
      return (std::abs(lhs-rhs) < std::numeric_limits<T>::epsilon()
              *std::abs(lhs+rhs)*ulp)
        // unless the result is subnormal
        || (std::abs(lhs-rhs) < std::numeric_limits<T>::min());
    }

    // =====================================================================
    template<typename T, std::enable_if_t<is_eigen_dynamic_vec_v<T> >*>
    int n_elem(T &&structure)
    {
      return std::forward<T>(structure).size();
    }

    // =====================================================================
    template<typename T,
             enable_if_all_t<is_eigen_v<T>,!is_eigen_dynamic_vec_v<T> >*>
    int n_elem(T &&structure)
    {
      return std::forward<T>(structure).cols();
    }

    // =====================================================================
    template<typename PT, typename BT, typename ST,
             enable_if_all_t<is_eigen_v<PT>,
                             is_eigen_v<BT>,
                             is_eigen_v<ST> >*>
    inline int eigen_pos_indx(PT &&pos, BT &&bound, ST &&spacing)
    {
      constexpr int __dim = eigen_rows_v<PT>;
      static_assert(eigen_cols_v<BT> == 2,
                    "ERROR: EXPECTS A BOUNDING BOX.");
      static_assert(eigen_rows_v<BT> == __dim,
                    "ERROR: POINT AND BOUND DIMENSION MISMATCH.");
      static_assert(eigen_rows_v<ST> == __dim,
                    "ERROR: SPACING AND BOUND DIMENSION MISMATCH.");
      static_assert(std::is_same_v<eigen_val_t<PT>, eigen_val_t<BT> >,
                    "ERROR: POINT AND BOUND VALUE TYPE MISMATCH.");
      static_assert(!std::is_integral_v<eigen_val_t<ST> >,
                    "ERROR: EXPECTS FLOATING TYPES FOR DIVISION PRECISION.");

      int __index = std::round((pos(0)-bound(0,0))/spacing(0));
      for(auto __d = 1; __d < __dim; ++__d)
      {
        int __inner = 1;
        for(auto __n = 0; __n < __d; ++__n)
          __inner *= (std::ceil((bound(__n,1)-bound(__n,0))/spacing(__n))+1);
        __index += std::round((pos(__d)-bound(__d,0))/spacing(__d))*__inner;
      }
      return __index;
    }

    // =====================================================================
    template<typename Mat, typename>
    inline eigen_mat_t<Mat> sqrt_mat(Mat &&mat)
    {
      return Eigen::SelfAdjointEigenSolver<std::decay_t<Mat> >
        (mat.inverse()).operatorSqrt();
    }

    // =====================================================================
    template<typename AT, enable_if_all_t
             <is_eigen_v<AT>, eigen_rows_v<AT> == 3>*>
    inline auto comp_rotmat(AT &&angles) -> rotmat_dispatch_t<AT>
    {
      constexpr int __dim = 3;
      using __scalr_t = eigen_val_t<AT>;
      using __rotmt_t = rotmat_dispatch_t<AT>;

      static_assert(eigen_cols_v<AT> == 1, "ERROR: EXPECTS A COL VEC.");
      static_assert(!std::is_integral_v<__scalr_t>,
                    "ERROR: INTEGRAL TYPE GIVE FOR FLOATING POINT COMPS.");
      __rotmt_t __tmp;
      __rotmt_t __rot_mat = __rotmt_t::Identity();
      for(int __r = 0; __r < 3; ++__r)
      {
        __tmp.setIdentity();
        __tmp((__r+1)%__dim,(__r+1)%__dim) = std::cos(angles(__r));
        __tmp((__r+1)%__dim,(__r+2)%__dim) = -std::sin(angles(__r));
        __tmp((__r+2)%__dim,(__r+1)%__dim) = std::sin(angles(__r));
        __tmp((__r+2)%__dim,(__r+2)%__dim) = std::cos(angles(__r));
        __rot_mat = __tmp*__rot_mat;
      }
      return __rot_mat;
    };

    // =====================================================================
    template<typename AT, enable_if_all_t
             <is_eigen_v<AT>, eigen_rows_v<AT> == 1>*>
    inline auto comp_rotmat(AT &&angle) -> rotmat_dispatch_t<AT>
    {
      static_assert(!std::is_integral_v<eigen_val_t<AT> >,
                    "ERROR: INTEGRAL TYPE GIVE FOR FLOATING POINT COMPS.");

      using __rotmt_t = rotmat_dispatch_t<AT>;
      __rotmt_t __rot_mat;
      __rot_mat(0,0) = std::cos(angle(0));
      __rot_mat(0,1) = -std::sin(angle(0));
      __rot_mat(1,0) = std::sin(angle(0));
      __rot_mat(1,1) = std::cos(angle(0));

      return __rot_mat;
    };

    // =====================================================================
    template<typename ...ATS,
             enable_if_all_t<std::is_arithmetic_v<ATS>...>*>
    inline auto comp_rotmat(ATS ...angles)
      -> typename __rot_dispatcher<std::common_type_t<ATS...>,
                                   sizeof...(ATS)>::type
    {
      constexpr int __nargs = sizeof...(ATS);
      using __scalr_t = std::common_type_t<ATS...>;
      static_assert(__nargs == 1 || __nargs == 3,
                    "ERROR: ROTATION IS NEITHER 2D NOR 3D.");
      static_assert(!std::is_integral_v<__scalr_t>,
                    "ERROR: INTEGRAL TYPE GIVE FOR FLOATING POINT COMPS.");

      return comp_rotmat(make_eigen_pt<__scalr_t>(angles...));
    };

    // =====================================================================
    template<typename MT, enable_if_all_t
             <is_eigen_v<MT>, eigen_rows_v<MT> == 3>*>
    inline auto comp_rot_angles(MT &&rot_mat)
      -> mutate_col_t<eigen_mat_t<MT>,1>
    {
      static_assert(eigen_rows_v<MT> == eigen_cols_v<MT>,
                    "ERROR: INPUT MUST BE 3 x 3 SQUARE MATRIX.");

      using __value_t = eigen_val_t<MT>;
      __value_t angle_x = 0;
      __value_t angle_y = 0;
      __value_t angle_z = 0;

      if( f_equal(rot_mat(2,0), __value_t(1.0)) )
      {
        // cos(angle_y) == 0 && angle_y = pi/2
        angle_z = 0.0;
        angle_y = pst_pi/2.0;
        angle_x = std::atan2(rot_mat(0,1),rot_mat(0,2));

      } else if( f_equal(rot_mat(2,0),  __value_t(-1.0)) )
      {
        // cos(angle_y) == 0 && angle_y = -pi/2
        angle_z = 0.0;
        angle_y = -pst_pi/2.0;
        angle_x = std::atan2(-rot_mat(0,1),-rot_mat(0,2));

      } else
      {
        // angle_y != pi/2 && angle_y != -pi/2 -> asin returns [-pi/2,pi/2].
        angle_y = std::asin(-rot_mat(2,0));
        angle_x = std::atan2(rot_mat(2,1),rot_mat(2,2));
        angle_z = std::atan2(rot_mat(1,0),rot_mat(0,0));
      }
      return { angle_x, angle_y, angle_z };
    }

    // =====================================================================
    template<typename MT, enable_if_all_t
             <is_eigen_v<MT>, eigen_rows_v<MT> == 2>*>
    inline auto comp_rot_angles(MT &&rot_mat) -> eigen_val_t<MT>
    {
      static_assert(eigen_rows_v<MT> == eigen_cols_v<MT>,
                    "ERROR: INPUT MUST BE 2 x 2 SQUARE MATRIX.");
      return std::atan2(rot_mat(1,0),rot_mat(0,0));
    }

    // =====================================================================
    template<typename MT, typename> inline void apply_rhr(MT &mat)
    {
      static_assert(eigen_rows_v<MT> ==3 && eigen_cols_v<MT> == 3,
                    "ERROR: CROSS PRODUCT ONLY FOR VEC OF SIZE 3.");
      if( mat.col(2).dot(mat.col(0).cross(mat.col(1))) < 0 )
        mat.col(2) *= (-1);
    }

    // =====================================================================
    template<typename Vec,
             enable_if_any_t<is_eigen_dynamic_vec_v<Vec>,
                             is_eigen_fixed_row_vec_v<Vec> >*>
    auto comp_bound(Vec &&data) -> bound_dispatch_t<Vec>
    {
      return bound_dispatch_t<Vec>{data.minCoeff(), data.maxCoeff()};
    };

    // =====================================================================
    template<typename MT, std::enable_if_t<is_eigen_dynamic_mat_v<MT> >*>
    auto comp_bound(MT &&data) -> bound_dispatch_t<MT>
    {
      bound_dispatch_t<MT> __result;
      __result << data.rowwise().minCoeff(), data.rowwise().maxCoeff();
      return __result;
    };

    // =====================================================================
    template<typename Bound, typename Steps,
             std::enable_if_t<is_eigen_v<Steps> >*>
    auto comp_bound_discrete_size(Bound &&bound, Steps &&steps)
      -> Eigen::Matrix<int,dim_dispatch_v<Steps>,1>
    {
      return ((bound.col(1)-bound.col(0)).array()/
              (steps.array())).template cast<int>() + 1;
    }

    // =====================================================================
    template<typename Bound, typename Steps,
             std::enable_if_t<!is_eigen_v<Steps> >*>
    int comp_bound_discrete_size(Bound &&bound, Steps steps)
    {
      return static_cast<int>((bound(1)-bound(0))/steps)+1;
    }

    // =====================================================================
    template<typename BT, typename>
    inline auto scale_bound(BT &&bound, float factor)
      -> mutate_col_t<eigen_mat_t<BT>,2>
    {
      using __bound_t = mutate_col_t<eigen_mat_t<BT>,2>;
      using __point_t = mutate_col_t<eigen_mat_t<BT>,1>;
      __bound_t __result;

      __point_t __centre = bound.rowwise().mean();
      __result.col(0) = __centre-(__centre-bound.col(0))*factor;
      __result.col(1) = __centre+(bound.col(1)-__centre)*factor;
      return __result;
    };

    // =====================================================================
    template<typename BT, std::enable_if_t<is_eigen_v<BT> >*>
    inline bool bound_intersect(BT &&lhs, BT &&rhs)
    {
      constexpr int __dim = eigen_rows_v<BT>;
      for(auto __i = 0; __i < __dim; ++__i)
      {
        if( (lhs(__i,0) < rhs(__i,1)) && (lhs(__i,1) > rhs(__i,0)) )
          continue;
        else
          return false;
      }
      return true;
    }

    // =====================================================================
    template<typename PT, typename BT, typename>
    inline bool in_bound(PT &&pt, BT &&bound)
    {
      static_assert(eigen_rows_v<PT> == eigen_rows_v<BT>,
                    "ERROR: PT AND BOUND DIMENSION MISMATCH.");
      static_assert(eigen_cols_v<PT> == 1 && eigen_cols_v<BT> == 2,
                    "ERROR: PT COLS != 1 OR BOUND COLS != 2.");
      using __scalr_t = eigen_val_t<PT>;

      return std::find_if
        (pt.data(),pt.data()+eigen_rows_v<PT>,
         [&pt,&bound](const __scalr_t &__val)
         {
           return (pt(&__val-pt.data()) < bound(&__val-pt.data(),0)) ||
             (pt(&__val-pt.data()) > bound(&__val-pt.data(),1));
         }) == pt.data()+eigen_rows_v<PT>;
    };

    // // =====================================================================
    // template<typename Shape,
    //          std::enable_if_t<is_shape_nonvariant_v<Shape>()>*>
    // inline auto comp_volume(Shape &&shape) -> decltype(shape.volume())
    // {
    //   return std::forward<Shape>(shape).volume();
    // }

    // // =====================================================================
    // template<typename Shape,
    //          enable_if_all_t<!is_shape_nonvariant_v<Shape>(),
    //                          is_pst_shape_v<Shape>()>*>
    // inline auto comp_volume(Shape &&shape) -> pst_shape_scalr_t<Shape>
    // {
    //   using __scalr_t = pst_shape_scalr_t<Shape>;
    //   return boost::apply_visitor(var::comp_volume<__scalr_t>(),
    //                               std::forward<Shape>(shape));
    // }

    // =====================================================================
    template<typename VT> inline auto comp_pol_angles3(VT &&vec)
      -> Eigen::Matrix<eigen_val_t<VT>,2,1>
    {
      static_assert(is_eigen_v<VT> && eigen_rows_v<VT> == 3,
                    "ERROR: INPUT TYPE MUST BE AN EIGEN 3 X 1 VEC.");
      static_assert(!std::is_integral_v<eigen_val_t<VT> >,
                    "ERROR: INT TYPE FOR FLOAT DIVISION: USE EIGEN CAST.");
      // result(0) = theta: x-y plane around old z aixs.
      // result(1) = phi: new z-x plane around new y axis.
      return {std::atan2(vec(1),vec(0)), std::acos(vec(2)/vec.norm())};
    }

    // =====================================================================
    template<typename VT, typename Rule>
    inline auto indexing(VT &&vec, Rule rule)
      -> mutate_val_t<container_dispatch_t<VT>,int>
    {
      mutate_val_t<container_dispatch_t<VT>,int> __indexing(vec.size());
      std::iota( __indexing.data(), __indexing.data()+vec.size(), 0 );
      std::sort( __indexing.data(), __indexing.data()+vec.size(), rule );
      return __indexing;
    }

    // =====================================================================
    template<typename VT> inline auto ascend_indexing(VT &&vec)
      -> mutate_val_t<container_dispatch_t<VT>,int>
    {
      return indexing(std::forward<VT>(vec),[&vec](int i, int j)
                      { return value_at(vec,i) < value_at(vec,j); } );
    }

    // =====================================================================
    template<typename VT> inline auto descend_indexing(VT &&vec)
      -> mutate_val_t<container_dispatch_t<VT>,int>
    {
      return indexing(std::forward<VT>(vec),[&vec](int i, int j)
                      { return value_at(vec,i) > value_at(vec,j); } );
    }

    // =====================================================================
    template<typename VT>
    inline auto cumsum(VT &&vec) -> container_dispatch_t<VT>
    {
      container_dispatch_t<VT> __res(vec.size());
      using size_type = std::decay_t<decltype(__res.size())>;

#pragma omp parallel for
      for(size_type __i = 0; __i < __res.size(); ++__i)
        value_at(__res,__i) = ref_as_eigen(vec).segment(0,__i+1).sum();
      return __res;
    }

    // =====================================================================
    template<typename FwdItr, typename T, typename Comp>
    FwdItr binary_find(FwdItr first, FwdItr last, const T& val, Comp comp)
    {
      first = std::lower_bound(first, last, val);
      return first != last && !comp(val, *first) ? first : last;
    }

  } //!utils
} //!pasta
