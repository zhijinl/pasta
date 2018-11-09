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
// File: computations.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:23:16 2018 Zhijin Li
// Last update Sat Nov 10 00:14:43 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_COMPUTATIONS_HH
# define PASTA_COMPUTATIONS_HH

# include "pasta/Utilities/utils.hh"
# include "pasta/Core/visitors.hh"


namespace pasta
{
  /// @ingroup group_utils
  namespace utils
  {

    /// @ingroup group_utils
    ///
    /// @brief Multi-instance min.
    ///
    /// @param args: variadic arithmetic arguments.
    /// @return The min element.
    ///
    template<typename... TPS>
    constexpr auto min(TPS &&...args) -> std::common_type_t<TPS...>;

    /// @ingroup group_utils
    ///
    /// @brief Multi-instance max.
    ///
    /// @param args: variadic arithmetic arguments.
    /// @return The max element.
    ///
    template<typename... TPS>
    constexpr auto max(TPS &&...args) -> std::common_type_t<TPS...>;

    /// @ingroup group_utils
    ///
    /// @brief Floating point equal comparison using machine epsilon.
    ///
    /// The machine epsilon  has  to be  scaled to the magnitude of
    /// the values used and  multiplied by the desired precision in
    /// ULPs (units in the last place).
    ///
    /// @warning `lhs` and `rhs` must be the same type.
    ///
    /// @param lhs: left hand side.
    /// @param rhs: right hand side.
    /// @param lhs: units in the last place.
    /// @return True if `lhs == rhs` in within machine tolerance.
    ///
    template<typename T> auto f_equal(T lhs, T rhs, int ulp=2)
      -> std::enable_if_t<!std::numeric_limits<T>::is_integer,bool>;

    ///@{
    /// @ingroup group_utils
    ///
    /// @brief Get the number of elements inside a `pasta` structure.
    ///
    /// The number of structural elements is
    /// * the number of element when an input is a dynamic Eigen vector.
    /// * the number of columns otherwise. In this case a structure in
    ///   `pasta` is always assumed column ordered.
    ///
    /// @param structure: the input structure.
    /// @return The number of elements inside the input structure.
    ///
    template<typename T, std::enable_if_t<is_eigen_dynamic_vec_v<T> >* = nullptr>
    int n_elem(T &&structure);

    template<typename T,
             enable_if_all_t<is_eigen_v<T>,
                             !is_eigen_dynamic_vec_v<T> >* = nullptr>
    int n_elem(T &&structure);
    ///@}

    /// @ingroup group_utils
    ///
    /// @brief Wrap spatial position to an array index given bound and spacing
    /// in length-units.
    ///
    /// A position in N-dimensional coordinates can be discretized and wrapped
    /// to a **1-dimensional** array using the following relation:
    ///
    /// `indx = pt(0)/spacing(0) + pt(1)/spacing(1)*dim0 + pt(2)/spacing(2)*`
    /// `dim1*dim0 + ...`
    ///
    /// This assumed the 1D array starts incrementing in low-high dimensional
    /// order. This function does no bound check.
    ///
    /// @param pos: the input spatial position: an Eigen point. In length-
    /// units.
    /// @param bound: the reference point: an Eigen point. In length-units.
    /// @param spacing: desired spacing: an Eigen point. In length-units.
    /// @return The mapped integral array index.
    ///
    template<typename PT, typename BT, typename ST,
             enable_if_all_t<is_eigen_v<PT>,
                             is_eigen_v<BT>,
                             is_eigen_v<ST>>* = nullptr>
    int eigen_pos_indx(PT &&pos, BT &&bound, ST &&spacing);

    /// @ingroup group_utils
    ///
    /// @brief Compute square root of a matrix.
    ///
    /// A matrix @f$ R @f$ is **said to be the square root of** a matrix
    /// @f$ A @f$ if:
    ///
    /// @f$ A = R \times A @f$
    ///
    /// @warning For now, it works only for **positive definite square
    /// matrices**. This function does no entry check, it is the user's
    /// responsibility to pass a conformant matrix.
    ///
    /// @param mat: an input Eigen matrix.
    /// @return The computed matrix square root.
    ///
    template<typename Mat, typename = std::enable_if_t<is_eigen_v<Mat> > >
    eigen_mat_t<Mat> sqrt_mat(Mat &&mat);

    ///@{
    /// @ingroup group_utils
    ///
    /// @brief Compute rotation mat from given angles.
    ///
    /// @warning No check here. Its the caller's responsibility to verify
    /// that all orientation angles are initialized. Also caller need to
    /// make sure that
    /// AT::RowsAtCompileTime == Dim && AT::ColsAtCompileTime == 1.
    ///
    /// @note Final rot_mat @f$ R = R_z \times R_y \times R_x @f$. Matrix
    /// pre-multiplies. This is an **extrinsic rotation in order x->y->z**.
    /// **Equal to an intrinsic rotation in z->y->x order**, each time about
    /// the **modified/rotated new axis**.
    ///
    /// @param angles: the input angles. An Eigen fixed size Dim x 1 vec.
    /// @return The computed rotation matrix. An Eigen Dim x Dim matrix.
    ///
    template<typename AT, enable_if_all_t
             <is_eigen_v<AT>, eigen_rows_v<AT> == 3>* = nullptr>
    auto comp_rotmat(AT &&angles) -> rotmat_dispatch_t<AT>;

    template<typename AT, enable_if_all_t
             <is_eigen_v<AT>, eigen_rows_v<AT> == 1>* = nullptr>
    auto comp_rotmat(AT &&angle) -> rotmat_dispatch_t<AT>;
    ///@}

    ///@{
    /// @ingroup group_utils
    ///
    /// @brief Compute rot mat from given angles. Arithmetic overload.
    ///
    /// No check here. Its the caller's responsibility to verify that all
    /// orientation angles are initialized. Also caller need to make sure
    /// that AT::RowsAtCompileTime == Dim && AT::ColsAtCompileTime == 1.
    ///
    /// Final rot_mat @f$ R = R_z \times R_y \times R_x @f$. Matrix pre-
    /// multiplies. This is an **extrinsic rotation in order x->y->z**.
    /// **Equal to an intrinsic rotation in z->y->x order**, each time about
    /// the **modified/rotated new axis**.
    ///
    /// @param angles: the input angles as separate arithmetic args.
    /// @return The computed rotation matrix. An Eigen Dim x Dim matrix.
    ///
    template<typename ...ATS,
             enable_if_all_t<std::is_arithmetic_v<ATS>...>* = nullptr>
    auto comp_rotmat(ATS ...angles)
      -> typename __rot_dispatcher
      <std::common_type_t<ATS...>,sizeof...(ATS)>::type;

    ///@{
    /// @ingroup group_utils
    ///
    /// @brief Compute rotation angles from a rot matrix.
    ///
    /// **Works only for 2D & 3D**. Compute extrinsic rotation angles.
    /// Return a fixed Dim x 1 vec containing angles of rotation **in:
    /// z->y->x order**, where each time about modified/rotated new axis.
    ///
    /// @param rot_mat: the input rotation matrix
    /// @return The computed angles: fixed-size Dim x 1 Eigen vec.
    ///
    template<typename MT, enable_if_all_t
             <is_eigen_v<MT>, eigen_rows_v<MT> == 3>* = nullptr>
    auto comp_rot_angles(MT &&rot_mat)
      -> mutate_col_t<eigen_mat_t<MT>,1>;

    template<typename MT, enable_if_all_t
             <is_eigen_v<MT>, eigen_rows_v<MT> == 2>* = nullptr>
    auto comp_rot_angles(MT &&rot_mat) -> eigen_val_t<MT>;
    ///@}

    /// @ingroup group_utils
    ///
    /// @brief Apply right-hand rule to a rotation (orthogonal) matrix.
    ///
    /// This makes sense only for 3D roration matrices.
    /// Assume each column is a directional vector. Apply Right-
    /// hand rule to the third column vec if condition it is not
    /// aligned in the same direction as `x vector_prod y`.
    ///
    /// @warning No check for orthogonality.
    ///
    /// @param rot_mat [in/out]: an input rotation/orthogonal matrix.
    ///
    ///
    template<typename MT, typename = enable_if_all_t<is_eigen_v<MT> > >
    void apply_rhr(MT &rot_mat);

    /// @ingroup group_utils
    ///
    /// @brief Compute bounding box of an Eigen dynamic vector, or fix
    /// sized row vector.
    ///
    /// @param data: the input Eigen vector. When **it is dynamic**, it
    /// can be row or col ordered. When it is fixed, it **must be a row
    /// vec**, for example a fix sized 1 x 10 vector.
    /// @return The computed bound: Eigen 1 x 2 point.
    ///
    template<typename Vec,
             enable_if_any_t<is_eigen_dynamic_vec_v<Vec>,
                             is_eigen_fixed_row_vec_v<Vec> >* = nullptr>
    auto comp_bound(Vec &&data) -> bound_dispatch_t<Vec>;

    /// @ingroup group_utils
    ///
    /// @brief Compute bounding box of an Eigen matrix.
    ///
    /// @param data: the input matrix, fixed or dynamic. It is **assumed
    /// to be column ordered**, i.e. each column represents a data point.
    /// @return The computed bound in form of a Dim by 2 mat / point.
    ///
    template<typename MT,
             std::enable_if_t<is_eigen_dynamic_mat_v<MT> >* = nullptr>
    auto comp_bound(MT &&data) -> bound_dispatch_t<MT>;

    /// @ingroup group_utils
    ///
    /// @brief Compute discrete sizes of a bounding box with respect to
    /// given step-sizes. For Dim > 1 cases.
    ///
    /// @note The computation is always conservative. Meaning that
    /// computed discrete size is never larger than size in length
    /// units. For example, for an interval (0.0, 1.05), with 0.1
    /// used as step size. The discrete size will be 11: the 0.05
    /// in the upper bound will always get rounded-off.
    ///
    /// @param bound: the input bounding box. An Eigen Dim x 2
    /// structure.
    /// @param steps: the input step size. An Eigen Dim x 1 sized
    /// vector.
    /// @return The compute size vector. An Eigen Dim x 1 integer
    /// vector.
    ///
    template<typename Bound, typename Steps,
             std::enable_if_t<is_eigen_v<Steps> >* = nullptr>
    auto comp_bound_discrete_size(Bound &&bound, Steps &&steps)
      -> Eigen::Matrix<int,dim_dispatch_v<Steps>,1>;

    /// @ingroup group_utils
    ///
    /// @brief Compute discrete size of a bounding box with respect to
    /// given step-sizes. For Dim == 1 cases.
    ///
    /// @note The computation is always conservative. Meaning that
    /// computed discrete size is never larger than size in length
    /// units. For example, for an interval (0.0, 1.05), with 0.1
    /// used as step size. The discrete size will be 11: the 0.05
    /// in the upper bound will always get rounded-off.
    ///
    /// @param bound: the input bounding box. An Eigen 1 x 2
    /// structure.
    /// @param steps: the input step size. A floating-point scalar.
    /// @return The compute size vector. An integer scalar.
    ///
    template<typename Bound, typename Steps,
             std::enable_if_t<!is_eigen_v<Steps> >* = nullptr>
    int comp_bound_discrete_size(Bound &&bound, Steps steps);

    /// @ingroup group_utils
    ///
    /// @brief Scale a bounding box by a factor.
    ///
    /// The bound center stays the same, but the bound expands
    /// are streched by factor in all directions.
    /// @param bound: the input bound. Eigen Dim x 2.
    /// @param factor: the scaling factor.
    /// @return The scaled bound. Eigen Dim x 2.
    ///
    template<typename BT, typename = std::enable_if_t<is_eigen_v<BT> > >
    auto scale_bound(BT &&, float) -> mutate_col_t<eigen_mat_t<BT>,2>;

    /// @ingroup group_utils
    ///
    /// @brief Comp if two bouding_boxes intersect each other.
    ///
    /// This conditioning generalize to arbitrary dim:
    /// (lhs(i,0) < rhs(i,1)) && (lhs(i,1) > rhs(i,0))
    ///
    /// @param lhs: one bounding box (Dim x 2 matrix).
    /// @param rhs: another bounding box (Dim x 2 matrix).
    /// @return True if intersect.
    ///
    template<typename BT, std::enable_if_t<is_eigen_v<BT> >* = nullptr>
    bool bound_intersect(BT &&, BT &&);

    /// @ingroup group_utils
    ///
    /// @brief Test if a pt is inside querry bounding box.
    ///
    /// @warning No bound check: caller's responsibility.
    ///
    /// @param pt: the input test point.
    /// @param bound: the bounding box to test on.
    /// @return: bool if the point is inside the bounding box.
    ///
    template<typename PT, typename BT, typename = enable_if_all_t
             <is_eigen_v<PT>, is_eigen_v<BT> > >
    bool in_bound(PT &&, BT &&);

    // ///@{
    // /// @ingroup group_utils
    // ///
    // /// @brief Compute volume of a shape.
    // ///
    // /// @param shape: the input shape, could be a real shape, or a
    // /// shape variant.
    // /// @return The computed volume.
    // ///
    // template<typename Shape,
    //          std::enable_if_t<is_shape_nonvariant_v<Shape>()>* = nullptr>
    // auto comp_volume(Shape &&shape) -> decltype(shape.volume());

    // template<typename Shape,
    //          enable_if_all_t<!is_shape_nonvariant_v<Shape>(),
    //                          is_pst_shape_v<Shape>()>* = nullptr>
    // auto comp_volume(Shape &&shape) -> pst_shape_scalr_t<Shape>;
    // ///@@}

    /// @ingroup group_utils
    ///
    /// @brief Compute polar coord angles in 3D.
    ///
    /// The result(0) is the rot angle in old x-y plane, by right-hand-rule
    /// around the old z axis. result(1) is the rot angle in the new z-x
    /// plane perpen to x-y, around the new y axis, by right-hand-rule.
    ///
    /// @note result(0) is in the range [-pi, pi], rsult(1) [0,pi].
    /// Might want to subtract result(1) by pi/2 if dealing with rotation.
    ///
    /// @param vec: the input Eigen vector.
    /// @return An Eigen fix-size 2x1 vector.
    ///
    template<typename VT> auto comp_pol_angles3(VT &&)
      -> Eigen::Matrix<eigen_val_t<VT>,2,1>;

    /// @ingroup group_utils
    ///
    /// @brief Return index ordering of an vector based on a certain rule.
    ///
    /// Ret an int vector of STL or Eigen representing the ordering of
    /// indices defined by a rule.
    ///
    /// @param vec: an input vector of STL or Eigen.
    /// @param rule: could be a functor, std::Comp, lambda expr etc. A
    /// binary operation defining the ordering rule..
    /// @return Ordering of indices. Same as input, but with data type
    /// mutated to int.
    ///
    template<typename VT, typename Rule> auto indexing(VT &&, Rule)
      -> mutate_val_t<container_dispatch_t<VT>,int>;

    /// @ingroup group_utils
    ///
    /// @brief Return index ordering of an vector based on ascendant rule.
    ///
    /// Ret an int vector of STL or Eigen representing the ordering of
    /// indices in ascendant value order.
    ///
    /// @param vec: an input vector of STL or Eigen.
    /// @return Ascendant ordering of indices. Same as input, but with
    /// data type mutated to int.
    ///
    template<typename VT> auto ascend_indexing(VT &&)
      -> mutate_val_t<container_dispatch_t<VT>,int>;

    /// @ingroup group_utils
    ///
    /// @brief Return index ordering of an vector based on descendant rule.
    ///
    /// Ret an int vector of STL or Eigen representing the ordering of
    /// indices in descendant value order.
    ///
    /// @param vec: an input vector of STL or Eigen.
    /// @return Descendant ordering of indices. Same as input, but with
    /// data type mutated to int.
    ///
    template<typename VT> auto descend_indexing(VT &&)
      -> mutate_val_t<container_dispatch_t<VT>,int>;

    /// @ingroup group_utils
    ///
    /// @brief Compute cummulative sum of a vector.
    ///
    /// @param vec: the input vector of STL or Eigen.
    /// @return The cummulative sum vector, same type as input.
    ///
    template<typename VT> auto cumsum(VT &&)
      -> container_dispatch_t<VT>;

    /// @ingroup group_utils
    ///
    /// @brief Binary search of a value in a **sorted / or partially sorted
    /// range** `[first, last)`.
    ///
    /// Complexity: at most `log(last - first) + O(1)`.
    ///
    /// @warning Type of `value` must be implicitly convertible to the
    /// value_type of the range. **Do not use this function with unsorted
    /// ranges.**
    ///
    /// @param first: beginning of the range.
    /// @param last: one-past-the-end of the range.
    /// @param val: value to find.
    /// @param comp: comparison function, defaults to `std::less<>`
    ///
    template<typename FwdItr, typename T, typename Comp=std::less<T> >
    FwdItr binary_find(FwdItr first, FwdItr last, const T& val, Comp comp={});

  }
}


# include "computations.hxx"
#endif
