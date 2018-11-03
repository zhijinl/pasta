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
// File: utils.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:27:29 2018 Zhijin Li
// Last update Sat Nov  3 23:33:14 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_UTILS_HH
# define PASTA_UTILS_HH

# include <chrono>
# include <memory>
# include <random>
# include <numeric>
# include <iomanip>
# include <functional>
# include "pasta/Core/meta.hh"
# include "pasta/Core/visitors.hh"


/// @ingroup group_utils
///
/// @brief Bench-mark a section of program.
///
/// @warning (**what makes macros evil**)
/// - Enclose the beginning & the end of the bench-mark session
///   with `{ }`, and put the macro at the beginning, inside the
///   first `{`.
/// - Give a **unique section name** to each bench-mark session.
///   Be carefull to **not violate ODR (One Definition Rule)**.
/// - `SECTION_NAME` must be a **single word** satisfying C++'s
///   variable naming rules.
/// - Be carefull with **the scope of variables** for bench-mark
///   of multiple code sessions.
///
# define PASTA_BENCHMARK(SECTION_NAME)                                 \
  auto SECTION_NAME##__ = pasta::utils::timer{#SECTION_NAME};   \

namespace pasta
{
  /// @ingroup group_utils
  namespace utils
  {

    /// @ingroup group_utils
    ///
    /// @brief Convenient bench-marker.
    ///
    /// Use it with `PASTA_BENCHMARK` macro.
    ///
    struct timer
    {
      /// @brief Construct a timer object.
      ///
      /// @param name: name of the timer.
      ///
      explicit timer(const char *name);

      /// @brief (Re)initialize the timer object to keep track
      /// of the current timestamp.
      ///
      void tic();

      /// @brief Measure current time elapse and output to
      /// stdout.
      ///
      void toc() const;

    private:
      std::string _name; //<: Timer name.
      long   _timestamp; //<: Tracked timestamp.
    };

    /// @ingroup group_utils
    ///
    /// @brief Set a timer. Return a timestamp in milliseconds.
    ///
    auto set_timer_ms() -> decltype
      (std::chrono::duration_cast<std::chrono::milliseconds>
       (std::chrono::system_clock::now().time_since_epoch()).count());

    /// @ingroup group_utils
    ///
    /// @brief Print on console a loadbar.
    ///
    /// @param ind: current index.
    /// @param total: total breadth of index.
    /// @param width: bar width.
    ///
    void loadbar(int ind, int total, int width=50);

    /// @ingroup group_utils
    ///
    /// @brief Create std::array filled with an input value.
    ///
    /// @param value: value to fill the `std::array`.
    /// @return An `std::array` filled with `value`.
    ///
    template <int N, typename T>
    std::array<T, N> make_array(const T& value);

    /// @ingroup group_utils
    ///
    /// @brief Forwarding function for unique_ptr factory.
    ///
    /// @param args: variadic args for constructor of T.
    /// @return `unique_ptr` of T.
    ///
    template<typename T, typename... TS>
    auto make_unique(TS &&...) -> std::unique_ptr<T>;

    /// @ingroup group_utils
    ///
    /// @brief Create a shared random engine for rng.
    ///
    /// Thread-safe (since c++11).
    /// @param seed: random seed, dflt to current time since epoch count.
    /// @return Ref to the static random engine.
    ///
    std::mt19937_64& shared_engine(std::mt19937_64::result_type seed=
                                   std::chrono::high_resolution_clock::
                                   now().time_since_epoch().count());

    /// @ingroup group_utils
    ///
    /// @brief Re-seeds the shared global random engine.
    ///
    /// This function is handy when you want to generate
    /// reproducible result from pasta's random objects.
    ///
    /// @warning When creating reproducible simulations,
    /// This function should be paired with the
    /// `reset_state` method of a random object. Some of
    /// the random objects have internal state, such that
    /// the results of simulation **do not only depend on
    /// the random engine**. Therefore, in order to have
    /// reproducible simulations, you should also call
    /// the `reset_state` method after resetting the engine.
    /// The `reset_state` method is available for all
    /// pasta random object classes.
    ///
    /// @param seed: new seed to be used, default to 0.
    ///
    void reset_shared_engine(std::mt19937_64::result_type seed=0);

    ///@{
    /// @ingroup group_utils
    ///
    /// @brief Make a Eigen column vector of Dim x 1 with equal elements.
    ///
    /// This function is convenient to create a vector with all elements
    /// duplicating a single value. It is also generic for all dimension.
    ///
    /// @note In Dim == 1 case, it will just return the scalar value: not
    /// an Eigen size 1 matrix.
    ///
    /// Need to explicitly specify template argument for Dim.
    ///
    /// @param val: val assigned to each element of the result point.
    /// @return The created Eigen point with all element equal to val.
    ///
    template<typename ScalrType, int Dim, typename T,
             std::enable_if_t<Dim!=1>* = nullptr>
    auto make_const_pt(T val) -> Eigen::Matrix<ScalrType,Dim,1>;

    template<typename ScalrType, int Dim, typename T,
             std::enable_if_t<Dim==1>* = nullptr>
    auto make_const_pt(T val) -> ScalrType;
    ///@}

    ///@{
    /// @ingroup group_utils
    ///
    /// @brief Generic numeric cast operation.
    ///
    /// When input is a scalar, it wraps to do a static_cast to desire type
    /// When input is an Eigen structure, it uses Eigen `cast()` function.
    /// When the input Eigen structure has the same scalar type as the dest
    /// it is a no-op: the function just forwards the input, keeping the
    /// same cvr qualifier.
    ///
    /// @param Dst: template argument. The desired result numeric type for
    /// cast.
    /// @param src: the input src object. Passed by forwarding reference.
    /// @return The cast result.
    ///
    template<typename Dst, typename Src,
             std::enable_if_t<std::is_arithmetic_v<Src> >* = nullptr>
    auto cast_to(Src &&src)
      -> decltype(static_cast<Dst>(std::declval<Src>()));

    template<typename Dst, typename Src, enable_if_all_t
             <is_eigen_v<Src>,
              !std::is_same_v<Dst,eigen_val_t<Src> > >* = nullptr>
    auto cast_to(Src &&src) -> decltype(src.template cast<Dst>());

    template<typename Dst, typename Src, enable_if_all_t
             <is_eigen_v<Src>,
              std::is_same_v<Dst,eigen_val_t<Src> > >* = nullptr>
    auto cast_to(Src &&src) -> decltype(std::forward<Src>(src));
    ///@}

    ///@{
    /// @ingroup group_utils
    ///
    /// @brief Create an Eigen pt / vector from various input types.
    ///
    /// The **rules** are:
    /// - For individually provided arithmetic numbers, it returns a **fixed
    ///   sized column vector**.
    /// - For Eigen vector input, dynamic or fix, column or row ordered, it
    ///   just forward the input value, **keeping the same cvr qualifier**,
    ///   when no cast is needed, i.e. the desire scalar type is the same as
    ///   that of the input vector; otherwise it performs an arithmetic type
    //    cast, and **return the casted new vector expression**.
    /// - For `initializer_list`, it will construct a **dynamic Eigen column
    ///   vector**.
    /// - For any input type that is none of the above, including Eigen
    ///   matrices, it will result in a compilation error.
    ///
    /// @note
    /// - Always return an Eigen type. Even for Dim == 1, an Eigen vector
    ///   of size 1 will be returned.
    /// - The behavior for `initializer_list` might change in the future. The
    ///   `initializer_list` cannot be constexpr prior to C++14
    ///
    /// @param args: could be variadic args for point coordinates. An Eigen
    /// vector or an `initialize_list`.
    /// @return An Eigen point / vector. See rules for detail.
    ///
    template<typename T, typename... TS,
             enable_if_all_t<(sizeof...(TS) <= 4 && sizeof...(TS) > 1),
                             std::is_arithmetic_v<TS>...>* = nullptr>
    auto make_eigen_pt(TS ...args) -> Eigen::Matrix<T,sizeof...(TS),1>;

    template<typename T, typename... TS,
             enable_if_all_t<(sizeof...(TS) > 4 || sizeof...(TS) == 1),
                             std::is_arithmetic_v<TS>...>* = nullptr>
    auto make_eigen_pt(TS ...args) -> Eigen::Matrix<T,sizeof...(TS),1>;

    template<typename T, typename Input,
             typename = std::enable_if_t<is_eigen_vec_v<Input>()> >
    auto make_eigen_pt(Input &&input)
      -> decltype(cast_to<T>(std::forward<Input>(input)));

    template<typename T, typename Scalar>
    auto make_eigen_pt(std::initializer_list<Scalar> input)
      -> Eigen::Matrix<T,Eigen::Dynamic,1>;
    ///@}

    ///@{
    /// @ingroup group_utils
    ///
    /// @brief Generic element accessor for an Eigen structure.
    ///
    /// Boilds down to const ref / non-const ref to an element of the input
    /// Eigen structure or a column of it, depending on the structural dim
    /// of the input.
    ///
    /// @param input: the input Eigen structure. For example an Eigen dynam
    /// vector or Eigen matrix.
    /// @param index: integer access index.
    /// @return Reference or const reference, depending on the const qualifier
    /// of input, to indexed structural element in input.
    ///
    template<typename T,
             enable_if_all_t<is_eigen_v<T>,
                             dim_dispatch_v<T>()==1>* = nullptr>
    auto elem_at(T &&input, int index) -> decltype(input(0));

    template<typename T,
             enable_if_all_t<is_eigen_v<T>,
                             dim_dispatch_v<T>()!=1>* = nullptr>
    auto elem_at(T &&input, int index) -> decltype(input.col(0));
    ///@}

    ///@{
    /// @ingroup group_utils
    ///
    /// @brief Slicing of an input structure based on logic defined in an
    /// `std::vector`, Eigen vector or Eigen column matrix.
    ///
    /// @note this is in-place slicing, works only for Dynamic Matrices.
    ///
    /// @warning When the input is an Eigen matrix, it **must be column
    /// ordered**.
    ///
    /// @param mat: the input. An `std::vector`, Eigen vector or Eigen col
    /// matrix.
    /// @param lv: the input logical vector, with its contents indicating
    /// which element in mat is to be sliced-off. Of type STL or Eigen.
    /// @param keep_val: rule for which value in lv indicates elements to
    /// keep. Defaults to 1.
    /// @param shrink_after: this is valid only for `std::vector`s, it
    /// indicates if the `shrink_to_fit` method of the `std::vector` will
    /// be called after slicing to **deallocate unused space**. Default is
    /// false: since one might need to preserved the originally reserved
    /// capacity.
    ///
    template<typename T, typename Alloc, typename Vec>
    void logical_slice(std::vector<T,Alloc> &input, Vec &&lv,
                       value_dispatch_t<Vec> keep_val=1,
                       bool shrink_after=false);

    template<typename T, typename Vec, int Dim, std::enable_if_t<Dim!=1>* = nullptr>
    void logical_slice(Eigen::Matrix<T,Dim,Eigen::Dynamic> &input, Vec &&lv,
                       value_dispatch_t<Vec> keep_val=1);

    template<typename T, typename Vec, int Dim, std::enable_if_t<Dim!=1>* = nullptr>
    void logical_slice(Eigen::Matrix<T,Eigen::Dynamic,Dim> &input, Vec &&lv,
                       value_dispatch_t<Vec> keep_val=1);

    template<typename T, typename Vec, int Rows, int Cols,
             typename = enable_if_any_t<(Rows==1 && Cols==Eigen::Dynamic),
                                        (Cols==1 && Rows==Eigen::Dynamic)> >
    void logical_slice(Eigen::Matrix<T,Rows,Cols> &input, Vec &&lv,
                       value_dispatch_t<Vec> keep_val=1);
    ///@}

    /// @ingroup group_utils
    ///
    /// @brief Bind matrices in column direction.
    ///
    /// New matrix will be added to the right of the previous one. Assume
    /// all matrices have the same number of rows.
    ///
    /// @param mat_vec: an STL vector of matrices.
    /// @return The binded new matrix using matrices in the vector, in the
    /// left to right order.
    ///
    template<typename Mat, typename = std::enable_if_t<is_eigen_v<Mat> > >
    auto bind_cols(const std::vector<Mat> &mat_vec)
      -> mutate_col_t<Mat,Eigen::Dynamic>;
    ///@}

    ///@{
    /// @ingroup group_utils
    ///
    /// @brief Remove one column from an Eigen dynamic colmajor matrix, or
    /// an element from an `std::vector`.
    ///
    /// @param input: the input matrix or `std::vector`.
    /// @param indx: index of the column / element to be removed.
    ///
    template<typename T, int Dim>
    void remove_elem(Eigen::Matrix<T,Dim,Eigen::Dynamic> &input, int indx);

    template<typename T, typename Alloc>
    void remove_elem(std::vector<T,Alloc> &input, std::size_t indx);
    ///@}

    ///@{
    /// @ingroup group_utils
    ///
    /// @brief Forward an `std::vector` or `std::array` as an Eigen Map.
    ///
    /// Return an Eigen::Map structure offering all Eigen matrix & vector
    /// interfaces.
    ///
    /// @note For generic reason, it does nothing when an Eigen structure
    /// is passed.
    ///
    /// @warning Be sure to make the lifetime of returned map **local to
    /// scope** since the underlying array of `std::vector` or `std::array`
    /// is shared by the new map -> it may cause surprises if not used with
    /// care.
    ///
    /// @param input: the input `std::vector` or `std::array`, or an Eigen
    /// type.
    /// @return A dynamic or fixed size Eigen Map structure mapping a col
    /// vector.
    ///
    template<typename T, typename Alloc>
    auto ref_as_eigen(std::vector<T,Alloc> &input)
      -> Eigen::Map<Eigen::Matrix<T,Eigen::Dynamic,1> >;

    template<typename T, std::size_t N>
    auto ref_as_eigen(std::array<T,N> &input)
      -> Eigen::Map<Eigen::Matrix<T,N,1> >;

    template<typename Input, std::enable_if_t<is_eigen_v<Input> >* = nullptr>
    auto ref_as_eigen(Input &&input) -> decltype(std::forward<Input>(input));
    ///@}

    namespace detail
    {

      /// @brief Implementation of make array using index sequences and
      /// operator `,`.
      ///
      /// @param value: uniform value for the array elements.
      ///
      template <typename T, int ...I>
      std::array<T, sizeof...(I)>
      make_array_impl(const T& value, indx_seq<I...>);
    }
  }
}


# include "utils.hxx"
#endif
