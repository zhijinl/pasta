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
// File: meta.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Thu Nov  1 23:06:31 2018 Zhijin Li
// Last update Sat Nov 10 00:21:04 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_META_HH
# define PASTA_META_HH

# include <tuple>
# include <variant>
# include <type_traits>

# include "defs.hh"


namespace pasta
{

  /// @defgroup group_meta Meta-Programming Utilities
  ///
  /// @brief Template meta-programming utilities for `pasta`:
  /// - Compile-time computations.
  /// - SFINAE friendly interfaces.
  /// - Type detection, manipulation & transformation.
  ///
  /// SFINAE helpers + enable_if utils are recommanded to be used for
  /// **type constraint of templated functions**. It helps you write
  /// efficient code taking full advantage of `Eigen` expression
  /// templates. The syntaxes can sometimes be very verbose.
  ///
  /// Large part of this code base will be re-written when C++
  /// `Concept` is available.
  ///

  /// @ingroup group_meta
  /// @brief Get size of a static c-style array at compile-time.
  ///
  /// @param AR: the input array.
  ///
  template<typename AR, int N>
  constexpr int c_arr_size(AR (&)[N]) noexcept { return N; }


  /// @brief Check if a pack of types is non-empty
  ///
  /// @param Types: an input pack pf types.
  /// @return True if the pack contains at least one type.
  ///
  template<typename ...Types>
  constexpr bool not_empty_v = (sizeof...(Types) > 0);


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Utility constexpr to be used with `static_assert`.
  ///
  /// Triggers compile time error when a template function
  /// or class is initiated.
  ///
  /// @warning Valid only when the template parameter T is a
  /// template parameter (not a specified type, like void).
  ///
  template<typename T, typename U>
  struct err_on_call_t: std::false_type {};

  template<typename T, typename U=T>
  constexpr bool err_on_call_v = err_on_call_t<T,U>::value;
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Detect if an input type is an `Eigen` data structure.
  ///
  /// @note To check if an expression produces an Eigen type, it **uses
  /// the fact that Eigen structures have** `RealScalar` **typedef**.
  ///
  /// @param T: an input type.
  /// @return True if the input type is an `Eigen` data structure.
  ///
  template<typename Input, typename = std::void_t<> >
  struct __has_realscalar: std::false_type {};

  template<typename Input>
  struct __has_realscalar
  <Input, std::void_t<typename Input::RealScalar> >:
    std::true_type {};

  template<typename Input>
  constexpr bool is_eigen_v =
    __has_realscalar<std::decay_t<Input> >::value;
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Detect if an input data structure can be accessed of its
  /// elements throught the `()` (parenthesis) operator.
  ///
  /// @param T: an input type.
  /// @return True if the input type is parentable.
  ///
  template <typename T, typename ...TS> struct __has_paren
  {
    template <typename T1, typename ...TS1>
    static decltype(std::declval<T1>()(std::declval<TS1>()...)) check(int);

    template <typename...> static void check(...);
    //< Note: ellipsis is always the last of overload resolution.

    static const bool value = !std::is_void<decltype(check<T,TS...>(0))>::value;
  };

  template<typename T, typename ...TS>
  constexpr bool is_parentable_v =
    __has_paren<std::decay_t<T>,std::decay_t<TS>...>::value;
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Detect if an input data structure can be accessed of its
  /// elements throught the `[]` (bracket) operator.
  ///
  /// @param T: an input type.
  /// @return True if the input type is bracketable.
  ///
  template <typename T, typename AT> struct __has_bracket
  {
    template <typename T1, typename AT1>
    static decltype(std::declval<T1>()[std::declval<AT1>()]) check(int);

    template <typename...> static void check(...);
    //< Note: ellipsis is always the last of overload resolution.

    static const bool value = !std::is_void<decltype(check<T,AT>(0))>::value;
  };

  template<typename T, typename AT>
  constexpr bool is_bracketable_v =
    __has_bracket<std::decay_t<T>,std::decay_t<AT> >::value;
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check if an input structure has the `dim` traits property.
  ///
  /// @param T: an input type.
  /// @return True if the `dim` property exists in the input type's traits.
  ///
  template<typename Input, typename = std::void_t<> >
  struct __has_dim: std::false_type {};

  template<typename Input>
  struct __has_dim
  <Input, std::void_t<decltype(traits::specs<Input>::dim)> >:
    std::true_type {};

  template<typename Input>
  constexpr bool has_dim_v = __has_dim<std::decay_t<Input> >::value;
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief `enable_if_all_t` and `enable_if_any_t`.
  ///
  /// @note They behaves exactly as their names indicate:
  /// `enable_if_all` bails out **right after the first false argument it
  /// encounters**, while `enable_if_any` bails out **right after the first
  /// true argument it encounters**.
  ///
  /// @param Bs: a sequence of `bool`s.
  /// @return `void` if the logical AND | OR stands, respectively.
  ///
  template<bool... Bs>
    using enable_if_any_t = std::enable_if_t
    <std::disjunction_v
     <std::conditional_t
      <Bs, std::true_type, std::false_type>...> >;

  template<bool... Bs>
  using enable_if_all_t = std::enable_if_t
    <std::conjunction_v
     <std::conditional_t
      <Bs, std::true_type, std::false_type>...> >;
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief This creates a compile-time integer sequence starting
  /// from 0 to (N-1) with `log(N)` complexity. Reference:
  /// http://stackoverflow.com/questions/17424477/implementation-
  /// c14-make-integer-sequence
  ///
  /// @param N: number of indices to generate.
  /// @return A compile-time integer sequence from 0 to (N-1).
  ///
  template<class T>
  using Invoke = typename T::type;

  template<int...>
  struct indx_seq{ using type = indx_seq; };

  template<class S1, class S2> struct concat;
  template<int... I1, int... I2>
  struct concat<indx_seq<I1...>, indx_seq<I2...> >
    : indx_seq<I1..., (sizeof...(I1)+I2)...> {};

  template<class S1, class S2>
  using Concat = Invoke<concat<S1, S2> >;

  template<int N> struct make_seq;
  template<int N> using GenSeq = Invoke<make_seq<N> >;

  template<int N>
  struct make_seq : Concat<GenSeq<N/2>,GenSeq<N-N/2> > {};

  template<> struct make_seq<0> : indx_seq<>{};
  template<> struct make_seq<1> : indx_seq<0>{};

  template<int N> using make_seq_t = typename make_seq<N>::type;
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Retrieve dimension of an input structure.
  ///
  /// Fails to compile if `dim` property is not available in the
  /// structure's type traits.
  ///
  /// @param T: an input type.
  /// @return An int indicating the dimension of the input structure.
  ///
  template<typename T, bool cond=has_dim_v<T> > struct pst_dim_t {};

  template<typename T> struct pst_dim_t<T,false>
  { static constexpr int value = -1; };

  template<typename T> struct pst_dim_t<T,true>
  { static constexpr int value = traits::specs<std::decay_t<T> >::dim; };

  template<typename T, typename ...Args>
  struct pst_dim_t<std::variant<T,Args...>,false>
  { static constexpr int value = pst_dim_t<T>::value; };

  template<typename T>
  constexpr int pst_dim_v = pst_dim_t<std::decay_t<T> >::value;

  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Retrive `RealScalar` type, compile-time nrows & compile-time
  /// ncols from input Eigen expr.
  ///
  /// Retrives `RealScalar`, `RowsAtCimpileTime`, `ColsAtCimpileTime` from
  /// an input Eigen structure.
  ///
  /// @warning Undefined for structs that are not Eigen. Call will fail in
  /// this case.
  ///
  /// @param ET: an input structure type for the test.
  /// @return
  /// - The `RealScalar` for `eigen_val_t`.
  /// - The `RowsAtCimpileTime` for `eigen_rows_v`.
  /// - The `ColsAtCimpileTime` for `eigen_cols_v`.
  ///
  template<typename ET> using eigen_val_t =
    typename std::decay_t<ET>::RealScalar;

  template<typename ET, typename = std::enable_if_t<is_eigen_v<ET> > >
  constexpr int eigen_rows_v = std::decay_t<ET>::RowsAtCompileTime;

  template<typename ET, typename = std::enable_if_t<is_eigen_v<ET> > >
  constexpr int eigen_cols_v = std::decay_t<ET>::ColsAtCompileTime;
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check if an input struct is Eigen colmajored: return false if
  /// not Eigen.
  ///
  /// @param ET: an input structure type for the test.
  /// @return True if the input type is an Eigen column-majored struct,
  /// false otherwise. Returns false if the structure is not an Eigen type.
  ///
  template<typename ET, std::enable_if_t<is_eigen_v<ET> >* = nullptr>
  constexpr bool eigen_colmajor()
  {
    return  !std::decay_t<ET>::IsRowMajor;
  }

  template<typename ET, std::enable_if_t<!is_eigen_v<ET> >* = nullptr>
  constexpr bool eigen_colmajor()
  {
    return  false;
  }

  template<typename ET>
  constexpr bool eigen_colmajor_v = eigen_colmajor<std::decay_t<ET> >();
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check size of an input structure matches the given value if
  /// it is Eigen. Return false if not Eigen.
  ///
  /// @param ET: an input structure type for the test.
  /// @param Rows: the input number of rows for the test.
  /// @param Rows: the input number of cols for the test.
  /// @return True if the input type is an Eigen struct, with
  /// `RowsAtCompileTime` == Rows && `ColsAtCompileTime` == Cols. Returns
  /// false if the structure is not an Eigen type.
  ///
  template<typename ET, int Rows, int Cols,
           std::enable_if_t<is_eigen_v<ET> >* = nullptr>
  constexpr bool eigen_size_is()
  {
    return (eigen_rows_v<ET> == Rows) && (eigen_cols_v<ET> == Cols);
  }

  template<typename ET, int Rows, int Cols,
           std::enable_if_t<!is_eigen_v<ET> >* = nullptr>
  constexpr bool eigen_size_is()
  {
    return false;
  }

  template<typename ET, int Rows, int Cols>
  constexpr bool eigen_size_is_v =
    eigen_size_is<std::decay_t<ET>,Rows,Cols>();
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check if an input structure is an Eigen dynamic row vector.
  /// Return false if not Eigen.
  ///
  /// @param ET: an input structure type for the test.
  /// @return True if the input type is an Eigen dynamic row vector, i.e.
  /// `RowsAtCompileTime` == 1 && `ColsAtCompileTime` == -1, Returns false if
  /// the structure is not an Eigen type.
  ///
  template<typename ET, std::enable_if_t<is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_dynamic_row_vec()
  {
    return (eigen_rows_v<ET> == 1) && (eigen_cols_v<ET> == -1);
  }

  template<typename ET, std::enable_if_t<!is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_dynamic_row_vec()
  {
    return false;
  }

  template<typename ET>
  constexpr bool is_eigen_dynamic_row_vec_v =
    is_eigen_dynamic_row_vec<std::decay_t<ET> >();
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check if an input structure is an Eigen dynamic row vector.
  /// Return false if not Eigen.
  ///
  /// @param ET: an input structure type for the test.
  /// @return True if the input type is an Eigen dynamic col vector, i.e.
  /// `ColsAtCompileTime` == 1 && `RowsAtCompileTime` == -1, Returns false if
  /// the structure is not an Eigen type.
  ///
  template<typename ET, std::enable_if_t<is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_dynamic_col_vec()
  {
    return (eigen_rows_v<ET> == -1) && (eigen_cols_v<ET> == 1);
  }

  template<typename ET, std::enable_if_t<!is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_dynamic_col_vec()
  {
    return false;
  }

  template<typename ET>
  constexpr bool is_eigen_dynamic_col_vec_v =
    is_eigen_dynamic_col_vec<std::decay_t<ET> >();
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check size of an input structure is an Eigen dynamic vector.
  /// Return false if not Eigen.
  ///
  /// @param ET: an input structure type for the test.
  /// @return True if the input type is an Eigen dynamic vector (row or col
  /// ordered), i.e. `RowsAtCompileTime` == 1 && `ColsAtCompileTime` == -1,
  /// or `RowsAtCompileTime` == -1 && `ColsAtCompileTime` == 1. Returns false
  /// if the structure is not an Eigen type.
  ///
  template<typename ET, std::enable_if_t<is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_dynamic_vec()
  {
    return is_eigen_dynamic_row_vec_v<ET> ||
      is_eigen_dynamic_col_vec_v<ET>;
  }

  template<typename ET, std::enable_if_t<!is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_dynamic_vec()
  {
    return false;
  }

  template<typename ET>
  constexpr bool is_eigen_dynamic_vec_v =
    is_eigen_dynamic_vec<std::decay_t<ET> >();
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check size of an input structure is an Eigen dynamic matrix.
  /// Return false if not Eigen.
  ///
  /// @param ET: an input structure type for the test.
  /// @return True if the input type is an Eigen dynamic matrix (row or col
  /// ordered), i.e. `RowsAtCompileTime` > 1 && `ColsAtCompileTime` == -1,
  /// or `RowsAtCompileTime` == -1 && `ColsAtCompileTime` > 1, or
  /// `RowsAtCompileTime` == -1 && `ColsAtCompileTime` == -1. Returns false
  /// if the structure is not an Eigen type.
  ///
  template<typename ET, std::enable_if_t<is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_dynamic_mat()
  {
    return ( (eigen_rows_v<ET> > 1) && (eigen_cols_v<ET> == -1) ) ||
      ( (eigen_rows_v<ET> == -1) && (eigen_cols_v<ET> > 1) ) ||
      ( (eigen_rows_v<ET> == -1) && (eigen_cols_v<ET> == -1) );
  }

  template<typename ET, std::enable_if_t<!is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_dynamic_mat()
  {
    return false;
  }

  template<typename ET>
  constexpr bool is_eigen_dynamic_mat_v =
    is_eigen_dynamic_mat<std::decay_t<ET> >();
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check if an input struct is Eigen fixed size: return false
  /// if not Eigen.
  ///
  /// @param ET: an input structure type for the test.
  /// @return True if the input type is an Eigen fix sized vector / matrix:
  /// i.e. both Cols and Rows are compile-time fixed constants. Returns
  /// false if the structure is not an Eigen type.
  ///
  template<typename ET, std::enable_if_t<is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_fixed_size()
  {
    return (eigen_rows_v<ET> != Eigen::Dynamic) &&
      (eigen_cols_v<ET> != Eigen::Dynamic);
  }

  template<typename ET, std::enable_if_t<!is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_fixed_size()
  {
    return false;
  }

  template<typename ET>
  constexpr bool is_eigen_fixed_size_v =
    is_eigen_fixed_size<std::decay_t<ET> >();
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check if an input struct is Eigen fixed matrix: return false
  /// if not Eigen.
  ///
  /// @param ET: an input structure type for the test.
  /// @return True if the input type is an Eigen fix matrix: i.e. both
  /// `RowsAtCompileTime` and `ColsAtCompileTime` are > 1. Returns false
  /// if the structure is not an Eigen type.
  ///
  template<typename ET, std::enable_if_t<is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_fixed_mat()
  {
    return (eigen_rows_v<ET> > 1) && (eigen_cols_v<ET> > 1);
  }

  template<typename ET, std::enable_if_t<!is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_fixed_mat()
  {
    return false;
  }

  template<typename ET>
  constexpr bool is_eigen_fixed_mat_v =
    is_eigen_fixed_mat<std::decay_t<ET> >();
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check if an input struct is Eigen fixed col vec: return false
  /// if not Eigen.
  ///
  /// @param ET: an input structure type for the test.
  /// @return True if the input type is an Eigen fix sized col vector: i.e.
  /// both Cols and Rows are compile-time fixed constants and Cols == 1.
  /// Returns false if the structure is not an Eigen type.
  ///
  template<typename ET, std::enable_if_t<is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_fixed_col_vec()
  {
    return is_eigen_fixed_size_v<ET> && (eigen_cols_v<ET> == 1);
  }

  template<typename ET, std::enable_if_t<!is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_fixed_col_vec()
  {
    return false;
  }

  template<typename ET>
  constexpr bool is_eigen_fixed_col_vec_v =
    is_eigen_fixed_col_vec<std::decay_t<ET> >();
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check if an input struct is Eigen fixed row vec: return false
  /// if not Eigen.
  ///
  /// @param ET: an input structure type for the test.
  /// @return True if the input type is an Eigen fix sized row vector: i.e.
  /// both Cols and Rows are compile-time fixed constants and Rows == 1.
  /// Returns false if the structure is not an Eigen type.
  ///
  template<typename ET, std::enable_if_t<is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_fixed_row_vec()
  {
    return is_eigen_fixed_size_v<ET> && (eigen_rows_v<ET> == 1);
  }

  template<typename ET, std::enable_if_t<!is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_fixed_row_vec()
  {
    return false;
  }

  template<typename ET>
  constexpr bool is_eigen_fixed_row_vec_v =
    is_eigen_fixed_row_vec<std::decay_t<ET> >();
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check if an input struct is Eigen fixed vec of any ordering:
  /// return false if not Eigen.
  ///
  /// @param ET: an input structure type for the test.
  /// @return True if the input type is an Eigen fix sized vector: i.e. both
  /// Cols and Rows are compile-time fixed constants and one of Rows / Cols
  /// is 1. Returns false if the structure is not an Eigen type.
  ///
  template<typename ET, std::enable_if_t<is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_fixed_vec()
  {
    return is_eigen_fixed_row_vec_v<ET> || is_eigen_fixed_col_vec_v<ET>;
  }

  template<typename ET, std::enable_if_t<!is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_fixed_vec() {
    return false;
  }

  template<typename ET>
  constexpr bool is_eigen_fixed_vec_v =
    is_eigen_fixed_vec<std::decay_t<ET> >();
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check if an input struct is Eigen fix sized or dynamic matrix:
  /// return false if not Eigen.
  ///
  /// @param ET: an input structure type for the test.
  /// @return True if the input type is an Eigen fix sized or dynamic matrix
  /// Returns false if the structure is not an Eigen type.
  ///
  template<typename ET, std::enable_if_t<is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_mat()
  {
    return is_eigen_fixed_mat_v<ET> || is_eigen_dynamic_mat_v<ET>;
  }

  template<typename ET, std::enable_if_t<!is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_mat()
  {
    return false;
  }

  template<typename ET>
  constexpr bool is_eigen_mat_v = is_eigen_mat<std::decay_t<ET> >();
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check if an input structure is an Eigen row vector, dynamic or
  /// fixed. Return false if not Eigen.
  ///
  /// @param ET: an input structure type for the test.
  /// @return True if the input type is an Eigen row vector, dynamic or fixed.
  /// Returns false if the structure is not an Eigen type.
  ///
  template<typename ET, std::enable_if_t<is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_row_vec()
  {
    return is_eigen_dynamic_row_vec_v<ET> ||
      is_eigen_fixed_row_vec_v<ET>;
  }

  template<typename ET, std::enable_if_t<!is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_row_vec()
  {
    return false;
  }

  template<typename ET>
  constexpr bool is_eigen_row_vec_v = is_eigen_row_vec<std::decay_t<ET> >();
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check if an input structure is an Eigen row vector dynamic or
  /// fixed. Return false if not Eigen.
  ///
  /// @param ET: an input structure type for the test.
  /// @return True if the input type is an Eigen col vector, dynamic or fixed
  /// Returns false if the structure is not an Eigen type.
  ///
  template<typename ET, std::enable_if_t<is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_col_vec()
  {
    return is_eigen_dynamic_col_vec_v<ET>() ||
      is_eigen_fixed_col_vec_v<ET>();
  }

  template<typename ET, std::enable_if_t<!is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_col_vec()
  {
    return false;
  }

  template<typename ET>
  constexpr bool is_eigen_col_vec_v = is_eigen_col_vec<std::decay_t<ET> >();
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check if an input struct is Eigen fix sized or dynamic vector:
  /// return false if not Eigen.
  ///
  /// @param ET: an input structure type for the test.
  /// @return True if the input type is an Eigen fix sized or dynamic vector
  /// Returns false if the structure is not an Eigen type.
  ///
  template<typename ET, std::enable_if_t<is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_vec()
  {
    return is_eigen_fixed_vec_v<ET> || is_eigen_dynamic_vec_v<ET>;
  }

  template<typename ET, std::enable_if_t<!is_eigen_v<ET> >* = nullptr>
  constexpr bool is_eigen_vec()
  {
    return false;
  }

  template<typename ET>
  constexpr bool is_eigen_vec_v = is_eigen_vec<std::decay_t<ET> >();
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Retrieve & mutate fields of Eigen structures.
  ///
  /// @param M: input Eigen structure.
  /// @param Row: integer number of rows.
  /// @param Col: integer number of cols.
  /// @return
  /// - `eigen_mat_t` is used to retrieve the `Matrix` type structure from
  ///   a valid Eigen expression.
  /// - `mutate_row_t` and `mutate_col_t` are used to alter in compile-time
  ///   the `RowsAtCompileTime` and `ColsAtCompileTime` of an Eigen Matrix
  ///   structure. **It need to be used with `Matrix` types, not any
  ///   expression**. The recommanded way of use it to **wrap an expression
  ///   with `eigen_mat_t`**, then pass the result to `mutate_row_t` and
  ///   `mutate_col_t`.
  /// - `mutate_val_t` is defined for any Eigen expressions and also for STL
  ///   `vector`s. It is not required that an Eigen `Matrix` is passed, since
  ///   internally it will wrap the given expression to a `Matrix` type.
  ///
  template<typename M, bool cond=is_eigen_v<M> > struct __to_mat {};
  template<typename M> struct __to_mat<M,true>
  { using type = Eigen::Matrix<eigen_val_t<M>,eigen_rows_v<M>,
                               eigen_cols_v<M> >; };
  template<typename M> struct __to_mat<M,false> { using type = void; };
  template<typename M> using eigen_mat_t =
    typename __to_mat<std::decay_t<M> >::type;

  // Alter scalr_type/nrows/ncols of an Eigen expression.
  template<typename M, typename T> struct __mutate_val { using type = void; };
  template<typename M, int Row> struct __mutate_row { using type = void; };
  template<typename M, int Col> struct __mutate_col { using type = void; };

  template<typename T0, typename T, int Row, int Col>
  struct __mutate_val<Eigen::Matrix<T0,Row,Col>,T>
  { using type = Eigen::Matrix<T,Row,Col>; };
  template<typename T, int Row0, int Row, int Col>
  struct __mutate_row<Eigen::Matrix<T,Row0,Col>,Row>
  { using type = Eigen::Matrix<T,Row,Col>; };
  template<typename T, int Row, int Col0, int Col>
  struct __mutate_col<Eigen::Matrix<T,Row,Col0>,Col>
  { using type = Eigen::Matrix<T,Row,Col>; };

  template<typename M, int Row>
  using mutate_row_t = typename __mutate_row<eigen_mat_t<M>,Row>::type;
  template<typename M, int Col>
  using mutate_col_t = typename __mutate_col<eigen_mat_t<M>,Col>::type;

  // Value mutator is also specialized for STL::vector.
  template<typename M, typename T, bool cond=is_eigen_v<M> >
  struct mutate_val {};
  template<typename M, typename T> struct mutate_val<M,T,true>
  { using type = typename __mutate_val<eigen_mat_t<M>,T>::type; };
  template<typename T0, typename T, typename Alloc>
  struct mutate_val<std::vector<T0,Alloc>,T,false>
  { using type = std::vector<T>; };
  template<typename M, typename T> struct mutate_val<M,T,false>
  { using type = void; };

  template<typename M, typename T>
  using mutate_val_t = typename mutate_val<M,T>::type;
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Rotation matrix size dispatch.
  ///
  /// @warning **N** is not dimension, but **the number of rotation angles.**
  ///
  /// Dimension 1 will be dispatched to an Eigen fixed matrix of size 2 x 2.
  /// Dimension 3 will be dispatched to an Eigen fixed matrix of size 3 x 3.
  /// @param Val: type of scalar used for rotation matrix.
  /// @param N: number of rotation angles.
  /// @return The dispatched fix-sized rotation matrix.
  ///
  template<typename Val,int N>
  struct __rot_dispatcher { using type = void; };
  template<typename Val> struct __rot_dispatcher<Val,1>
  { using type = Eigen::Matrix<Val,2,2>; };
  template<typename Val> struct __rot_dispatcher<Val,3>
  { using type = Eigen::Matrix<Val,3,3>; };

  template<typename Val, bool cond=is_eigen_v<Val> >
  struct rotmat_dispatch {};
  template<typename Val> struct rotmat_dispatch<Val,true>
  {
    using type = typename __rot_dispatcher
      <eigen_val_t<Val>,eigen_rows_v<Val> >::type;
  };

  template<typename Val>
  struct rotmat_dispatch<Val,false> { using type = void; };
  template<typename Val>
  using rotmat_dispatch_t = typename rotmat_dispatch<Val>::type;
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Value type dispatch for STL containers & Eigen.
  ///
  /// @param Struct: an input structure type from Eigen or STL.
  /// @param cond: not required, for dispatch purpose.
  /// @return The data type of elements inside the input structure.
  ///
  template<typename Struct, bool cond=is_eigen_v<Struct> >
  struct value_dispatch {};
  template<typename Struct> struct value_dispatch<Struct, true>
  { using type = eigen_val_t<Struct>; };
  template<typename Struct> struct value_dispatch<Struct, false>
  { using type = typename Struct::value_type; };

  template<typename Struct>
  using value_dispatch_t =
    typename value_dispatch<std::decay_t<Struct> >::type;
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Accessor dispatch for Eigen and STL containers.
  ///
  /// Generic interface wrapping both operator () for Eigen structures
  /// and operator [] for STL container (`std::vector` notably).
  ///
  /// @param cont: the input container or Eigen structure whose value
  /// will be accessed.
  /// @param args: integeral index / indices.
  /// @return Const reference to value in the container or Eigen struct
  /// indexed by the input index / indices.
  ///
  struct value_at_t
  {
    constexpr value_at_t() {};

    template<typename Struct, typename ...Args,
             std::enable_if_t<is_eigen_v<Struct> >* = nullptr>
    auto operator()(Struct &&cont, Args ...args)
      const -> decltype(cont(args...))
    {
      return std::forward<Struct>(cont)(args...);
    }

    template<typename Struct, std::enable_if_t
             <!is_eigen_v<Struct> >* = nullptr>
    auto operator()(Struct &&cont, std::size_t arg)
      const -> decltype(cont[arg])
    {
      return std::forward<Struct>(cont)[arg];
    }
  };
  constexpr value_at_t value_at{};
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Dispatch for kernel smooth estimator band-width
  ///
  /// When Dim == 1, it dispatches to a scalar. When Dim > 1, it dispatches
  /// to an Eigen fixed matrix of size Dim x Dim.
  ///
  /// @param Scalar: the input scalar type for floating-point computations.
  /// @param Dim: the input dimension.
  ///
  template<typename Scalar, int Dim> struct bandwidth_dispatch
  { using type = Eigen::Matrix<Scalar,Dim,Dim>; };

  template<typename Scalar> struct bandwidth_dispatch<Scalar,1>
  { using type = Scalar; };

  template<typename Scalar, int Dim>
  using bandwidth_dispatch_t = typename bandwidth_dispatch<Scalar,Dim>::type;
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Check if an input type's ID matches the argument.
  ///
  /// @param T: an input type.
  /// @param id: an input `pst_id_list` enum object.
  /// @return True if the ID of `T` matches `id`.
  ///
  /// @sa `pasta::pst_id_list`
  ///
  template<typename T, pst_id_list id>
  constexpr bool pst_id_is_v = (traits::specs<std::decay_t<T> >::pst_id == id);
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Detect if an input type is a pasta distribution.
  ///
  /// @param T: the input type.
  /// @return True when the input type is a pasta distribution.
  ///
  template<typename T>
  constexpr bool is_pst_distr_v =  pst_id_is_v<T,pst_id_list::PASTA_RANDOM_VAR>;


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Detect if an input type is a pasta symmetric distribution.
  ///
  /// @param T: the input type.
  /// @return True is the input type is a pasta symmetric distribution.
  ///
  template<typename T>
  constexpr bool is_symmetric_distr_v =
    (is_pst_distr_v<T> && traits::specs<std::decay_t<T> >::is_symmetric);


  ///@{
  ///
  /// @brief Check if an input type is a std::variant of an input
  /// `pasta` ID.
  ///
  /// @note A `std::variant` containing **at least one**
  /// matched type **dispatches also to true**. This is because
  /// old `std::variant` implementations append `void_` types
  /// after std::variant's argument list, **if the AND logical is
  /// used, a std::variant will never dispatch to true
  /// even if all std::variant arguments match the input ID.
  ///
  /// @param T: the input type.
  /// @param id: a `pasta` ID.
  ///
  /// @sa `pst_id_list`.
  ///
  template<typename T, pst_id_list id> struct is_variant_of
  { static const bool value = false; };

  template<typename ...Args, pst_id_list id>
  struct is_variant_of<std::variant<Args...>, id>
  {
    // A std::variant containing at least one matched ID is
    // considered true.
    static const bool value =
      std::disjunction_v<(pst_id_is_v<Args,id>)...>();
  };

  template<typename T, pst_id_list id>
  constexpr bool is_variant_of_v = is_variant_of<std::decay_t<T>,id>::value;
  ///@}


  /// @ingroup group_meta
  ///
  /// @brief Detect if an input type is a pasta kernel.
  ///
  /// @param T: the input type.
  /// @return True is the input type is a pasta kernel.
  ///
  template<typename T>
  constexpr bool is_pst_kernel_v = pst_id_is_v<T,pst_id_list::PASTA_KERNEL>;


  /// @ingroup group_meta
  ///
  /// @brief Detect if an input type is a pasta domain.
  ///
  /// @param T: the input type.
  /// @return True is the input type is a pasta domain.
  ///
  template<typename T>
  constexpr bool is_pst_domain_v = pst_id_is_v<T,pst_id_list::PASTA_DOMAIN>;


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Detect if an input type is an STL vector.
  ///
  /// @param T: the input type.
  /// @return True is the input type is an STL vector.
  ///
  template<typename T>
  struct is_stl_vector_t: std::false_type {};

  template<typename ...Args>
  struct is_stl_vector_t<std::vector<Args...> >: std::true_type {};

  template<typename T>
  constexpr bool is_stl_vector_v = is_stl_vector_t<std::decay_t<T> >::value;
  ///@}


  /// @ingroup group_meta
  ///
  /// @brief Detect if an input type is an STL vector with specific element
  /// type and allocator type.
  ///
  /// @param Struct: type of the input structure.
  /// @param Element: the element's type, i.e. `value_type` of the vector.
  /// @param Allocator: the allocator's type. Defaults to the standard
  /// `allocator_type` of `std::vector`.
  /// @return True is the input type is an STL vector with specific element
  /// type and allocator type.
  ///
  template<typename Struct, typename Element,
           typename Allocator = typename std::vector<Element>::allocator_type>
  constexpr bool is_stl_vector_of_v =
    std::is_same_v<std::decay_t<Struct>, std::vector<Element,Allocator> >;


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Dispatch for scalar value type based on input.
  ///
  /// When the input is an Eigen structure, it dispatches to the
  /// `RealScalar` typedef. When the input is an arithmetic, it dispatches
  /// to the type of the input itself.
  ///
  /// @param T: an input type.
  /// @return The dispatched scalar type.
  ///
  template<typename MT,
           bool cond1=is_eigen_v<MT>, bool cond2=std::is_arithmetic_v<MT> >
  struct scalar_dispatch { using type = void; };

  template<typename MT> struct scalar_dispatch<MT,true,false>
  { using type = eigen_val_t<MT>; };

  template<typename MT> struct scalar_dispatch<MT,false,true>
  { using type = MT; };

  template<typename T>
  using scalar_dispatch_t = typename scalar_dispatch<std::decay_t<T> >::type;
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Dispatch for point type depending on input dimension &
  /// scalar type.
  ///
  /// When Dim == 1, the point type is simply the input scalar type.
  /// When Dim > 1, the point type is an Eigen column vector of size Dim
  /// x 1, with input `Scalar` type.
  ///
  /// @param Scalar: input scalar type.
  /// @param Dim: the dimension.
  /// @return The dispatch point type.
  ///
  template<typename Scalar, int Dim> struct point_dispatch
  { using type = Eigen::Matrix<Scalar,Dim,1>; };
  template<typename Scalar> struct point_dispatch<Scalar,1>
  { using type = Scalar; };

  template<typename Scalar, int Dim>
  using point_dispatch_t = typename point_dispatch<Scalar,Dim>::type;
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Dispatch for Eigen::aligned_allocator.
  ///
  /// This function retrieves the appropriate allocator type for an input
  /// structure. The rationale is, for any structures containing **fix sized
  /// Eigen types**, the usage with **STL containers** need to be passed
  /// alone with `Eigen::aligned_allocator` for alignment reasons.
  ///
  /// @param T: input type for the test.
  /// @param cond: defaulted, for specialzation purpose.
  /// @return `Eigen::aligned_allocator<T>` if
  /// 1. T has the traits `need_align` set to `true`.
  /// 2. Or T is an Eigen fix sized structure itself.
  ///
  template<typename T, bool cond = is_eigen_fixed_size_v<T> >
  struct alloc_dispatch
    : public std::conditional<cond, Eigen::aligned_allocator<std::decay_t<T> >,
                              std::allocator<std::decay_t<T> > > {};

  template<typename T> using alloc_dispatch_t =
    typename alloc_dispatch<std::decay_t<T> >::type;
  ///@}


  ///@(
  /// @brief Dispatch for **structural dimension** on input structure type.
  ///
  /// The **structural dimension** is the dimension of the domain represented
  /// by an Eigen structure. For example, an Eigen matrix of size 3 x Dynamic
  /// has structural dimension 3, each column represents a 3-dimensional elem.
  ///
  /// @note The following rules are used for dimension dispatch:
  /// 1. If a scalar type is given, then it leads to 1.
  /// 2. If an Eigen **dynamic** row or col vector is given, it leads to 1.
  /// This means a dynamic vector contains an **ensemble of 1D elements**.
  /// 3. If an Eigen **fix sized** col or row vector is used, it leads to
  /// `RowsAtCompileTime`. For example, a vector of size 3 x 1 represents a 3
  /// dimensional element. A vector of size 1 x 3, represents **three
  /// 1-dimensional elements**.
  /// 4. If an Eigen Matrix, whether fixed or dynamic is given, it leads to
  /// `RowsAtCompileTime`. This means that the matrix **show be column ordered**,
  /// each column represnts a Dim-dimensional element.
  /// 5. If an `std::vector` is given, returns 1. `std::vector` has the same
  /// structural dimensional as a dynamic Eigen vector.
  ///
  /// @param T: the input structure type.
  /// @return The structure dimension.
  ///
  template<typename T, std::enable_if_t<is_stl_vector_v<T> >* = nullptr>
  constexpr int dim_dispatch() { return 1; }

  template<typename T, std::enable_if_t<std::is_arithmetic_v<T> >* = nullptr>
  constexpr int dim_dispatch() { return 1; }

  template<typename T, std::enable_if_t<is_eigen_dynamic_vec_v<T> >* = nullptr>
  constexpr int dim_dispatch() { return 1; }

  template<typename T, std::enable_if_t<is_eigen_fixed_vec_v<T> >* = nullptr>
  constexpr int dim_dispatch() { return eigen_rows_v<T>; }

  template<typename T, std::enable_if_t<is_eigen_mat_v<T> >* = nullptr>
  constexpr int dim_dispatch() { return eigen_rows_v<T>; }

  template<typename T>
  constexpr int dim_dispatch_v = dim_dispatch<std::decay_t<T> >();
  ///@}


  ///@{
  /// @brief Dispatch for STL container and Eigen matrix.
  ///
  /// When the input is an Eigen expression, it dispatches to the an Eigen
  /// matrix transformed from the input expression using corresponding nRows,
  /// nCols and scalar type. When the input is an STL container, it dispatches
  /// to the input type itself.
  ///
  /// @warning No constraint is enforced on the input type being an STL
  /// container. So any non container, non Eigen type will also dispatch to
  /// itself.
  ///
  /// @param T: the input type.
  /// @return The dispatched container type.
  ///
  template<typename T, bool cond = is_eigen_v<T> >
  struct container_dispatch: public std::conditional<cond,eigen_mat_t<T>,T> {};

  template<typename T> using container_dispatch_t =
    typename container_dispatch<std::decay_t<T> >::type;
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Dispatch for bounding box types on input Eigen structure.
  ///
  /// @note The following rules are used:
  /// 1. When a matrix type is passed. It assumes that it is **column
  /// ordered**, meaning that each column represents a point element. The
  /// bounding box type is for column elements in this case. And it will
  /// dispatch to an Eigen matrix of size `RowsAtCompileTime` x 2.
  /// 2. When a **dynamic vector** type is passed, it **does not** assume
  /// the order of elements, meaning that either a column or a row vector
  /// can be passed, and they will both be dispatched as Eigen point with
  /// size 1 x 2. **It is the only exception when order does no matter.**
  /// 3. When a **fix sized row vector** type is passed, it acts the same
  /// as for matrices. For example, a fix sized vector of size 1 x 10 will
  /// dispatch to a bound of size 1 x 2, since it is considered as 10 one-
  /// dimensional elements.
  /// 4. When a **fix sized col vector** type is passed, it dispatches to
  /// `void`. That's because a fix sized col vector represents one single
  /// element, and bounding box for a single vector is not well defined.
  ///
  /// @param MT: an input structure type for the test.
  /// @param cond1: defaulted, for specialization purpose.
  /// @param cond2: defaulted, for specialization purpose.
  /// @return The dispatched bounding_box type:
  /// 1. When it's a matrix: `RowsAtCompileTime` x 2.
  /// 2. When it's a dynamic vector: Eigen 1 x 2 point.
  /// 3. When it's a fix sized vector: `void`.
  ///
  template<typename MT,
           bool cond0=is_eigen_v<MT>,
           bool cond1=is_eigen_fixed_col_vec_v<MT>,
           bool cond2=is_eigen_dynamic_vec_v<MT> >
  struct bound_dispatch {};

  // For non-Eigen structures
  template<typename MT, bool cond1, bool cond2>
  struct bound_dispatch<MT,false,cond1,cond2> { using type = void; };

  // For fixed size col vectors.
  template<typename MT, bool cond> struct bound_dispatch<MT,true,true,cond>
  { using type = void; };

  // This case is for dynamic vectors.
  template<typename MT> struct bound_dispatch<MT,true,false,true>
  { using type = Eigen::Matrix<eigen_val_t<MT>,1,2>; };

  // This case is for Matrices or fix sized row vectors.
  template<typename MT> struct bound_dispatch<MT,true,false,false>
  { using type = Eigen::Matrix<eigen_val_t<MT>,eigen_rows_v<MT>,2>; };

  template<typename MT>
  using bound_dispatch_t = typename bound_dispatch<std::decay_t<MT> >::type;
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Constant expression functional object to test
  /// if an arithmetic value is 0.
  ///
  /// @param __val: the input arithmetic value.
  /// @return True if __val is zero.
  ///
  struct is_zero_t
  {
    constexpr is_zero_t() {};
    template<typename T> constexpr bool operator()(T __val) const
    { return !__val; }
  };
  constexpr is_zero_t is_zero{};
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Constant expression functional object to test
  /// if an arithmetic value is not 0.
  ///
  /// @param __val: the input arithmetic value.
  /// @return True if __val is not zero.
  ///
  struct non_zero_t
  {
    constexpr non_zero_t() {};
    template<typename T> constexpr bool operator()(T __val) const
    { return __val; }
  };
  constexpr non_zero_t non_zero{};
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Constant expression functional object
  /// returning `true` for anything.
  ///
  /// @param: an input.
  /// @return `true`.
  ///
  struct anything_t
  {
    constexpr anything_t() {};
    template<typename T> constexpr bool operator()(T) { return true; }
  };
  constexpr anything_t anything{};
  ///@}


  ///@{
  /// @ingroup group_meta
  ///
  /// @brief Constant expression functional object
  /// returning `false` for anything.
  ///
  /// @param: an input.
  /// @return `false`.
  ///
  struct nothing_t
  {
    constexpr nothing_t() {};
    template<typename T> constexpr bool operator()(T) { return false; }
  };
  constexpr nothing_t nothing{};
  ///@}

}


#endif
