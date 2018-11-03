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
// File: DiscreteDom.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 22:22:45 2018 Zhijin Li
// Last update Sat Nov  3 21:06:39 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_DISCRETEDOM_HH
# define PASTA_DISCRETEDOM_HH

# include "pasta/utilities.hh"


namespace pasta
{
  namespace dom
  {

    /// @defgroup group_domain Domain of distributions and sampling
    ///
    /// @brief Defines the domain of distributions and sampling
    /// methods:
    /// - Discrete domains: lattice grids and discretized geometric
    ///   shapes.
    /// - Continuous domains: geometric shapes and functional
    ///   domains.
    ///

    template<typename Scalar, int Dim> class DiscreteDom;
  }

  /// @ingroup group_traits
  namespace traits
  {
    /// @ingroup group_traits
    ///
    /// @brief Type traits properties for the
    /// `pasta::stats::DiscreteDom<Scalar,Dim,false>` class.
    ///
    template<typename Scalar, int Dim>
    struct specs<dom::DiscreteDom<Scalar,Dim> >
    {
      static constexpr int dim =                              Dim;
      static const pst_id_list pst_id = pst_id_list::PASTA_DOMAIN;
      typedef Scalar                                      scalr_t;
      typedef point_dispatch_t<Scalar,Dim>                locat_t;
      typedef Eigen::Matrix<Scalar,Dim,2>                 bound_t;
      typedef point_dispatch_t<int,Dim>                   count_t;
    };

  }


  namespace dom
  {

    /// @ingroup group_domain
    ///
    /// @brief Class for discrete domain.
    ///
    /// @param Scalar: type used as scalar for floating-point
    /// computations.
    /// @param Dim: an integer indicating dimension of the
    /// domain.
    ///
    template<typename Scalar, int Dim> class DiscreteDom
    {
    public:

      using exact_t =   DiscreteDom<Scalar,Dim>;
      using specs_t =    traits::specs<exact_t>;

      using scalr_t = typename specs_t::scalr_t;
      using locat_t = typename specs_t::locat_t;
      using bound_t = typename specs_t::bound_t;
      using count_t = typename specs_t::count_t;

      /// @brief Default constructor.
      DiscreteDom() = default;

      /// @brief Construct a discrete domain. With bounding box & floating
      /// point step sizes.
      ///
      /// @param bound: the input bounding box. An Eigen Dim x 2 matrix, with
      /// 1st column the lower-bound and 2nd column upper-bound of the system.
      /// @param steps: size of the step in each dimensional direction when
      /// doing domain traversal. When Dim == 1, it is a scalar. When Dim
      /// >= 2, it is an Eigen Dim x 1 point.
      ///
      template<typename Bound, typename Steps,
               std::enable_if_t<std::is_floating_point_v
                           <scalar_dispatch_t<Steps> > >* = nullptr>
      DiscreteDom(Bound &&bound, Steps &&steps):
        _bound_box(std::forward<Bound>(bound)),
        _step_size(std::forward<Steps>(steps)),
        _side_size(utils::comp_bound_discrete_size(_bound_box,_step_size))
      {};

      ///@{
      /// @brief Construct a discrete domain. With bounding box & floating
      /// point step sizes.
      ///
      /// @param bound: the input bounding box. An Eigen Dim x 2 matrix, with
      /// 1st column the lower-bound and 2nd column upper-bound of the system.
      /// @param sizes: integral size of the bounding box. When Dim == 1, it
      /// is an integer. When Dim >= 2, it is an Eigen Dim x 1 int point.
      ///
      template<typename Bound, typename Size,
               enable_if_all_t<Dim==1, std::is_integral_v<Size> >* = nullptr>
      DiscreteDom(Bound &&bound, Size size):
        _bound_box( std::forward<Bound>(bound) ),
        _step_size( (_bound_box(1)-_bound_box(0))/(size-1) ),
        _side_size( size ) {};

      template<typename Bound, typename Sizes,
               enable_if_all_t<Dim!=1, std::is_integral_v
                               <scalar_dispatch_t<Sizes> > >* = nullptr>
      DiscreteDom(Bound &&bound, Sizes &&sizes):
        _bound_box( std::forward<Bound>(bound) ),
        _step_size( (_bound_box.col(1)-_bound_box.col(0))/
                    ((sizes-1).template cast<Scalar>()) ),
        _side_size( std::forward<Sizes>(sizes) ) {};
      ///@}

      /// @brief Return const ref to the current traversal point position.
      ///
      /// @return The position (in physical length unit) of the traversal pt.
      ///
      const point_dispatch_t<Scalar,Dim>& pos() const
      { return _currn_pos; }

      ///@{
      /// @brief Get cardinality of the discrete domain: i.e. number of
      /// location counts.
      ///
      /// @return The cardinality.
      ///
      template<int __dummy=Dim>
      std::enable_if_t<__dummy==1,int> card() const
      { return _side_size; }

      template<int __dummy=Dim>
      std::enable_if_t<__dummy!=1,int> card() const
      { return _side_size.prod(); }
      ///@}

      ///@{
      /// @brief Get the elementary volume of the discrete domain: i.e.
      /// volume of each small rectangle grid.
      ///
      /// @return The elementary volume.
      ///
      template<int __dummy=Dim>
      std::enable_if_t<__dummy==1,Scalar> elem_volume() const
      { return _step_size; }

      template<int __dummy=Dim>
      std::enable_if_t<__dummy!=1,Scalar> elem_volume() const
      { return _step_size.prod(); }
      ///@}

      /// @brief Const access the step-sizes of the discrete domain.
      ///
      /// @return Const reference to the step size vec.
      ///
      const locat_t& step_size() const { return _step_size; }

      /// @brief Const access the discrete-sizes of the discrete domain.
      ///
      /// @return Const reference to the side counts.
      ///
      const locat_t& discrete_size() const { return _side_size; }

      /// @brief Const access the bounding box of the discrete domain.
      ///
      /// @return Const reference to the bounding box.
      ///
      const bound_t& bounding_box() const { return _bound_box; }

      /// @brief Set the step-sizes of the discrete domain.
      ///
      /// @note This is the proper way to alter the step-size, since it will
      /// internally re-evaluate the integral side counts and reset the curr
      /// traversal position to the lower-bound of the discrete domain.
      ///
      /// @param steps: the input new step-sizes.
      ///
      template<typename Steps> void set_step_size(Steps &&steps);

      /// @brief Set the bounding box of the discrete domain.
      ///
      /// @note This is the proper way to alter the bounding box, since it will
      /// internally re-evaluate the integral side counts and reset the curr
      /// traversal position to the lower-bound of the discrete domain.
      ///
      /// @param bound: the input new bounding_box.
      ///
      template<typename Bound> void set_bound(Bound &&bound);

      /// @brief Return const ref to the current traversal point coordinates.
      ///
      /// @return The coordinates (in integer) of the traversal pt.
      ///
      // const point_dispatch_t<Scalar,Dim>& coords() const
      // { return _curr_coords; }

      /// @brief Traversal all locations inside the discrete domain. Perform
      /// operation specified by the functional handler at each traversal
      /// position.
      ///
      /// @note The unity of `locat_t` is the length unit. If integer behavor
      /// is desired, make a `DiscreteDom` of `int`.
      ///
      /// @param handler: the input functional handler, could be a lambda or a
      /// functor. It is recommanded to write generic functor / lambda (if c++
      /// 14 is available), otherwise the input of functor should be const ref
      /// to locat_t.
      ///
      template<typename Func>
      void traverse(Func handler) const { __traverse<Dim,Func>(handler); };

    private:

      /// @brief This implements traversal for any loop > 1 case using
      /// recursion.
      ///
      /// This function is for internal usage. This function handles Dim > 1.
      /// @param handler: the input functional handler.
      ///
      template<int Loop, typename Func,
               enable_if_all_t<Loop!=1,Dim!=1>* = nullptr>
      void __traverse(Func handler) const;

      /// @brief This specialize recursive traversal for the first loop.
      ///
      /// This function is for internal usage. This function handles Dim > 1.
      /// @param handler: the input functional handler.
      ///
      template<int Loop, typename Func,
               enable_if_all_t<Loop==1,Dim!=1>* = nullptr>
      void __traverse(Func handler) const;

      /// @brief This specialize the traversal for Dim == 1.
      ///
      /// This function is for internal usage.
      /// @param handler: the input functional handler.
      ///
      template<int Loop, typename Func,
               enable_if_all_t<Loop==1,Dim==1>* = nullptr>
      void __traverse(Func handler) const;

      bound_t         _bound_box; //!< Physical boudning box in length units.
      locat_t         _step_size; //!< Step size of each trav step in all dir.
      count_t         _side_size; //!< Discrete counts of the discrete domain.
      mutable locat_t _currn_pos; //!< Pos in length units of current traversal.

    };

  }


  /// @ingroup group_utils
  namespace utils
  {

    /// @ingroup group_stats group_utils
    ///
    /// @brief Create a discrete domain from input data points.
    ///
    /// A `scale` factor can be applied to scale up the
    /// computed bound in each direction by a factor.
    ///
    /// @param Scalar: template argument for floating-point scalar
    /// type.
    /// @param data: the input data.
    /// - Can be a row or column orderd vector for 1D **dynamic**
    ///   case.
    /// - For Dim > 1 or for **fix sized** vector, it must be column
    ///   ordered.
    /// @param sizes: an Eigen vector.
    /// - If of integer type, it specifies the domain's discrete
    ///   sizes in each dimension.
    /// - If of floating point type, it indicates the step size in
    ///   each dimension.
    /// Provide Eigen Dim x 1 point if Dim > 1. Provide a scalar
    /// when Dim == 1.
    /// @return The created discrete domain.
    ///
    template<typename Scalar, typename Data, typename Params>
    auto make_discrete_domain(Data && data,
                              Params &&sizes,
                              double scale=1)
      -> dom::DiscreteDom<Scalar,dim_dispatch_v<Params>()>;

  } //!utils

}


# include "DiscreteDom.hxx"
#endif
