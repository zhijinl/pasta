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
// File: EmpiricalStats.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:28:38 2018 Zhijin Li
// Last update Thu Nov  8 22:16:52 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_EMPIRICAL_STATS
# define PASTA_EMPIRICAL_STATS

# include "../Utilities/utils.hh"


namespace pasta
{
  // Fwd decl.
  namespace stats { template<typename,int,typename,int> class EmpiricalStats; }

  /// @ingroup group_traits
  namespace traits
  {
    /// @ingroup group_traits
    ///
    /// @brief Type traits property for the `pasta::stats::EmpiricalStats`
    /// class.
    ///
    template<typename Scalr, int DomDim, typename Value, int ValDim>
    struct specs<stats::EmpiricalStats<Scalr,DomDim,Value,ValDim> >
    {
      static constexpr int locat_dim = DomDim;
      static constexpr int value_dim = ValDim;
      typedef Scalr                                        scalr_t;
      typedef Value                                        value_t;
      typedef Eigen::Matrix<scalr_t,DomDim,Eigen::Dynamic> evals_t;
      typedef Eigen::Matrix<value_t,ValDim,Eigen::Dynamic> stats_t;
    };
  }

  /// @ingroup group_stats
  namespace stats
  {

    /// @ingroup group_stats
    ///
    /// @brief Empirical statistics container.
    ///
    /// Wraps a matrix / vector for empirically estimated statistics.
    /// And a point matrix / vector with corresponding evaluation points.
    ///
    /// @param Scalr: data type for an evaluation point.
    /// @param Value: data type for a statistic value.
    /// @param DomDim: dimension of the support.
    /// @param ValDim: dimension of evaluated statistics.
    ///
    template<typename Scalr, int DomDim, typename Value, int ValDim>
    class EmpiricalStats
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      using exact_t = EmpiricalStats<Scalr,DomDim,Value,ValDim>;
      using specs_t = traits::specs<exact_t>;
      using scalr_t = typename specs_t::scalr_t;
      using value_t = typename specs_t::value_t;
      using evals_t = typename specs_t::evals_t;
      using stats_t = typename specs_t::stats_t;

      static constexpr int locat_dim = specs_t::locat_dim;
      static constexpr int value_dim = specs_t::value_dim;

      /// Defaulr constructor.
      EmpiricalStats() = default;

      /// Ctor.
      ///
      /// Resize the number of elements in point & value matrices.
      /// @param n_elem: the number of element to be value-initialized.
      ///
      explicit EmpiricalStats(int n_elem):
        _locations(DomDim, n_elem), _statistic(ValDim, n_elem) {}

      /// Ctor.
      ///
      /// Initialize with point & value matrix / vector.
      /// @param pt_matrix: the input point matrix.
      /// @param estimates: the input estimates (value matrix / vector).
      ///
      template<typename PtMat, typename ValMat,
               enable_if_all_t<is_eigen_v<PtMat>,
                               is_eigen_v<ValMat>,
                               eigen_rows_v<PtMat> == DomDim,
                               eigen_rows_v<ValMat> == ValDim>* = nullptr>
      EmpiricalStats(PtMat &&pt_matrix, ValMat &&estimates):
        _locations(pt_matrix), _statistic(estimates) {}

      /// @return The number of elements.
      ///
      int n_elem() const { return _locations.cols(); }

      /// Const access all points.
      ///
      /// @return Const reference to point matrix / vector.
      ///
      const evals_t& points() const { return _locations; }

      /// Non-const access all points.
      ///
      /// @return Non-const reference to point matrix / vector.
      ///
      evals_t& points() { return _locations; }

      /// Const access all estimated statistics.
      ///
      /// @return Const reference to estimates matrix / vector.
      ///
      const stats_t& stats() const { return _statistic; }

      /// Non-const access all estimates.
      ///
      /// @return Non-const reference to estimates matrix / vector.
      ///
      stats_t& stats() { return _statistic; }

      /// Const indexed-access of an evaluation point.
      ///
      /// @param index: the access index.
      /// @return Const ref to indexed location in the point matrix / vector.
      ///
      auto point_at(int index) const
        -> decltype(utils::elem_at(std::add_const_t<evals_t>(),0))
      { return utils::elem_at(_locations,index); }

      /// Non-const indexed-access of an evaluation point.
      ///
      /// @param index: the access index.
      /// @return Non-const ref to indexed location in the point matrix / vector.
      ///
      auto point_at(int index) -> decltype(utils::elem_at(evals_t(),0))
      { return utils::elem_at(_locations,index); }

      /// Const indexed-access of an estimated value.
      ///
      /// @param index: the access index.
      /// @return Const ref to estimated value in the matrix / vector.
      ///
      auto value_at(int index) const
        -> decltype(utils::elem_at(std::add_const_t<stats_t>(),0))
      { return utils::elem_at(_statistic,index); }

      /// Non-const indexed-access of an estimated value.
      ///
      /// @param index: the access index.
      /// @return Non-const ref to estimated value in the matrix / vector.
      ///
      auto value_at(int index) -> decltype(utils::elem_at(stats_t(),0))
      { return utils::elem_at(_statistic,index); }

    private:

      evals_t _locations; //<! Point matrix/vector containing eval points.
      stats_t _statistic; //<! Value matrix/vector containing estimation values.

    };

  } //!stats
} //!pasta


#endif //!PASTA_EMPIRICAL_STATS
