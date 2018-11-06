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
// File: distrbase.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:45:34 2018 Zhijin Li
// Last update Tue Nov  6 00:14:52 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_DISTRBASE_HH
# define PASTA_DISTRBASE_HH

# include "pasta/core.hh"
# include "pasta/Utilities/utils.hh"


namespace pasta
{

#ifndef PST_USE_SHARED_ENGINE
  namespace utils
  {
    namespace detail
    {
      /// @brief Create thread-safe global random device.
      ///
      /// @return A `std::random_device object`.
      ///
      std::random_device& get_rnd_dev();
    }
  }
#endif


  /// @defgroup group_stats Multivariate Statistics
  ///
  /// @brief Classes for modeling multivariate statistical distributions,
  /// sampling & empirical estimation etc.

  /// @ingroup group_stats
  namespace rnd
  {
    namespace abstract
    {

      /// @ingroup group_stats
      ///
      /// @brief The base class for all statistical distributions.
      ///
      /// @param EXACT: type of derived class inheriting it.
      /// @param Dim: the dimension.
      ///
      template<typename EXACT>
      class distrbase: public internal__::root__<EXACT>
      {
      private:

        using internal__::   root__<EXACT>::exact;
        using specs_t =      traits::specs<EXACT>;
        using scalr_t = typename specs_t::scalr_t;
        using value_t = typename specs_t::value_t;
        using locat_t = typename specs_t::locat_t;
        using param_t = typename specs_t::param_t;

      public:

        /// @brief Draw a single value.
        ///
        /// @return One sample following the current distribution.
        ///
        value_t draw() const;

        /// @brief Independently draw vals into en Eigen structure.
        ///
        /// @param structure: the input Eigen structure.
        ///
        template<typename MT>
        void draw(MT &&structure) const;

        /// @brief Independently draw vals into en Eigen structure.
        ///
        /// Need to explicitly specify the output Eigen Matrix type.
        /// @param: optional rows of the output matrix. Has no effect
        /// if the output is fixed size.
        /// @param: optional cols of the output matrix. Has no effect
        /// if the output is fixed size.
        /// @return Output mat/vec filled with random number from distr.
        ///
        template<typename Matrix> Matrix draw(int rows=0, int cols=0) const;

        /// @brief Reset state of the random distribuition: so that
        /// next call to draw() is independent from the previous one.
        ///
        /// @note For usual cases of simulations, you barely need
        /// to use this function. It is **different from reseting the
        /// random seed**, which is managed by the function
        /// `pasta::utils::reset_shared_engine()`. In one stream of
        /// simulations, it is rarely necessary to call this function.
        ///
        /// @note In two use cases this function might be usefull:
        /// - When you want to reproduce the same simulation results,
        ///   consider combining this function with
        ///   `pasta::utils::reset_shared_engine()`.
        /// - When you want to simulate **another independent
        ///   stream of values use the same distribution object**.
        ///
        /// @return Non-const reference to `*this`.
        ///
        EXACT& reset_state();

        /// @brief Reset parameters of the distribution.
        ///
        /// @param args: variadic params same as used for the distr.
        ///
        template<typename ...Args,
                 typename = enable_if_all_t<std::is_arithmetic_v<Args>...> >
        EXACT& reset_param(Args ...args);

        /// @brief Return the max value of the pdf / pmf.
        ///
        /// @return The maximum value of the pdf / pmf function.
        ///
        scalr_t distr_max() const
        { return exact().max_distr_val_impl(); }

        /// @brief Return the value at specified index of the pdf / pmf.
        ///
        /// @param location: where pdf / pmf will be evaluated.
        /// @return The pdf / pmf value evaluated at `location`.
        ///
        scalr_t operator()(locat_t location) const
        { return exact().distr_val_at_impl(location); }

      protected:

#ifdef PST_USE_SHARED_ENGINE
        /// @brief Ctor. Protected to prevent instantiation.
        distrbase() = default;
#else
        /// @brief Ctor. Protected to prevent instantiation.
        distrbase(): _engine(utils::detail::get_rnd_dev()()) {};

        /// @brief Ctor.
        ///
        /// Initialize the random engine with specified seed.
        ///
        /// @param seed: the input seed value.
        ///
        explicit distrbase(int seed): _engine(seed) {};
#endif

        /// @brief Default copy ctor.
        distrbase(const distrbase &rhs) = default;

        /// @brief Default move ctor.
        distrbase(distrbase &&rhs) = default;

        /// @brief Default copy assignment operator.
        distrbase& operator=(const distrbase &rhs) = default;

        /// @brief Default move assignment operator.
        distrbase& operator=(distrbase &&rhs) = default;

#ifndef PST_USE_SHARED_ENGINE
        mutable std::mt19937_64 _engine;
#endif
      };

    } //!abstract
  } //!rnd

} //!pasta


# include "distrbase.hxx"
#endif //!PASTA_DISTRBASE_HH
