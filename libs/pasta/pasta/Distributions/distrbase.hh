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
// Last update Wed Nov  7 22:39:45 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_DISTRBASE_HH
# define PASTA_DISTRBASE_HH

# include "pasta/core.hh"
# include "pasta/Utilities/utils.hh"


namespace pasta
{

#ifndef PST_USE_SHARED_RND_ENGINE
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

      protected:

        using engine_t = std::mt19937_64;
        using seed_t   = typename engine_t::result_type;

#ifdef PST_USE_SHARED_RND_ENGINE
        /// @brief Ctor.
        distrbase() = default;
#else
        /// @brief Ctor.
        distrbase(): _engine(utils::detail::get_rnd_dev()()) {};

        /// @brief Ctor.
        ///
        /// Initialize the random engine with specified seed.
        ///
        /// @param seed: the input seed value.
        ///
        explicit distrbase(seed_t seed): _engine(seed) {};
#endif

        /// @brief Default copy ctor.
        distrbase(const distrbase &rhs) = default;

        /// @brief Default move ctor.
        distrbase(distrbase &&rhs) = default;

        /// @brief Default copy assignment operator.
        distrbase& operator=(const distrbase &rhs) = default;

        /// @brief Default move assignment operator.
        distrbase& operator=(distrbase &&rhs) = default;

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

        /// @brief Reset the random seed.
        ///
        /// Resetting random seed of a random variable is only
        /// relevant when a global seeded shared engine is not used.
        /// If a global engine is used, consider calling
        /// `pasta::utils::reset_shared_engine()` to reset the random
        /// seed.
        ///
        /// @warning Resetting random seed alone does not guarantee
        /// reproducing the same result for a random variable. This
        /// is because some random distributions have cached internal
        /// state. Successive calls of `draw()` in random variables
        /// created from such distributions are dependent of one
        /// another. In this case consider discarding the cached
        /// internal state by calling `reset_state()`, or by calling
        /// `reset_state_with_seed()`, which calls `reset_state()`
        /// internally.
        ///
        /// @param seed: new random seed.
        /// @return Non-const reference to `*this`.
        ///
        EXACT& reset_seed(seed_t seed);

        /// @brief Reset state of the random distribution, such that
        /// the next call to draw() is independent from the previous one.
        ///
        /// Some random distributions such as `Gaussian` have internal
        /// state random variables. In this case, successive calls to
        /// `draw()` method of a random variable are **dpendent of one
        /// another**. As a consequence, in the middle of one simulation
        /// stream, resetting the random seed of a random variable
        /// might not guarantee to reproduce the same result,
        /// since the random variable continues using previously
        /// cached internal state. Calling `reset_state()` discards
        /// the internal state of a random variable, and resets the
        /// internal state to default. In one simulation stream of a
        /// random variable, resetting the random seed followed by
        /// resetting its state guarantees the reproduction of the
        /// same result.
        ///
        /// @warning This function is **different from reseting the
        /// random seed**. In one stream of simulation, successively
        /// calling this function is unneccessary and may significantly
        /// slow down the simulation.
        ///
        /// @note In two use cases this function might be helpfull:
        /// - When you want to reproduce the same results during one
        ///   simulation stream from a random variable. In this case,
        ///   reset the random seed before calling this function.
        /// - For random distributions that have internal state,
        ///   calling this function allows to simulate **another
        ///   independent stream of values, use the same random
        ///   variable object**.
        ///
        /// @return Non-const reference to `*this`.
        ///
        EXACT& reset_state();

        /// @brief Reset the internal state of a random variable and
        /// the random seed.
        ///
        /// It might be usefull for reproducible simulations with
        /// random variables that have internal state.
        ///
        /// @param seed: new random seed.
        /// @return Non-const reference to `*this`.
        ///
        /// @sa `reset_seed()`, `reset_state()`.
        ///
        EXACT& reset_state_with_seed(seed_t seed);

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

#ifndef PST_USE_SHARED_RND_ENGINE

        mutable engine_t _engine;
#endif
      };

    } //!abstract
  } //!rnd

} //!pasta


# include "distrbase.hxx"
#endif //!PASTA_DISTRBASE_HH
