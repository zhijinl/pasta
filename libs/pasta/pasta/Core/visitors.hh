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
// File: visitors.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Thu Nov  1 23:58:06 2018 Zhijin Li
// Last update Sat Nov  3 23:31:36 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_VISITORS_HH
# define PASTA_VISITORS_HH

# include <variant>

# include "meta.hh"
# include "roots.hh"


namespace pasta
{
  namespace var
  {

    /// @brief Visitor: determine if a `std::variant` is a r.v.
    ///
    /// Expacts a `std::variant` that is either a random va or
    /// a deterministic value. Works on all these types. Not a
    /// template.
    ///
    /// @return: if the `std::variant` holds a r.v. or not.
    ///
    struct is_rnd_var
    {
      template<typename VT, std::enable_if_t
               <std::is_arithmetic_v<std::decay_t<VT> > >* = nullptr>
      bool operator()(VT &&) const { return false; }

      template<typename VT, std::enable_if_t<is_pst_distr_v<VT>()>* = nullptr>
      bool operator()(VT &&) const { return true; }
    };

    // template<typename Var>
    // constexpr bool is_rnd_var()
    // {

    // }

    /// @brief Visitor: reset a `std::variant` of r.v.
    ///
    /// Internally call the `reset_state` function of the random
    /// vairbale. Does nothing if a deterministic value is encountered.
    ///
    struct reset_rv
    {
      template<typename VT, std::enable_if_t
               <std::is_arithmetic_v<std::decay_t<VT> > >* = nullptr>
      void operator()(VT &&) const {}

      template<typename VT, std::enable_if_t<is_pst_distr_v<VT>()>* = nullptr>
      void operator()(VT &&rv) const { rv.reset_state(); }
    };

    /// @brief Visitor: get the curr held value of the `std::variant`.
    ///
    /// Similar to std::get, with static assert. Expects a std::variant
    /// that is either arithmetic or not. Works only on arithmetic types.
    ///
    /// Template arg: the value type.
    ///
    /// @return: the held value.
    ///
    template<typename T> struct get_val
    {
      static_assert(std::is_arithmetic_v<T>,
                    "ERROR: GET_VAL NOT PASSED ARITHMETIC TYPE.");

      T operator()(const T &val) const { return val; }

      template<typename PT, std::enable_if_t
               <!std::is_arithmetic_v<std::decay_t<PT> > >* = nullptr>
      T operator()(PT &&) const
      { throw err::exception(std::string("err: get_val expects non-rand.\n")); }
    };

    /// @brief Visitor: draw a realization of a r.v.
    ///
    /// Expects a `std::variant` that is either a r.v. or a
    /// deterministic val works only when it's an r.v.
    ///
    /// Template arg: T the value type.
    ///
    /// @return: the drawn random value.
    ///
    template<typename T> struct draw_val
    {
      template<typename VT, std::enable_if_t<!is_pst_distr_v<VT>()>* = nullptr>
      T operator()(VT &&) const
      { throw err::exception(std::string("err: draw_val expects randvar.\n")); }

      template<typename VT, std::enable_if_t<is_pst_distr_v<VT>()>* = nullptr>
      T operator()(VT &&operand) const { return operand.draw(); }
    };

    template<typename T, bool is_arith=std::is_arithmetic_v<T> >
    struct rndvar_sampler {};

    /// @brief Generic sampler for `std::variant` of `pasta` random
    /// variables.
    ///
    /// This visitor applies to a random variable or a deterministic
    /// scalar. For a deterministic scalar input, the same scalar will
    /// be **copied and returned**. For a random variable input, a
    /// random value will be drawn. A compilation error will occur if
    /// the input is neither a scalar nor a random variable.
    ///
    /// @param T: the scalar value type.
    ///
    template<typename T>
    struct rndvar_sampler<T,true>
    {
      template<typename Var, std::enable_if_t<is_pst_distr_v<Var>()>* = nullptr>
      T operator()(Var &&operand) const { return operand.draw(); }

      template<typename Var,
               std::enable_if_t<std::is_arithmetic_v<Var> >* = nullptr>
      T operator()(Var &&operand) const { return operand; }

      template<typename Var, enable_if_all_t
               <!std::is_arithmetic_v<Var>, !is_pst_distr_v<Var>()>* = nullptr>
      T operator()(Var &&operand) const
      {
        static_assert(std::is_arithmetic_v<Var> || is_pst_distr_v<Var>(),
                      "ERR: THE VARIANT MUST BE SCALAR OR RANDOM VAR.");
        return -1;
      }
    };
    ///@}

    /// @brief Generic sampler for `std::variant` of `pasta` random
    /// variables.
    ///
    /// This visitor applies to a random variable or a deterministic
    /// scalar. The constructor takes an `Eigen` structure as input and
    /// samples values into it. For a deterministic scalar input, the
    /// output will be filled with the same scalar value. For a random
    /// variable input, the output will be filled with random values
    /// sampled from the random variable. A compilation error occurs if
    /// the input is neither a scalar nor a random variable.
    ///
    /// @param T: the scalar value type.
    /// @param Output: the output container structure. Can be any
    /// type that that draw method accepts, f.ex, an Eigen Matrix.
    ///
    template<typename Output>
    struct rndvar_sampler<Output,false>
    {
      static_assert(is_eigen_v<Output>,
                    "ERROR: EXPECTS AN EIGEN TYPE AS INPUT.");

      rndvar_sampler(Output &output): _output( output ) {}

      template<typename Var, std::enable_if_t<is_pst_distr_v<Var>()>* = nullptr>
      void operator()(Var &&operand) const { operand.draw(_output); }

      template<typename Var,
               std::enable_if_t<std::is_arithmetic_v<Var> >* = nullptr>
      void operator()(Var &&operand) const
      {
#pragma omp parallel for collapse(2)
        for(int __m = 0; __m < _output.rows(); __m++)
          for(int __n = 0; __n < _output.cols(); __n++)
            _output(__m, __n) = operand.draw();
      }

      template<typename Var, enable_if_all_t
               <!std::is_arithmetic_v<Var>, !is_pst_distr_v<Var>()>* = nullptr>
      void operator()(Var &&operand) const
      {
        static_assert(std::is_arithmetic_v<Var> || is_pst_distr_v<Var>(),
                      "ERR: THE VARIANT MUST BE SCALAR OR RANDOM VAR.");
      }

    private:
      Output       &_output;
    };

  } //!var


  namespace utils
  {

    template<typename Var, typename Output,
             typename = std::enable_if_t
             <is_variant_of_v<Var,pst_id_list::PASTA_RANDOM_VAR>()> >
    void sample_var(Var &&var, Output &output)
    {
      std::visit
        (var::rndvar_sampler<Output>(output), std::forward<Var>(var));
    }

    template<typename Scalar, typename Var,
             typename = std::enable_if_t
             <is_variant_of_v<Var,pst_id_list::PASTA_RANDOM_VAR>()> >
    Scalar sample_var(Var &&var)
    {
      return std::visit
        (var::rndvar_sampler<Scalar>(), std::forward<Var>(var));
    }

    template<typename Output, typename Var, typename ...Sizes,
             enable_if_all_t<is_variant_of_v<Var,pst_id_list::PASTA_RANDOM_VAR>(),
                             not_empty<Sizes...>()>* = nullptr>
    Output sample_var(Var &&var, Sizes ...sizes)
    {
      Output __result(sizes...);
      std::visit
        (var::rndvar_sampler<Output>(__result), std::forward<Var>(var));
      return __result;
    }

  } //!utils

} //!pasta


#endif //!PASTA_VISITORS_HH
