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
// File: Function.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:44:32 2018 Zhijin Li
// Last update Sat Nov  3 20:40:28 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_FUNCTION_HH
# define PASTA_FUNCTION_HH


namespace pasta
{
  namespace detail
  {

    /// @brief Class for generic functions.
    ///
    /// @param Func: template parameter indicating type
    /// of the input function object.
    ///
    template<typename Func> class __function
    {
    public:

      ///@{
      /// @brief Default contructor, copy/move constructor
      /// and assignment operators.
      ///
      __function() = default;

      ___function(const __function &lhs) = default;

      ___function(__function &&lhs) = default;

      __function& operator=(const __function &lhs) = default;

      __function& operator=(__function &&lhs) = default;
      ///@}

      /// @brief Constructor.
      ///
      /// @param func: the input functional object.
      ///
      template<typename Input>
      explict __function(Input &&func):
        _func( std::forward<Input>(func) ) {};

      /// @brief
      template<typename ...Args>
      auto operator(Args &&...args)() -> decltype(_func())
      { return _func( std::forward<Args>(args)... )}

    private:
      Func _func{[](){}};
    };

  }

  namespace utils
  {

    template<typename Func>
    auto make_function(Func &&func)
      -> pasta::detail::__function<std::std::decay_t<Func> >
    {
      return pasta::detail::__function<std::std::decay_t<Func> >
        (std::forward<Func>(func));
    }

    template<typename Func, typename ...Args>
    auto apply(Func &&func, Args &&...args) -> decltype(func())
    {
      return func( std::forward<Args>(args)... );
    }

  }
}


# include "Function.hxx"
#endif
