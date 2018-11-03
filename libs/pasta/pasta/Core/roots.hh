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
// File: roots.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 17:24:23 2018 Zhijin Li
// Last update Fri Nov  2 17:24:24 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_ROOTS_HH
# define PASTA_ROOTS_HH

# include <string>
# include <exception>


namespace pasta
{
  namespace internal__
  {
    /// The root class. Cannot be instantiated.
    template<typename EXACT>
    class root__
    {
    public:
      /// Static dispatches.
      EXACT& exact() { return static_cast<EXACT&>(*this); };
      const EXACT& exact() const { return static_cast<const EXACT&>(*this); };

    protected:
      /// Ctor is protected to prevent instantiation.
      constexpr root__() {};
    };

#ifdef PASTA_ENABLE_INSTANCE_COUNT
    /// Counter class for ctor calls
    template<typename T>
    class call_count__
    {
    protected:
      // Ctors.
      call_count__() { ++_count; };
      call_count__(const call_count__&) { ++_count; };
      ~call_count__() {};

      static int count() { return _count; }

    private:
      static int _count;
      void operator delete(void*); //< Prohibit ptr instantiation w/ new.
    };
    template<typename EXACT> int call_count__<EXACT>::_count = 0;
#endif

  } //!internal__

  namespace err
  {
    /// Exception class for error handling.
    struct exception : public std::exception
    {
      explicit exception(std::string msg = "") noexcept:
        std::exception(),_msg(msg) {}
      virtual ~exception() noexcept {}
      const char* what() const noexcept override { return _msg.c_str(); }

    private:
      std::string _msg;
    };
  } //!err

} //!pasta


#endif //!PASTA_ROOTS_HH
