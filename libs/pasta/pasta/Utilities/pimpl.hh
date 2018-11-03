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
// File: pimpl.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:26:24 2018 Zhijin Li
// Last update Sat Nov  3 20:45:01 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_PIMPL_HH
# define PASTA_PIMPL_HH

# include "pasta/Utilities/utils.hh"


namespace pasta
{
  namespace internal__
  {
    /// Generic pimpl.
    template<typename T> class pimpl
    {
    public:

      /// Default ctor.
      pimpl();

      /// Variadic forwarding ctor.
      template<typename ...Args, enable_if_all_t
               <!std::is_base_of_v<pimpl<T>,std::decay_t<Args> >()...>* = nullptr>
      explicit pimpl(Args &&...);

      /// Copy ctor.
      pimpl(const pimpl<T> &);

      /// Copy assignment.
      pimpl<T>& operator=(const pimpl<T> &);

      /// Move ctor. Defaulted.
      pimpl(pimpl<T> &&);

      /// Move assignment. Defaulted.
      pimpl<T>& operator=(pimpl<T> &&);

      /// Dtor.
      ~pimpl();

      const T* operator->() const;

      T* operator->();

      const T& operator*() const;

      T& operator*();

    private:

      std::unique_ptr<T> _pdata;
    };
  } //!internal__
} //!pasta


# include "pimpl.hxx"
#endif //!PASTA_PIMPL_HH
