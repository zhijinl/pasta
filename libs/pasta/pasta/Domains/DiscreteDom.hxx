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
// File: DiscreteDom.hxx for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 22:58:41 2018 Zhijin Li
// Last update Fri Nov  2 23:03:35 2018 Zhijin Li
// ---------------------------------------------------------------------------


namespace pasta
{
  namespace dom
  {

    // =====================================================================
    template<typename Scalar, int Dim> template<typename Steps>
    void DiscreteDom<Scalar,Dim>::set_step_size(Steps &&steps)
    {
      _step_size = std::forward<Steps>(steps);
      _currn_pos = _bound_box.col(0);
      utils::comp_bound_discrete_size(_bound_box, _step_size);
    }

    // =====================================================================
    template<typename Scalar, int Dim> template<typename Bound>
    void DiscreteDom<Scalar,Dim>::set_bound(Bound &&bound)
    {
      _bound_box = std::forward<Bound>(bound);
      _currn_pos = _bound_box.col(0);
      utils::comp_bound_discrete_size(_bound_box, _step_size);
    }

    // =====================================================================
    template<typename Scalar, int Dim>
    template<int Loop, typename Func, enable_if_all_t<Loop!=1,Dim!=1>*>
    void DiscreteDom<Scalar,Dim>::__traverse(Func handler) const
    {
      /// TODO: parallelize me!
      for(int __n = 0; __n < _side_size(Loop-1); ++__n)
      {
        _currn_pos(Loop-1) = _bound_box(Loop-1,0)+__n*_step_size(Loop-1);
        __traverse<Loop-1,Func>(handler);
      }
    }

    // =====================================================================
    template<typename Scalar, int Dim>
    template<int Loop, typename Func, enable_if_all_t<Loop==1,Dim!=1>*>
    void DiscreteDom<Scalar,Dim>::__traverse(Func handler) const
    {
      for(int __n = 0; __n < _side_size(0); ++__n)
      {
        _currn_pos(0) = _bound_box(0,0)+__n*_step_size(0);
        handler(_currn_pos);
      }
    }

    // =====================================================================
    template<typename Scalar, int Dim>
    template<int Loop, typename Func, enable_if_all_t<Loop==1,Dim==1>*>
    void DiscreteDom<Scalar,Dim>::__traverse(Func handler) const
    {
      for(int __n = 0; __n < _side_size; ++__n)
      {
        _currn_pos = _bound_box(0)+__n*_step_size;
        handler(_currn_pos);
      }
    }

  } //discrete


  namespace utils
  {

    // =====================================================================
    template<typename Scalar, typename Data, typename Params>
    auto make_discrete_domain(Data && data, Params &&sizes, double scale)
      -> dom::DiscreteDom<Scalar,dim_dispatch_v<Params>()>
    {
      constexpr int __dim = dim_dispatch_v<Params>();

      return dom::DiscreteDom<Scalar, __dim>
        ( utils::scale_bound
          ( utils::comp_bound(std::forward<Data>(data)), scale ),
          std::forward<Params>(sizes) );
    }

  } //!utils

} //!pasta
