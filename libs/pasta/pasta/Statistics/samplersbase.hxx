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
// File: samplersbase.hxx for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:43:52 2018 Zhijin Li
// Last update Sat Nov 10 00:07:55 2018 Zhijin Li
// ---------------------------------------------------------------------------


namespace pasta
{
  namespace stats
  {
    namespace abstract
    {
      // =====================================================================
      template<typename EXACT>
      auto samplersbase<EXACT>::draw() const -> sampl_t
      { return exact().draw_impl(); }

      // =====================================================================
      template<typename EXACT>
      template<typename Vector,
               enable_if_all_t<dim_dispatch_v<Vector> == 1,
                               is_eigen_dynamic_vec_v<Vector> >*>
      void samplersbase<EXACT>::draw(Vector &vec)
      {
        for(int __n = 0; __n < vec.size(); __n++)
            vec(__n) = draw();
      }

      // =====================================================================
      template<typename EXACT>
      template<typename Struct,
               enable_if_all_t<dim_dispatch_v<Struct> != 1,
                               is_eigen_v<Struct> >*>
      void samplersbase<EXACT>::draw(Struct &structure)
      {
        for(int __c = 0; __c < structure.cols(); __c++)
          structure.col(__c) = draw();
      }

    } //!abstract
  } //!stats
} //!pasta
