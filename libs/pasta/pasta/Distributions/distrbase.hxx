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
// File: distrbase.hxx for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:47:44 2018 Zhijin Li
// Last update Wed Nov  7 21:28:54 2018 Zhijin Li
// ---------------------------------------------------------------------------


namespace pasta
{
  namespace rnd
  {
    namespace abstract
    {

      // =====================================================================
      template<typename EXACT>
      auto distrbase<EXACT>::draw() const -> value_t
      { return (*this).exact().draw_impl(); }

      // =====================================================================
      template<typename EXACT> template<typename MT>
      void distrbase<EXACT>::draw(MT &&structure) const
      {
        static_assert(is_eigen_v<MT>, "ERROR: EXPECTS EIGEN TYPE INPUT.");
        // for(int __n = 0; __n < structure.size(); __n++)
        //   *(structure.data() + __n) = draw();
        for(int __m = 0; __m < structure.rows(); __m++)
          for(int __n = 0; __n < structure.cols(); __n++)
            structure(__m, __n) = draw();
      }

      // =====================================================================
      template<typename EXACT> template<typename Matrix>
      Matrix distrbase<EXACT>::draw(int rows, int cols) const
      {
        Matrix __result(rows,cols); draw(__result);
        return __result;
      }

      // =====================================================================
      template<typename EXACT>
      auto distrbase<EXACT>::reset_seed(seed_t seed) -> EXACT&
      {
#ifndef PST_USE_SHARED_RND_ENGINE
        _engine.seed(seed);
        return (*this).exact();
#else
        static_assert
        (err_on_call_v<EXACT>,
         "RANDOM ENGINE IS SHARED: USE RESET_SHARED_ENGINE FOR \
LESS ERROR-PRONE GLOBAL RESEEDING.");
#endif
      }

      // =====================================================================
      template<typename EXACT>
      auto distrbase<EXACT>::reset_state() -> EXACT&
      {
        (*this).exact()._distribution.reset();
        return (*this).exact();
      }

      // =====================================================================
      template<typename EXACT>
      auto distrbase<EXACT>::reset_state_with_seed(seed_t seed) -> EXACT&
      {
#ifndef PST_USE_SHARED_RND_ENGINE
        (*this).exact().reset_state();
        _engine.seed(seed);
        return (*this).exact();
#else
        static_assert
        (err_on_call_v<EXACT>,
         "RANDOM ENGINE IS SHARED: USE RESET_SHARED_ENGINE FOR \
LESS ERROR-PRONE GLOBAL RESEEDING.");
#endif
      }

      // =====================================================================
      template<typename EXACT>
      template<typename ...Args, typename>
      auto distrbase<EXACT>::reset_param(Args ...args) -> EXACT&
      {
        exact()._distribution.param(param_t(args...));
        return (*this).exact();
      }

    }
  }


#ifndef PST_USE_SHARED_RND_ENGINE
  namespace utils
  {
    namespace detail
    {

      // =====================================================================
      inline std::random_device& get_rnd_dev()
      {
        static std::random_device __rndd{};
        return __rndd;
      }
    }
  }
#endif

}
