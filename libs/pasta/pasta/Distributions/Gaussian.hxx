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
// File: Gaussian.hxx for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:52:09 2018 Zhijin Li
// Last update Tue Nov  6 23:39:49 2018 Zhijin Li
// ---------------------------------------------------------------------------


namespace pasta
{
  namespace rnd
  {

    // =====================================================================
    template<typename T>
    Gaussian<T>::Gaussian(scalr_t mu, scalr_t sigma):
      abstract::distrbase<Gaussian<T> >(),
      _distribution(mu, sigma) {}

    // =====================================================================
    template<typename T>
    Gaussian<T>::Gaussian(scalr_t mu, scalr_t sigma, unsigned seed):
#ifdef PST_USE_SHARED_ENGINE
      abstract::distrbase<Gaussian<T> >(),
#else
      abstract::distrbase<Gaussian<T> >(seed),
#endif
      _distribution(distr_t(mu, sigma))
    {
#ifdef PST_USE_SHARED_ENGINE
      static_assert
        (err_on_call_v<T>,
         "RANDOM ENGINE IS SHARED: USE RESET_SHARED_ENGINE FOR \
LESS ERROR-PRONE GLOBAL RESEEDING.");
#endif
    }

    // =====================================================================
    template<typename T>
    auto Gaussian<T>::draw_impl() const -> value_t
#ifdef PST_USE_SHARED_ENGINE
    { return _distribution(utils::shared_engine()); };
#else
    { return _distribution(this->_engine); };
#endif

    // =====================================================================
    template<typename T>
    auto Gaussian<T>::max_distr_val_impl() const -> scalr_t
    { return one_over_sqrt2pi/sigma(); };

    // =====================================================================
    template<typename T>
    auto Gaussian<T>::distr_val_at_impl(locat_t location) const -> scalr_t
    {
      return pasta::one_over_sqrt2pi/sigma()*
        std::exp( -(location-mu())*(location-mu())/(2*sigma()*sigma()) );
    };

  }
}
