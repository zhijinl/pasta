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
// File: Bernoulli.hxx for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:54:16 2018 Zhijin Li
// Last update Sat Nov  3 21:01:08 2018 Zhijin Li
// ---------------------------------------------------------------------------


namespace pasta
{
  namespace rnd
  {
    // =====================================================================
    template<typename PT, std::enable_if_t<std::is_arithmetic_v<PT> >*>
    Bernoulli::Bernoulli(PT p):
      abstract::distrbase<Bernoulli>(),
      _distribution(distr_t(p)) {}

    // =====================================================================
    inline auto Bernoulli::draw_impl() const -> value_t
#ifdef PST_NON_REPRODUCIBLE
    { return _distribution(this->_engine); };
#else
    { return _distribution(utils::shared_engine()); };
#endif

    // =====================================================================
    inline auto Bernoulli::max_distr_val_impl() const -> scalr_t
    { return std::max(success_rate(), 1-success_rate()); };

    // =====================================================================
    inline auto Bernoulli::distr_val_at_impl(locat_t location)
      const -> scalr_t
    { return location ? success_rate() : (1-success_rate()); };

  } //!rnd
} //!pasta