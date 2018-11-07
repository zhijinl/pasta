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
// File: samplersbase.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:43:37 2018 Zhijin Li
// Last update Wed Nov  7 18:43:32 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_SAMPLERSBASE_HH
# define PASTA_SAMPLERSBASE_HH

# include "Kernels.hh"
# include "pasta/core.hh"
# include "pasta/distributions.hh"


namespace pasta
{
  namespace stats
  {
    namespace abstract
    {

      /// The abstract class samplersbase.
      template<typename EXACT>
      class samplersbase: public internal__::root__<EXACT>
      {
      private:

        using internal__::root__<EXACT>::   exact;
        using specs_t =      traits::specs<EXACT>;
        using scalr_t = typename specs_t::scalr_t;
        using sampl_t = typename specs_t::sampl_t;

      protected:

        /// Default ctor.
        samplersbase() = default;

        /// Draw a sample.
        ///
        /// @return: the sampled value.
        ///
        sampl_t draw() const;

        /// Draw samples into an Eigen dynamic vector.
        ///
        /// @param vec: the input Eigen dynamic vector. Can be row or column
        /// ordered
        ///
        template<typename Vector,
                 enable_if_all_t<dim_dispatch_v<Vector>()==1,
                                 is_eigen_dynamic_vec_v<Vector>()>* = nullptr>
        void draw(Vector &vec);

        /// Draw samples into an Eigen structure other than a dynamic vector.
        ///
        /// @param structure: the input Eigen struct. **Must be col ordered**.
        ///
        template<typename Struct,
                 enable_if_all_t<dim_dispatch_v<Struct>()!=1,
                                 is_eigen_v<Struct> >* = nullptr>
        void draw(Struct &structure);

        /// @brief Reset the sampler state.
        ///
        /// This resets the state of the internal random variables and
        /// distributions.
        ///
        /// Together with the `pasta::utils::reset_shared_engine()`
        /// function, they can be used to reproduce the same simulation
        /// results.
        ///
        EXACT& reset_state() { return exact().reset_state_impl(); }

      };

    } //!abstract
  } //!stats
} //!pasta


# include "samplersbase.hxx"
#endif //!PASTA_SAMPLERSBASE_HH
