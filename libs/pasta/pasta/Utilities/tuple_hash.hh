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
// File: tuple_hash.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:26:55 2018 Zhijin Li
// Last update Fri Nov  2 15:26:56 2018 Zhijin Li
// ---------------------------------------------------------------------------


# include <tuple>


namespace pasta
{
  namespace utils
  {

    /// @brief Base case for tuple hash.
    ///
    /// Calls `std::hash` internally.
    ///
    template <typename Type> struct tuple_hash
    {
      std::size_t operator()(const Type &input) const
      {
        return std::hash<Type>()(input);
      }
    };

    /// @brief Combine new hashed value to existing one.
    ///
    /// @note See http://stackoverflow.com/questions/4948780.
    ///
    /// @param lhs: an existing hashed value.
    /// @param rhs: a new hashed value to be combined.
    ///
    template <class Type>
    void hash_combine(std::size_t& lhs, const Type &rhs)
    {
      lhs ^= pasta::tuple_hash<Type>()(rhs) + 0x9e3779b9 + (lhs<<6) + (lhs>>2);
    }

    namespace detail
    {

      ///@{
      ///
      /// @brief Recursively hash each key in tuple, and combine them
      /// to a final value.
      ///
      template <class Tuple, std::size_t Indx=(std::tuple_size<Tuple>::value-1)>
      struct hash_value_impl
      {
        static void apply(std::size_t& seed, const Tuple &tuple)
        {
          hash_value_impl<Tuple,Indx-1>::apply(seed, tuple);
          hash_combine(seed, std::get<Indx>(tuple));
        }
      };

      template <class Tuple>
      struct hash_value_impl<Tuple,0>
      {
        static void apply(std::size_t& seed, Tuple const& tuple)
        {
          hash_combine(seed, std::get<0>(tuple));
        }
      };
      ///@}

    }

    /// @brief Hash a tuple of basic type keys.
    ///
    /// @note Types must expands to standard library defined hashable
    /// types.
    ///
    template <typename ...Types>
    struct tuple_hash<std::tuple<Types...> >
    {
      std::size_t operator()(std::tuple<Types...> const& tt) const
      {
        std::size_t seed = 0;
        detail::hash_value_impl<std::tuple<Types...> >::apply(seed, tt);
        return seed;
      }
    };

  }
}
