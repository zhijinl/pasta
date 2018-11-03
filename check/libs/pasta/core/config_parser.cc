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
// File: config_parser.cc for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 17:44:23 2018 Zhijin Li
// Last update Fri Nov  2 17:44:23 2018 Zhijin Li
// ---------------------------------------------------------------------------


# include "pasta/utilities.hh"


int main()
{

  using dtype = double;
  using vectr_t = Eigen::Matrix<dtype,Eigen::Dynamic,1>;

  std::string input = "{3.14, 1.57, 0.0} | {0.0} | {1.0} | {2.0} | {3.0}";
  pasta::utils::rm_spaces(input);

  auto tokens = pasta::utils::tokenize(input);

  std::cout << tokens[0] << '\n';
  std::cout << tokens[1] << '\n';
  std::cout << tokens[2] << '\n';
  std::cout << tokens[3] << '\n';
  std::cout << tokens[4] << '\n';

  std::vector<vectr_t> truth
  {
    (vectr_t(3) << 3.14, 1.57, 0.0).finished(),
      (vectr_t(1) << 0.0).finished(),
      (vectr_t(1) << 1.0).finished(),
      (vectr_t(1) << 2.0).finished(),
      (vectr_t(1) << 3.0).finished()
      };

  for( std::size_t n = 0; n < tokens.size(); ++n )
  {
    auto &&elem = tokens[n];
    auto res = pasta::utils::parse_list_as_pt<dtype>(elem);
    std::cout << res.transpose() << '\n';

    if( res != truth[n] )
    {
      std::cerr << "point parsing went wrong. TRUTH: "
                << truth[n] << " | " << "PARSED: "
                << res << '\n';
      return 1;
    }
  }

  return 0;
}
