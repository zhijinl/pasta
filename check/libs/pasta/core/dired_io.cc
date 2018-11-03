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
// File: dired_io.cc for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 17:44:11 2018 Zhijin Li
// Last update Sat Nov  3 00:45:06 2018 Zhijin Li
// ---------------------------------------------------------------------------


# include <cstdio>
# include "pasta/core.hh"
# include "pasta/utilities.hh"


template<typename Scalar>
void print_arr(const std::unique_ptr<Scalar[]> &arr, int size)
{
  std::copy(arr.get(), arr.get()+size,
            std::ostream_iterator<Scalar>{std::cout, " "});
  std::cout << '\n';
}

int main()
{
  /// Definitions.
  using vtype = float;

  const std::string dir_name = "./";
  const std::string name_base = "__data.pasta.unit.test.slice";
  const std::string hint = "__data.pasta.unit.test";

  int n_files = 10;
  int arr_size = 25;
  int skip = 7*sizeof(vtype);


  /// Synthetic data.
  std::vector<vtype> data_vec(n_files*arr_size);
  std::iota(std::begin(data_vec), std::end(data_vec), 0.0f);

  for(int n = 0; n < n_files; ++n)
  {
    auto curr_name = name_base+"."+std::to_string(n);
    pasta::io::write_arr_to_rawfile(data_vec.data()+n*arr_size,
                                     arr_size, curr_name);
  }


  /// List files.
  auto file_list = pasta::io::ls_files(dir_name.c_str(), hint);
  std::cout << file_list.size() << " files listed.\n";


  /// Read hinted dir files.
  auto no_skip = pasta::io::read_raw_dired_to_arr<vtype>(dir_name, hint);
  auto skipped = pasta::io::read_raw_dired_to_arr<vtype>(dir_name, hint, skip);

  print_arr(std::get<2>(no_skip), std::get<0>(no_skip)*std::get<1>(no_skip));
  print_arr(std::get<2>(skipped), std::get<0>(skipped)*std::get<1>(skipped));


  /// Check for correct size.
  if( std::get<0>(no_skip) != n_files ||
      std::get<0>(skipped) != n_files )
  {
    std::cerr << "number of files went wrong.\n";
    return 1;
  }


  /// Check for correct skip.
  if( std::get<1>(no_skip) - std::get<1>(skipped) != 7)
  {
    std::cerr << "file size went wrong.\n";
    return 1;
  }


  /// Check for correct no skip val.
  for(int n = 0; n < std::get<0>(no_skip)*std::get<1>(no_skip); ++n)
  {
    if( !pasta::utils::f_equal(std::get<2>(no_skip)[n], vtype(n)) )
    {
      std::cerr << "no_skip: multiple files reading went wrong.\n"
                << std::get<2>(no_skip)[n] << " vs " << vtype(n) << '\n';
      return 1;
    }
  }


  /// Check for correct skipped val.
  for(int n = 0; n < std::get<0>(skipped)*std::get<1>(skipped); ++n)
  {
    int period = arr_size-skip/sizeof(vtype);
    vtype val = (n/period)*arr_size + n%period + skip/sizeof(vtype);
    if( !pasta::utils::f_equal(std::get<2>(skipped)[n], val) )
    {
      std::cerr << "skipped: multiple files reading went wrong.\n"
                << std::get<2>(skipped)[n] << " vs " << val << '\n';
      return 1;
    }
  }


  /// Remove files.
  for(int n = 0; n < n_files; ++n)
  {
    auto curr_name = name_base+"."+std::to_string(n);
    remove(curr_name.c_str());
  }


  return 0;
}
