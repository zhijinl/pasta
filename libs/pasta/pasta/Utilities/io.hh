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
// File: io.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:21:59 2018 Zhijin Li
// Last update Sat Nov  3 22:40:58 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_IO_HH
# define PASTA_IO_HH

# include <memory>
# include <iomanip>
# include <fstream>
# include <algorithm>
# include <filesystem>
// # include <sys/stat.h>
// # include "common/dirent.hh"
# include "pasta/Core/meta.hh"
// # include "pasta/Utilities/mipp_helpers.hh"
// # include "boost/filesystem.hpp"


namespace pasta
{

  namespace io
  {

    /// @brief Read data from a binary file.
    ///
    /// Function returns pair of arr size and arr data. Use the std::get()
    /// method afterwards to retrieve data and size: arr is of type unique
    /// ptr.
    ///
    /// @param data_path: string path to data file.
    /// @param enable_log: if true, print on console summary.
    /// @return A std::pair<data size, data unique_ptr>.
    ///
    template<typename T> auto read_raw_to_array(const std::string &data_path,
                                                bool enable_log=true)
      -> std::pair<long, std::unique_ptr<T[]> >;

    /// @brief Read data from raw files to a STL vector.
    ///
    /// No ownership involved.
    /// @param data_path: string path to data file.
    /// @param enable_log: if true, print on console summary.
    /// @return A std::vector containing the read data.
    ///
    template<typename T>
    std::vector<T> read_raw_to_vec(const std::string &data_path,
                                   bool enable_log=true);

    /// @brief Write array data to a raw file.
    ///
    /// Take an input non-owning raw pointer and its size, write its
    /// content to a raw format file. Assume storage to be contiguous.
    /// This function does not take ownership.
    ///
    /// @param data: the non-owning data pointer, the contiguous arr.
    /// @param arr_size: allocated size of the content arr.
    /// @param data_path: path to write.
    /// @param enable_log: if true, print on console summary.
    ///
    template<typename T>
    void write_arr_to_rawfile(T *data, long arr_size,
                              const std::string &data_path,
                              bool enable_log=true);

    /// @brief Read coordinated txt file to an Eigen::Matrix.
    ///
    /// Text file assumed line-by-line with Dim-dimentional coordinates,
    /// separated by white spaces. i.e each line contains Dim coords.
    /// Return matrix will be col-majored Dim x size.
    ///
    /// @param data_path: string path to data file.
    /// @param enable_log: if true, print on console summary.
    /// @return The Eigen Dim x size dynamic matrix.
    ///
    template<typename T, int Dim>
    auto read_text_to_mat(const std::string &data_path,
                          bool enable_log=true)
      -> Eigen::Matrix<T,Dim,Eigen::Dynamic>;

    /// @brief Write an Eigen matrix to text file.
    ///
    /// Write each column as a line separated by white spaces
    ///
    /// @param save_path: std::string indicating file path.
    /// @param mat: the input matrix to be writen.
    /// @param pr: precision for floating-point writing. Defaults to 18.
    /// @param enable_log: if true, print on console summary.
    ///
    template<typename MT, typename = std::enable_if_t<is_eigen_v<MT> > >
    void write_mat_to_text(const std::string &save_path, MT &&mat, int pr=18,
                           bool enable_log=true);

    /// @brief Read txt file into an `std::vector<std::string>`.
    ///
    /// Each line will be read into a `std::string`. The ordering
    /// in the vector is the same as the line order.
    ///
    /// @param txt_path: path to the input text file.
    /// @return An `std::vector<std::string>` object containing
    /// the parsed line strings.
    ///
    std::vector<std::string> read_txt_to_strs(const char *txt_path);

    /// @brief Read all raw files undr the current dir to unique_arr.
    ///
    /// Use std::get<2> then std::move to extract the filled array. Use
    /// std::get<0> to get the # of files. Use std::get<1> to get data
    /// size for each file. The function assumes files having uniform
    /// size, which must be true.
    ///
    /// @note Some codes are courtesys of mipp::io.
    ///
    /// @param path: std::string of the dir.
    /// @param hint: hint for starting sequence of viable filenames.
    /// @param skip: use skip to specify # of bytes to ignore in the
    /// beginning of each file.
    /// @return An `std::tuple`. 0. # of arr, 1. size of one file, 2. the
    /// allocated arr.
    ///
    ///
    template<typename T>
    auto read_raw_dired_to_arr(const std::string &path,
                               const std::string &hint="",
                               int skip=0)
      ->  std::tuple<int, long, std::unique_ptr<T[]> >;

    /// @brief Check if the input path represent a file.
    ///
    /// @param path: the input path.
    /// @return True if the path represents a file.
    ///
    bool is_file(const char* path);

    /// @brief Check if the input path represent a directory.
    ///
    /// @param path: the input path.
    /// @return True if the path represents a directory.
    ///
    bool is_dir(const char* path);

    /// @brief List file names in a directory.
    ///
    /// @param path: path of the directory.
    /// @param hint: hint for starting sequence of viable filenames.
    /// @return A `std::vector` containing full paths of all files.
    ///
    auto ls_files(const char *path, const std::string &hint="")
      -> std::vector<std::string>;

    /// @brief Parse an input txt into a an `std::vector<std::string>`
    /// where each line becomes a string element.
    ///
    /// @param txt_path: input path of the txt file.
    /// @return Tokenized `std::vector<std::string>` object.
    ///
    std::vector<std::string> tokenize_txt(const char *txt_path);


    namespace detail
    {

      /// @brief Count number of bytes in a binary fstream.
      ///
      /// @param file: the input fstream.
      /// @param skip: specifying number of bytes to skip for counting.
      /// @return The number of bytes in file (skip substracted).
      ///
      int __count_bytes(std::fstream &file, int skip=0);

      /// @brief Count number of lines in a text fstream.
      ///
      /// @param file: the input fstream.
      /// @return The bumber of lines in file.
      ///
      int __count_lines(std::fstream &file);

      /// @brief Open a file and check status.
      ///
      /// @param file: the input/output fstream.
      /// @param path: path of the file to be opened.
      /// @param tags: variadic args indicating file openning tags.
      /// @return An `std::fstream`.
      ///
      template<typename ...Args>
      void __open_file_and_check(std::fstream &file,
                                 const std::string &path,
                                 Args &&...tags);

      /// @brief Read data in an `std::fstream` to an `std::unique_ptr`.
      ///
      /// @param file: the input std::fstream.
      /// @param path: path where the std::fstream was opened.
      /// @param n_bytes: the number of bytes in the input file.
      /// @param enable_log: if true, print on console summary.
      /// @return An `std::unique_ptr`.
      ///
      template<typename T>
      auto __fs_to_uptr(std::fstream &file, const std::string &path,
                        int n_bytes, bool enable_log=true)
        -> std::pair<long, std::unique_ptr<T[]> >;

      /// @brief Read data in an `std::fstream` to an `std::vector`.
      ///
      /// @param file: the input std::fstream.
      /// @param path: path where the std::fstream was opened.
      /// @param n_bytes: the number of bytes in the input file.
      /// @param enable_log: if true, print on console summary.
      /// @return An `std::vector`.
      ///
      template<typename T>
      auto __fs_to_vec(std::fstream &file, const std::string &path,
                       int n_bytes, bool enable_log=true) -> std::vector<T>;

      /// @brief Read data in an std::fstream to an Eigen matrix.
      ///
      /// Extra entries in each line will be discarded.
      /// @param file: the input std::fstream.
      /// @param path: path where the std::fstream came from.
      /// @param nl: the number of lines in the input file.
      /// @param enable_log: if true, print on console summary.
      /// @return An Eigen Dim x Dynamic matrix.
      ///
      template<typename T, int Dim>
      auto __fs_to_mat(std::fstream &file, const std::string &path,
                       int nl, bool enable_log=true)
        -> Eigen::Matrix<T,Dim,Eigen::Dynamic>;

      /// @brief Write an array of data into a std::fstream.
      ///
      /// @param arr: raw arr containing data to write.
      /// @param arr_size: size of the input array.
      /// @param file: the output std::fstream where data will be writen.
      /// @param path: path where the std::fstream was opened.
      /// @param enable_log: if true, print on console summary.
      ///
      template<typename T>
      void __arr_to_fs(T *arr, int arr_size, std::fstream &file,
                       const std::string &path, bool enable_log=true);

      /// @brief Write an Eigen matrix to an text fstream.
      ///
      /// @param file: the fstream in which matrix will be writen.
      /// @param path: path where the std::fstream was opened.
      /// @param mat: the input matrix. Colmajored, size Dim x Dynamic.
      /// @param pr: precision for floating-point writing.
      /// @param enable_log: if true, print on console summary.
      ///
      template<typename Matrix>
      void __mat_to_fs(std::fstream &file, const std::string &path,
                       Matrix &&mat, int pr, bool enable_log=true);

      /// @brief Check size of files in a directory.
      ///
      /// Throws if size is not consistent.
      /// @param names: a std::vector containing cached file names.
      /// @param skip: the number of bytes to be skipped for size counting.
      /// @return The uniform size of every file in directory.
      ///
      int __check_filesize(const std::vector<std::string> &names, int skip=0);

    } //!detail

  } //!io
} //!pasta


# include "io.hxx"
#endif //!PASTA_IO_HH
