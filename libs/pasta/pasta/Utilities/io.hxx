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
// File: io.hxx for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:25:03 2018 Zhijin Li
// Last update Sat Nov  3 00:51:53 2018 Zhijin Li
// ---------------------------------------------------------------------------


namespace pasta
{
  namespace io
  {

    // =====================================================================
    template<typename T> std::pair<long, std::unique_ptr<T[]> >
    read_raw_to_array(const std::string &path, bool enable_log)
    {
      std::fstream __file;
      detail::__open_file_and_check(__file, path, std::fstream::in |
                                    std::fstream::binary);
      auto __n_bytes = detail::__count_bytes(__file);

      return detail::__fs_to_uptr<T>(__file,path,__n_bytes,enable_log);
    }

    // =====================================================================
    template<typename T>
    std::vector<T> read_raw_to_vec(const std::string &path, bool enable_log)
    {
      std::fstream __file;
      detail::__open_file_and_check(__file, path, std::fstream::in |
                                    std::fstream::binary);
      auto __n_bytes = detail::__count_bytes(__file);

      return detail::__fs_to_vec<T>(__file,path,__n_bytes,enable_log);
    }

    // =====================================================================
    template<typename T>
    void write_arr_to_rawfile(T *arr,long arr_size,
                              const std::string &path, bool enable_log)
    {
      std::fstream __file;
      detail::__open_file_and_check(__file, path, std::fstream::out |
                                    std::fstream::binary);
      detail::__arr_to_fs(arr, arr_size, __file, path, enable_log);
    }

    // =====================================================================
    template<typename T, int Dim >
    auto read_text_to_mat(const std::string &path, bool enable_log)
      -> Eigen::Matrix<T,Dim,Eigen::Dynamic>
    {
      std::fstream __file;
      detail::__open_file_and_check(__file, path);

      return detail::__fs_to_mat<T,Dim>
        (__file, path, detail::__count_lines(__file), enable_log);
    }

    // =====================================================================
    template<typename MT, typename>
    void write_mat_to_text(const std::string &path, MT &&mat, int pr,
                           bool enable_log)
    {
      std::fstream __file;
      detail::__open_file_and_check(__file, path, std::ofstream::out |
                                    std::ofstream::trunc);
      detail::__mat_to_fs(__file, path, std::forward<MT>(mat), pr, enable_log);
    }

    // =====================================================================
    inline std::vector<std::string> read_txt_to_strs(const char *txt_path)
    {
      std::fstream __file;
      detail::__open_file_and_check(__file, txt_path);
      auto __n_files = detail::__count_lines(__file);

      std::vector<std::string> __result;
      __result.reserve(__n_files);

      std::string __line;
      while( std::getline(__file, __line) )
        if( !__line.empty() ) __result.emplace_back(std::move(__line));

      return __result;
    }

    // =====================================================================
    template<typename T>
    auto read_raw_dired_to_arr(const std::string &path,
                               const std::string &hint, int skip)
      -> std::tuple<int, long, std::unique_ptr<T[]> >
    {
      auto __names = ls_files(path.c_str(), hint);
      auto __size = detail::__check_filesize(__names, skip)/sizeof(T);

      std::unique_ptr<T[]> __arr(new T[__names.size()*__size]);
      for(const auto &__file:__names)
      {
        std::fstream __curr_stream;
        detail::__open_file_and_check(__curr_stream, __file,
                                      std::fstream::in | std::fstream::binary);
        __curr_stream.seekg(skip, std::fstream::beg);
        __curr_stream.read
          (reinterpret_cast<char*>(__arr.get()+(&__file-&__names[0])*__size),
           __size*sizeof(T));
      }
      return std::make_tuple(__names.size(),__size,std::move(__arr));
    }

    // =====================================================================
    inline bool is_file(const char* path)
    {
      return std::filesystem::is_regular_file(path);
    }

    // =====================================================================
    inline bool is_dir(const char* path)
    {
      return std::filesystem::is_directory(path);
    }

    // // =====================================================================
    // inline auto ls_files(const char *path, const std::string &hint)
    //   -> std::vector<std::string>
    // {
    //   std::vector<std::string> __names;
    //   std::string __dir_name = std::string(path);
    //   if(__dir_name[__dir_name.size()-1] != '/') __dir_name += "/";

    //   int __nfiles = 0;
    //   DIR* __dir = opendir(path);

    //   if(!__dir) throw err::exception(std::string("err: open: ")+path+'\n');
    //   struct dirent* __entry;

    //   while( (__entry = readdir(__dir)) )
    //   {
    //     std::string __file = std::string(__entry->d_name);
    //     if( strcmp(__file.c_str(), ".") && strcmp(__file.c_str(), "..") )
    //     {
    //       if( hint.size() ) if( __file.substr(0, hint.size()) != hint) continue;
    //       __names.emplace_back(__dir_name+__file);
    //       ++__nfiles;
    //     }
    //   }
    //   closedir(__dir);

    //   std::sort(__names.begin(), __names.end());
    //   return __names;
    // }

    // =====================================================================
    inline auto ls_files(const char *path, const std::string &hint)
      -> std::vector<std::string>
    {
      std::vector<std::string> __files;
      using __iter_t = std::filesystem::directory_iterator;

      if( std::filesystem::exists(path) )
      {
        if( !std::filesystem::is_directory(path) )
          throw pasta::err::exception
            (std::string("err: not a directory: ") + path);

        std::for_each
          (__iter_t(path), __iter_t(),
           [&__files, &hint](const std::filesystem::directory_entry &__p)
           {
             if( std::filesystem::is_regular_file(__p) )
             {
               auto __str = __p.path().filename().string();
               if( hint.size() && __str.substr(0, hint.size()) != hint)
                 return;
               __files.emplace_back(std::move(__str));
             }
           });
        sort(__files.begin(), __files.end());
      }
      else
        throw pasta::err::exception
          (std::string("err: directory does not exist: ") + path);
      return __files;
    }

    // =====================================================================
    inline std::vector<std::string> tokenize_txt(const char *txt_path)
    {
      std::fstream __file;
      io::detail::__open_file_and_check(__file, txt_path);
      auto __n_files = io::detail::__count_lines(__file);

      std::vector<std::string> __result;
      __result.reserve(__n_files);

      std::string __line;
      while( std::getline(__file, __line) )
        if( !__line.empty() ) __result.emplace_back(std::move(__line));

      return __result;
    }


    namespace detail
    {

      // =====================================================================
      inline int __count_bytes(std::fstream &file, int skip)
      {
        file.seekg(0,std::fstream::end);
        int __n_bytes = file.tellg();
        file.seekg(0,std::fstream::beg);
        return __n_bytes-skip;
      }

      // =====================================================================
      inline int __count_lines(std::fstream &file)
      {
        file.seekg(0,std::fstream::beg);

        long __nl = 0;
        std::string __line;
        while(std::getline(file,__line))
          if( !__line.empty() ) ++__nl;

        file.clear(); // Remove eofbit state. getline alwys fail ate.
        file.seekg(0,std::fstream::beg); // Return to file head.
        return __nl;
      }

      // =====================================================================
      template<typename ...Args>
      void __open_file_and_check(std::fstream &file, const std::string &path,
                                 Args &&...tags)
      {
        file.open(path,std::forward<Args>(tags)...);
        if(!file.is_open())
          throw err::exception(std::string("err: opening: ") + path + '\n');
      }

      // =====================================================================
      template<typename T>
      auto __fs_to_uptr(std::fstream &file, const std::string &path,
                        int n_bytes, bool enable_log)
        -> std::pair<long, std::unique_ptr<T[]> >
      {
        auto __ret_pair =
          std::make_pair(static_cast<long>(n_bytes/sizeof(T)),
                         std::unique_ptr<T[]>(new T[n_bytes/sizeof(T)]));
        file.read(reinterpret_cast<char*>(__ret_pair.second.get()),
                  n_bytes);
        if( !file )
          throw err::exception(std::string("err: reading: ") + path + '\n');
        if( enable_log )
          std::cout << "log: " << path << " read to unique_ptr ("
                    << "size = " << n_bytes/sizeof(T) << " elems).\n";
        return __ret_pair;
      }

      // =====================================================================
      template<typename T>
      auto __fs_to_vec(std::fstream &file, const std::string &path,
                       int n_bytes, bool enable_log) -> std::vector<T>
      {
        std::vector<T> __vec(n_bytes/sizeof(T));
        file.read(reinterpret_cast<char*>(__vec.data()),n_bytes);

        if(!file)
          throw err::exception(std::string("err: during reading file: ")
                               + path + '\n');
        if( enable_log )
          std::cout << "log: " << path << " read to std::vector ("
                    << "size = " << n_bytes/sizeof(T) << " elems).\n";
        return __vec;
      }

      // =====================================================================
      template<typename T, int Dim>
      auto __fs_to_mat(std::fstream &file, const std::string &path,
                       int nl, bool enable_log)
        -> Eigen::Matrix<T,Dim,Eigen::Dynamic>
      {
        Eigen::Matrix<T,Dim,Eigen::Dynamic> __mat(Dim,nl);
        int __col = 0; T __tmp; std::string __line;
        while( std::getline(file,__line) )
          if( !__line.empty() )
          {
            auto __row = 0; std::istringstream __iss(__line);
            while(__iss >> __tmp)
            { __mat(__row,__col) = __tmp; ++__row; if( __row > Dim ) break; }
            if(__row > Dim) std::cerr << __row-Dim << " entries discarded.\n";
            if(__row < Dim) std::cerr << Dim-__row << " entries missed.\n";
            ++__col;
          }
        if(enable_log)
          std::cout << "log: " << __col << " " << Dim << "D points read from: "
                    << path << ".\n";
        return __mat;
      }

      // =====================================================================
      template<typename T>
      void __arr_to_fs(T *arr, int arr_size, std::fstream &file,
                       const std::string &path, bool enable_log)
      {
        file.write(reinterpret_cast<char*>(arr),sizeof(T)*arr_size);
        if( !file )
          throw err::exception(std::string("err: writing: ") + path + '\n');
        if( enable_log )
          std::cout << "log: " << path << " writen (size = "
                    << arr_size << " elems).\n";
      }

      // =====================================================================
      template<typename Matrix>
      void __mat_to_fs(std::fstream &file, const std::string &path,
                       Matrix &&mat, int pr, bool enable_log)
      {
        for(int __k = 0; __k < mat.cols(); ++__k)
          file << std::setprecision(pr) << mat.col(__k).transpose() << "\n";

        if( file.fail() )
          throw err::exception(std::string("err: writing: ") + path + '\n');
        if( enable_log )
          std::cout << "log: " << mat.cols() << " " << mat.rows() << "D points"
                    << " writen in: " << path << ".\n";
      }

      // =====================================================================
      inline int __check_filesize(const std::vector<std::string> &names,
                                  int skip)
      {
        int __size = 0;
        for(std::size_t __i = 0; __i < names.size(); ++__i)
        {
          std::fstream __file;
          __open_file_and_check(__file, names[__i], std::fstream::in |
                                std::fstream::binary);

          if( __i == 0 ) __size = __count_bytes(__file, skip);
          else
            if( __size != __count_bytes(__file, skip) )
              throw err::exception(std::string("err: size mimatch: ")
                                   + names[__i] + '\n');
        }
        return __size;
      }

    } //!detail

  } //!io
} //!pasta
