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
// File: utils.hxx for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:27:38 2018 Zhijin Li
// Last update Sat Nov  3 23:31:56 2018 Zhijin Li
// ---------------------------------------------------------------------------


namespace pasta
{
  namespace utils
  {

    // =====================================================================
    inline auto set_timer_ms() -> decltype
      (std::chrono::duration_cast<std::chrono::milliseconds>
       (std::chrono::system_clock::now().time_since_epoch()).count())
    {
      return std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::system_clock::now().time_since_epoch()).count();
    }

    // =====================================================================
    inline timer::timer(const char *name):
      _name(name)
    {
      std::cout << "-------> pasta timer [" << name << "] set.\n";
      _timestamp = set_timer_ms();
    }

    // =====================================================================
    inline void timer::tic()
    {
      std::cout << "-------> pasta timer [" << _name << "] reset.\n";
      _timestamp = set_timer_ms();
    }

    // =====================================================================
    inline void timer::toc() const
    {
      std::cout << "-------> pasta timer [" << _name << "] elapses: "
                << (set_timer_ms() - _timestamp) << " ms.\n";
    }

    // =====================================================================
    inline void loadbar(int ind, int total, int width)
    {
      width -= 3;
      if( (ind!=total) && (ind%(total/100+1)!=0) ) return;
      else if( ind != total )
      {
        float __ratio = ind/static_cast<float>(total);
        int __count =  __ratio * width;

        std::cout << std::setw(3) << static_cast<int>(__ratio*100) << "% [";
        for (int __xx = 0; __xx < __count; ++__xx) std::cout << "=";
        for (int __xx = __count; __xx < width; ++__xx) std::cout << " ";
        std::cout << "]\r" << std::flush;
      }else
      {
        std::cout << std::setw(3) << "100% [";
        for (int __xx = 0; __xx < width; ++__xx) std::cout << "=";
        std::cout << "]" << std::endl;
      }
    }

    // =====================================================================
    template <int N, typename T>
    std::array<T, N> make_array(const T& value)
    {
      return detail::make_array_impl(value, make_seq_t<N>{});
    }

    // =====================================================================
    template<typename T, typename... TS>
    auto inline make_unique(TS &&...args) -> std::unique_ptr<T>
    {
      return std::unique_ptr<T>(new T(std::forward<TS>(args)...));
    }

    // =====================================================================
    inline std::mt19937_64& shared_engine(std::mt19937_64::result_type seed)
    {
      static std::mt19937_64 __engine(seed);
      return __engine;
    }

    // =====================================================================
    inline void reset_shared_engine(std::mt19937_64::result_type seed)
    {
      auto& __engine = shared_engine();
      __engine.seed(seed);
    }

    // =====================================================================
    template<typename ScalrType, int Dim, typename T,
             std::enable_if_t<Dim!=1>*>
    inline auto make_const_pt(T val) -> Eigen::Matrix<ScalrType,Dim,1>
    {
      return Eigen::Matrix<ScalrType,Dim,1>::Constant(val);
    }

    // =====================================================================
    template<typename ScalrType, int Dim, typename T,
             std::enable_if_t<Dim==1>*>
    inline auto make_const_pt(T val) -> ScalrType
    {
      return val;
    }

    // =====================================================================
    template<typename Dst, typename Src,
             std::enable_if_t<std::is_arithmetic_v<Src> >*>
    inline auto cast_to(Src &&src)
      -> decltype(static_cast<Dst>(std::declval<Src>()))
    {
      return static_cast<Dst>(std::forward<Src>(src));
    }

    // =====================================================================
    template<typename Dst, typename Src,
             enable_if_all_t<is_eigen_v<Src>,
                             !std::is_same_v<Dst,eigen_val_t<Src> > >*>
    inline auto cast_to(Src &&src)
      -> decltype(src.template cast<Dst>())
    {
      return std::forward<Src>(src).template cast<Dst>();
    }

    // =====================================================================
    template<typename Dst, typename Src,
             enable_if_all_t<is_eigen_v<Src>,
                             std::is_same_v<Dst,eigen_val_t<Src> > >*>
    auto cast_to(Src &&src) -> decltype(std::forward<Src>(src))
    {
      return std::forward<Src>(src);
    }

    // =====================================================================
    template<typename T, typename... TS,
             enable_if_all_t<(sizeof...(TS) <= 4 && sizeof...(TS) > 1),
                             std::is_arithmetic_v<TS>...>*>
    inline auto make_eigen_pt(TS ...args) -> Eigen::Matrix<T,sizeof...(TS),1>
    {
      return Eigen::Matrix<T,sizeof...(TS),1>{static_cast<T>(args)...};
    }

    // =====================================================================
    template<typename T, typename... TS,
             enable_if_all_t<(sizeof...(TS) > 4 || sizeof...(TS) == 1),
             std::is_arithmetic_v<TS>...>*>
    inline auto make_eigen_pt(TS ...args) -> Eigen::Matrix<T,sizeof...(TS),1>
    {
      std::initializer_list<T> __list{static_cast<T>(args)...};
      Eigen::Matrix<T,sizeof...(TS),1> __pt;
      for(auto &__el: __list) __pt(&__el - std::begin(__list)) = __el;
      return __pt;
    }

    // =====================================================================
    template<typename T, typename Input, typename>
    inline auto make_eigen_pt(Input &&input)
      -> decltype(cast_to<T>(std::forward<Input>(input)))
    {
      return cast_to<T>(std::forward<Input>(input));
    }

    // =====================================================================
    template<typename T, typename Scalar>
    auto make_eigen_pt(std::initializer_list<Scalar> input)
      -> Eigen::Matrix<T,Eigen::Dynamic,1>
    {
      Eigen::Matrix<T,Eigen::Dynamic,1> __result(input.size());
      for(auto &__elem : input)
        __result(&__elem-std::begin(input)) = static_cast<T>(__elem);
      return __result;
    }

    // =====================================================================
    template<typename T,
             enable_if_all_t<is_eigen_v<T>, dim_dispatch_v<T>()==1>*>
    inline auto elem_at(T &&input, int index) -> decltype(input(0))
    {
      return input(index);
    }

    // =====================================================================
    template<typename T,
             enable_if_all_t<is_eigen_v<T>, dim_dispatch_v<T>()!=1>*>
    inline auto elem_at(T &&input, int index) -> decltype(input.col(0))
    {
      return input.col(index);
    }

    // =====================================================================
    template<typename T, typename Alloc, typename Vec>
    void logical_slice(std::vector<T,Alloc> &input, Vec &&lv,
                       value_dispatch_t<Vec> keep_val, bool shrink_after)
    {
      assert(static_cast<int>(lv.size()) == static_cast<int>(input.size())
             && "logical vec size mismatch.");
      input.erase(std::remove_if(std::begin(input), std::end(input),
                               [&lv,&input,keep_val](const T &elem)
                               {
                                 return value_at(lv,&elem-&input[0])
                                   != keep_val;
                               } ), std::end(input));
      if(shrink_after) input.shrink_to_fit();
    }

    // =====================================================================
    template<typename T, typename Vec, int Dim, std::enable_if_t<Dim!=1>*>
    void logical_slice(Eigen::Matrix<T,Dim,Eigen::Dynamic> &input, Vec &&lv,
                       value_dispatch_t<Vec> keep_val)
    {
      assert(lv.size() == input.cols() && "logical vec size mismatch.");
      auto __l = 0;
      for(auto __k = 0; __k < input.cols(); ++__k)
        if( value_at(lv,__k) == keep_val )
        { if(__k > __l) { input.col(__l) = input.col(__k); } ++__l; }
      input.conservativeResize(Dim, __l);
    }

    // =====================================================================
    template<typename T, typename Vec, int Dim, std::enable_if_t<Dim!=1>*>
    void logical_slice(Eigen::Matrix<T,Eigen::Dynamic,Dim> &input, Vec &&lv,
                       value_dispatch_t<Vec> keep_val)
    {
      assert(lv.size() == input.rows() && "logical vec size mismatch.");
      auto __l = 0;
      for(auto __k = 0; __k < input.rows(); ++__k)
        if( value_at(lv,__k) == keep_val )
        { if(__k > __l) { input.row(__l) = input.row(__k); } ++__l; }
      input.conservativeResize(__l,Dim);
    }

    // =====================================================================
    template<typename T, typename Vec, int Rows, int Cols, typename>
    void logical_slice(Eigen::Matrix<T,Rows,Cols> &input, Vec &&lv,
                       value_dispatch_t<Vec> keep_val)
    {
      assert(static_cast<int>(lv.size()) == static_cast<int>(input.size())
             && "logical vec size mismatch.");
      auto __l = 0;
      for(auto __k = 0; __k < input.size(); ++__k)
        if( value_at(lv,__k) == keep_val )
        { if(__k > __l) { input(__l) = input(__k); } ++__l; }
      input.conservativeResize(__l);
    }

    // =====================================================================
    template<typename Mat, typename>
    auto bind_cols(const std::vector<Mat> &mat_vec)
      -> mutate_col_t<Mat,Eigen::Dynamic>
    {
      constexpr int __dim = eigen_rows_v<Mat>();

      int __cols = 0;
      for(const auto &__el: mat_vec) __cols += __el.cols(); // Get size first.

      int __col_counter = 0;
      mutate_col_t<Mat,Eigen::Dynamic> __binded(__dim, __cols);
      for(const auto &__el: mat_vec)
      {
        __binded.block(0,__col_counter,__dim,__el.cols()) = __el;
        __col_counter += __el.cols();
      }
      return __binded;
    }

    // =====================================================================
    template<typename T, int Dim>
    void remove_elem(Eigen::Matrix<T,Dim,Eigen::Dynamic> &input, int indx)
    {
      assert(indx < input.cols() && "error: indx out of bound");
      for(auto __k = indx+1; __k < input.cols(); ++__k)
        input.col(indx++) = input.col(__k);
      input.conservativeResize(Dim, input.cols()-1);
    }

    // =====================================================================
    template<typename T, typename Alloc>
    inline void remove_elem(std::vector<T,Alloc> &input, std::size_t indx)
    {
      assert(indx < input.size() && "error: indx out of bound");
      input.erase(input.begin()+indx);
    }

    // =====================================================================
    template<typename T, typename Alloc>
    inline auto ref_as_eigen(std::vector<T,Alloc> &input)
      -> Eigen::Map<Eigen::Matrix<T,Eigen::Dynamic,1> >
    {
      return Eigen::Map<Eigen::Matrix<T,-1,1> >(input.data(), input.size());
    }

    // =====================================================================
    template<typename T, std::size_t N>
    inline auto ref_as_eigen(std::array<T,N> &input)
      -> Eigen::Map<Eigen::Matrix<T,N,1> >
    {
      return Eigen::Map<Eigen::Matrix<T,N,1> >(input.data());
    }

    // =====================================================================
    template<typename Input, std::enable_if_t<is_eigen_v<Input> >*>
    inline auto ref_as_eigen(Input &&input) -> decltype(std::forward<Input>(input))
    {
      return std::forward<Input>(input);
    }

    namespace detail
    {

      // =====================================================================
      template <typename T, int ...I>
      std::array<T, sizeof...(I)>
      make_array_impl(const T& value, indx_seq<I...>)
      {
        return std::array<T, sizeof...(I)>{ ((void)I, value)... };
      }

    }

  }
}
