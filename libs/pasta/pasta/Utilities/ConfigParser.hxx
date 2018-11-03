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
// File: ConfigParser.hxx for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:22:58 2018 Zhijin Li
// Last update Sat Nov  3 15:48:21 2018 Zhijin Li
// ---------------------------------------------------------------------------


namespace pasta
{
  namespace utils
  {

    // =====================================================================
    template<typename Key, typename Val>
    ConfigParser<Key,Val>::
    ConfigParser(const char *path, const char head_hint[],
                 const char comment[], const char delimiter[]):
      _config(parse(path,head_hint,comment,delimiter)) {}

    // =====================================================================
    template<typename Key, typename Val>
    ConfigParser<Key,Val>::
    ConfigParser(const char *path, const char comment[],
                 const char delimiter[]):
      _config(parse(path,"\0",comment,delimiter)) {}

    // =====================================================================
    template<typename Key, typename Val>
    auto ConfigParser<Key,Val>::
    parse(const char *path, const char head_hint[], const char comment[],
          const char delimiter[]) -> hasht_t
    {
      std::ifstream __config(path);
      if(!__config.is_open())
        throw pasta::err::exception
          (std::string("err: failed open config file: ")+path+'\n');

      // Reserve buckets for hash table.
      hasht_t __table;
      __table.reserve( count_valid_lines(__config,head_hint,comment) );

      build_table(__table, __config, head_hint, comment, delimiter);
      return __table;
    }

    // =====================================================================
    template<typename Key, typename Val>
    auto ConfigParser<Key,Val>::retrieve(const ktype_t &key) const
      -> const vtype_t&
    {
      auto __pos_itr = _config.find(key);
      if( __pos_itr == _config.end() )
        throw pasta::err::exception
          (std::string("err: non-existent config key: ")+key+'\n');

      return (*__pos_itr).second;
    }

    // =====================================================================
    template<typename Key, typename Val>
    void ConfigParser<Key,Val>::log_entries(int width) const
    {
      int __l = (width%2 == 0) ? (width/2-1) : width/2;
      int __r = width/2;

      std::cout << '\n' << std::string(width,'=') << '\n';
      std::cout << std::string(width,'-') << '\n';

      format_out_pair("LOG: config parsed. # of entries", __l, std::left,
                      _config.size(), __r, std::right);
      std::cout << std::string(width,'-') << "\n\n";

      for(const auto &__pair: _config)
        format_out_pair(__pair.first.c_str(), __l, std::left,
                        __pair.second.c_str(), __r, std::right);

      std::cout << '\n' << std::string(width,'-') << '\n';
      std::cout << std::string(width,'=') << "\n\n";
    }

    // =====================================================================
    template<typename Key, typename Val> template<typename Func>
    void ConfigParser<Key,Val>::
    process_valid_lines(std::ifstream &config, const char head_hint[],
                        const char comment[], Func handler) const
    {
      std::string __line;
      while( std::getline(config, __line) )
      {
        if( std::strcmp(head_hint,"\0") == 0 )
        {
          if( !is_empty(__line) && !is_comment_line(__line, comment) )
            handler(__line);
        }
        else
          if( str_starts_with(__line, head_hint) ) handler(__line);
      }
    }

    // =====================================================================
    template<typename Key, typename Val>
    int ConfigParser<Key,Val>::
    count_valid_lines(std::ifstream &config, const char head_hint[],
                      const char comment[]) const
    {
      long __nlines = 0;
      process_valid_lines(config, head_hint, comment,
                          [&__nlines](const std::string&) { ++__nlines; });

      // Clean up error state returned by process_valid_lines
      // -> Get_line always fails at eof.
      config.clear();                     // Remove eofbit state.
      config.seekg(0,std::ifstream::beg); // Return to file head.

      return __nlines;
    }

    // =====================================================================
    template<typename Key, typename Val>
    void ConfigParser<Key,Val>::
    build_table(hasht_t &table, std::ifstream &config,
                const char head_hint[], const char comment[],
                const char delimiter[]) const
    {
      process_valid_lines
        ( config, head_hint, comment,
          [this, &table, &comment, &delimiter] (std::string __line)
          {
            strip_line(__line, comment);

            auto __delim_pos = check_format(__line,delimiter);
            auto __key       = boost::lexical_cast<Key>
              (extract_pre(__line, __delim_pos));
            check_duplicate(table, __key);

            table.emplace
              ( std::move(__key),
                boost::lexical_cast<Val>(extract_post(__line,__delim_pos)) );
          });
    }

    // =====================================================================
    template<typename Key, typename Val>
    auto ConfigParser<Key,Val>::
    check_format(const std::string &line, const char delimiter[])
      const -> strsz_t
    {
      auto __delim_pos = line.find(delimiter);
      // If delimiter found at the begining/end or delimiter followed by noth.
      if(  __delim_pos == std::string::npos || // Must be found.
           __delim_pos == 0 ||                 // Not at the beginning
           line.size() <= __delim_pos+1 )      // At least tail chars follows.
        throw err::exception
          (std::string("err: config line wrong format: ")+line+'\n');

      return __delim_pos;
    }

    // =====================================================================
    template<typename Key, typename Val>
    void ConfigParser<Key,Val>::
    check_duplicate(const hasht_t &table, const Key &key) const
    {
      if( table.find(key) != std::end(table) )
        throw err::exception(std::string("err: duplicated entry: ") +
                             boost::lexical_cast<std::string>(key) + '\n');
    }

    // =====================================================================
    template<typename Key, typename Val>
    void ConfigParser<Key,Val>::
    strip_line(std::string &line, const char comment[]) const
    {
      rm_comment(line,comment);
      rm_spaces(line);
    }

    // =====================================================================
    template<typename Key, typename Val>
    void ConfigParser<Key,Val>::
    rm_comment(std::string &line, const char comment[]) const
    {
      auto __pos = line.find(comment);
      if( __pos != std::string::npos ) line.erase(__pos,std::string::npos);
    }

    // =====================================================================
    template<typename Key, typename Val>
    bool ConfigParser<Key,Val>::
    is_comment_line(const std::string &line, const char comment[]) const
    {
      std::string __cmnt(comment);
      return line.substr(line.find_first_not_of(' '),__cmnt.size()) ==
        __cmnt;
    }


    // =====================================================================
    template<typename Key, typename Val, typename Adj>
    inline void
    format_out_pair(const Key &key, int key_len, Adj key_rule,
                    const Val &val, int val_len, Adj val_rule,
                    char sep, int sep_len, Adj sep_rule)
    {
      std::cout.width(key_len); std::cout << key_rule << key;
      std::cout.width(sep_len); std::cout << sep_rule << sep;
      std::cout.width(val_len); std::cout << val_rule << val << '\n';
    }

    // =====================================================================
    inline void rm_spaces(std::string &str)
    {
      str.erase( std::remove_if(std::begin(str), std::end(str),
                                ::isspace), std::end(str) );
    }

    // =====================================================================
    inline bool is_empty(const std::string &str)
    {
      return str.empty() ||
        (str.find_first_not_of(' ') == std::string::npos);
    }

    // =====================================================================
    inline bool
    str_starts_with(const std::string &str, const char head_hint[])
    {
      std::string __head_str(head_hint);
      auto __nonspace_pos = str.find_first_not_of(' ');

      return ( str.size() >= (__nonspace_pos + __head_str.size()) )
        && str.compare(__nonspace_pos, __head_str.size(), __head_str) == 0;
    }

    // =====================================================================
    inline std::string
    extract_pre(const std::string &str, std::string::size_type pos)
    { return str.substr(0,pos); }

    // =====================================================================
    inline std::string
    extract_post(const std::string &str, std::string::size_type pos)
    { return str.substr(pos+1,str.size()-pos-1); }

    // =====================================================================
    inline std::vector<std::string>
    tokenize(const std::string &str, char pipe)
    {
      std::size_t __ntokens = std::count(str.begin(), str.end(), pipe);
      std::vector<std::string> __res;
      __res.reserve(__ntokens+1);

      size_t __head = 0;
      size_t __tail = str.find(pipe, __head);
      while( __tail != std::string::npos )
      {
        __res.emplace_back(std::move(str.substr(__head,__tail-__head)));
        __head = __tail + 1;
        __tail = str.find(pipe, __head);
      }
      __res.emplace_back(std::move(str.substr(__head)));
      return __res;
    }

    // =====================================================================
    template<typename Scalar>
    auto make_rv(const std::vector<std::string> &tokens)
      -> std::variant<Scalar, rnd::RUniform<Scalar>,
                 rnd::Gaussian<Scalar> >
    {
      std::variant<Scalar, rnd::RUniform<Scalar>,
                     rnd::Gaussian<Scalar> > __rv;
      if( tokens[0] == tags::gaussian_tag )
        __rv = resolve_gaussian<Scalar>(tokens);
      else if( tokens[0] == tags::runiform_tag )
        __rv = resolve_runiform<Scalar>(tokens);
      else if( tokens[0] == tags::deterministic_tag )
        __rv = resolve_deterministic<Scalar>(tokens);
      return __rv;
    }

    // =====================================================================
    template<typename Scalar, typename BeginItr, typename EndItr>
    auto make_rv(BeginItr token_begin, EndItr token_end)
      -> std::variant<Scalar, rnd::RUniform<Scalar>,
                        rnd::Gaussian<Scalar> >
    {
      std::variant<Scalar, rnd::RUniform<Scalar>,
                     rnd::Gaussian<Scalar> > __rv;
      if( (*token_begin) == tags::gaussian_tag )
        __rv = resolve_gaussian<Scalar>(std::next(token_begin), token_end);
      else if( (*token_begin) == tags::runiform_tag )
        __rv = resolve_runiform<Scalar>(std::next(token_begin), token_end);
      else if( (*token_begin) == tags::deterministic_tag )
        __rv = resolve_deterministic<Scalar>(std::next(token_begin), token_end);
      return __rv;
    }

    // =====================================================================
    template<typename Scalar>
    auto resolve_gaussian(const std::vector<std::string> &tokens)
      -> rnd::Gaussian<Scalar>
    {
      if( tokens.size() != 3 )
        throw err::exception(std::string("wrong args: ")+tokens[0]);

      return rnd::Gaussian<Scalar>
        (boost::lexical_cast<Scalar>(tokens[1]),
         boost::lexical_cast<Scalar>(tokens[2]));
    }

    // =====================================================================
    template<typename Scalar, typename BeginItr, typename EndItr>
    auto resolve_gaussian(BeginItr token_begin, EndItr token_end)
      -> rnd::Gaussian<Scalar>
    {
      if( (token_end - token_begin) != 2 )
        throw err::exception(std::string("wrong args for: ")+(*token_begin));

      return rnd::Gaussian<Scalar>
        (boost::lexical_cast<Scalar>(*token_begin),
         boost::lexical_cast<Scalar>(*(std::next(token_begin))));
    }

    // =====================================================================
    template<typename Scalar>
    auto resolve_runiform(const std::vector<std::string> &tokens)
      -> rnd::RUniform<Scalar>
    {
      if( tokens.size() != 3 )
        throw err::exception(std::string("wrong args: ")+tokens[0]);

      return rnd::RUniform<Scalar>
        (boost::lexical_cast<Scalar>(tokens[1]),
         boost::lexical_cast<Scalar>(tokens[2]));
    }

    // =====================================================================
    template<typename Scalar, typename BeginItr, typename EndItr>
    auto resolve_runiform(BeginItr token_begin, EndItr token_end)
      -> rnd::RUniform<Scalar>
    {
      if( (token_end - token_begin) != 2 )
        throw err::exception(std::string("wrong args for: ")+(*token_begin));

      return rnd::RUniform<Scalar>
        (boost::lexical_cast<Scalar>(*token_begin),
         boost::lexical_cast<Scalar>(*(std::next(token_begin))));
    }

    // =====================================================================
    template<typename Scalar>
    Scalar resolve_deterministic(const std::vector<std::string> &tokens)
    {
      if( tokens.size() != 2 )
        throw err::exception(std::string("wrong args: ")+tokens[0]);

      return boost::lexical_cast<Scalar>(tokens[1]);
    }

    // =====================================================================
    template<typename Scalar, typename BeginItr, typename EndItr>
    Scalar resolve_deterministic(BeginItr token_begin, EndItr token_end)
    {
      if( (token_end - token_begin) != 1 )
        throw err::exception(std::string("wrong args for: ")+(*token_begin));

      return boost::lexical_cast<Scalar>(*token_begin);
    }

    // =====================================================================
    template<typename Scalar, int Dim>
    auto parse_point(const std::vector<std::string> &tokens)
      -> Eigen::Matrix<Scalar,Dim,1>
    {
      return detail::parse_point_impl<Scalar,Dim>(tokens, make_seq_t<Dim>{});
    }

    // =====================================================================
    template<typename Scalar, int Dim, typename InputItr>
    Eigen::Matrix<Scalar,Dim,1> parse_point(InputItr in)
    {
      return detail::parse_point_impl<Scalar,Dim>(in, make_seq_t<Dim>{});
    }

    // =====================================================================
    template<typename Scalar, int Dim>
    auto parse_list_as_pt(const std::string &token, char sep)
      -> Eigen::Matrix<Scalar,Dim,1>
    {
      assert( token.size() && "An empty string provided." );
      assert( std::count(token.begin(), token.end(), ',') + 1 == Dim &&
              "List size and point dimension mismatch" );

      Eigen::Matrix<Scalar,Dim,1> __pt;
      detail::parse_list_as_pt_impl<Scalar>(token, __pt, sep);
      return __pt;
    }

    // =====================================================================
    template<typename Scalar>
    auto parse_list_as_pt(const std::string &token, char sep)
      -> Eigen::Matrix<Scalar,Eigen::Dynamic,1>
    {
      assert( token.size() && "An empty string provided." );

      int __dim = std::count(token.begin(), token.end(), ',') + 1;
      Eigen::Matrix<Scalar,Eigen::Dynamic,1> __pt(__dim);
      detail::parse_list_as_pt_impl<Scalar>(token, __pt, sep);
      return __pt;
    }

    namespace detail
    {

      // =====================================================================
      template<typename Scalar, int Dim, int ...Indx>
      auto parse_point_impl(const std::vector<std::string> &tokens,
                            indx_seq<Indx...>)
        -> Eigen::Matrix<Scalar,Dim,1>
      {
        return pasta::utils::make_eigen_pt<Scalar>
          (boost::lexical_cast<Scalar>(tokens[Indx])...);
      }

      // =====================================================================
      template<typename Scalar, int Dim, typename InputItr, int ...Indx>
      auto parse_point_impl(InputItr in, indx_seq<Indx...>)
        -> Eigen::Matrix<Scalar,Dim,1>
      {
        return pasta::utils::make_eigen_pt<Scalar>
          ( boost::lexical_cast<Scalar>(*(std::next(in, Indx)))... );
      }

      // =====================================================================
      template<typename Scalar, typename PointType>
      void parse_list_as_pt_impl(const std::string &token, PointType &pt,
                                 char sep)
      {
        size_t __head = 1;
        for( int __n = 0; __n < pt.size()-1; ++__n)
        {
          int __tail = token.find(sep, __head);
          pt(__n) = boost::lexical_cast<Scalar>
            ( token.substr(__head, __tail - __head) );
          __head = __tail + 1;
        }
        pt(pt.size()-1) = boost::lexical_cast<Scalar>
          ( token.substr(__head, token.size()-1-__head) );
      }
    }

  }
}
