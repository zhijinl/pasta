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
// File: ConfigParser.hh for pasta
//
// Created by Zhijin Li
// E-mail:   <jonathan.zj.lee@gmail.com>
//
// Started on  Fri Nov  2 15:22:35 2018 Zhijin Li
// Last update Sat Nov  3 15:48:21 2018 Zhijin Li
// ---------------------------------------------------------------------------


#ifndef PASTA_CONFIGPARSER_HH
# define PASTA_CONFIGPARSER_HH

# include <cctype>
# include <fstream>
# include <iterator>
# include <unordered_map>
# include "pasta/core.hh"
# include "pasta/distributions.hh"
# include "pasta/Utilities/utils.hh"
# include "boost/lexical_cast.hpp"


namespace pasta
{
  /// @ingroup group_utils
  namespace utils
  {

    /// @ingroup group_utils
    ///
    /// @brief Class managing config file parsing and params extraction.
    ///
    ///
    ///
    /// @param Key: type of the key, usually it's `std::string`.
    /// @param Val: type of the value, it is common (and recommanded) to
    /// use `std::string` as the value type, then use the
    /// `ConfigParser::retrieve()` interface with **lexical cast** to get
    /// numeric value related to a key.
    ///
    template<typename Key, typename Val> class ConfigParser
    {
    public:

      using ktype_t = Key;
      using vtype_t = Val;
      using strsz_t = std::string::size_type;
      using hasht_t = std::unordered_map<Key,Val>;

      /// @brief Ctor. Initialize a mbd_config with head_hint + comment
      /// + delimiter.
      ///
      /// Head hint can be used to filter lines with specific starting
      /// sequence.
      ///
      /// @param path: the path to the config_file.
      /// @param head_hint: the head_hint_tag that all config line starts with.
      /// @param comment: char specifying the comment char.
      /// @param delimiter: the delimiter char separating Key/Value.
      ///
      ConfigParser(const char *path, const char head_hint[],
                   const char comment[], const char delimiter[]);

      /// @brief Ctor. Initialize a mbd_config with comment char + delimiter.
      ///
      /// @param path: the path to the config_file.
      /// @param comment: char specifying the comment char.
      /// @param delimiter: the delimiter char separating Key/Value.
      ///
      ConfigParser(const char *path, const char comment[],
                   const char delimiter[]);


      /// @brief Parse a config file.
      ///
      /// @param path: path of the config file.
      /// @param head_hint: the head_hint_tag that all config line starts with.
      /// @param comment: char specifying the comment char.
      /// @param delimiter: the delimiter char separating Key/Value.
      /// @return A hash tabel of all parsed <key,val> pairs.
      ///
      hasht_t parse(const char*path, const char head_hint[],
                    const char comment[], const char delimiter[]);

      /// @brief Const access the internal hash table (unordered_map).
      ///
      /// @return: the internal hash table (unordered_map).
      ///
      const hasht_t& retrieve() const { return _config; }

      /// @brief Const access element with Key inside internal unordered_map.
      ///
      /// @param key: the key value.
      /// @return: the found value. out_of_range is thrown in case of
      /// non existence.
      ///
      const vtype_t& retrieve(const ktype_t &key) const;

      /// @brief Lexical const access. With lexical cast.
      ///
      /// Need to specify TargType for lexical cast.
      /// @param key: the key value.
      /// @return: the found value. out_of_range is thrown in case of
      /// non existence.
      ///
      template<typename TargType>
      TargType retrieve(const ktype_t &key) const
      { return boost::lexical_cast<TargType>(retrieve(key)); }

      /// @brief Log config entries (in random order).
      ///
      /// @param width: desired total width.
      ///
      void log_entries(int width=100) const;

    private:

      /// @brief Process valid lines with a functional handler.
      ///
      /// A valid line is non-empty, non-pure comment and meets the
      /// `Key [delimiter] Val` format.
      ///
      /// @param config: the open config file `fstream`.
      /// @param head_hint: the input starting sequence.
      /// @param comment: the comment sequence.
      /// @param delimiter: the input delimiter.
      /// @param handler: a functional handler, could be a lambda or
      /// a functor.
      ///
      template<typename Func>
      void
      process_valid_lines(std::ifstream &config, const char head_hint[],
                          const char comment[], Func handler) const;

      /// @brief Count number of valid lines in config file.
      ///
      /// @param head_hint: the input starting sequence.
      /// @param comment: the comment sequence.
      /// @param delimiter: the input delimiter.
      /// @return The counted number of valid lines.
      ///
      int
      count_valid_lines(std::ifstream &config, const char head_hint[],
                        const char comment[]) const;

      /// @brief Build hash table: fill table with Key & Val pairs.
      ///
      /// @param table: the hash table.
      /// @param config: the open config file `fstream`.
      /// @param head_hint: the input starting sequence.
      /// @param comment: the comment sequence.
      /// @param delimiter: the input delimiter.
      ///
      void
      build_table(hasht_t &table, std::ifstream &config,
                  const char head_hint[], const char comment[],
                  const char delimiter[]) const;

      /// @brief Check if line meets `Key [delimiter] Val` format.
      ///
      /// Pre-condition:
      /// 1. String already passed `str_starts_with` or `!is_empty` and
      /// `!is_comment_line` test.
      /// 2. String is trimmed off spaces & comments.
      ///
      /// Predicate: (syntax [Key][delimiter][Val])
      /// 1. delimiter must be found in the line.
      /// 2. delimiter is not at the beginning of the line.
      /// 3. At least 1 char is followed after delimiter.
      ///
      /// @param line: a string line to be examined.
      /// @param delimiter: the input delimiter.
      /// @return The position index of the delimiter.
      ///
      strsz_t check_format(const std::string &line,
                           const char delimiter[]) const;

      /// @brief Check if an input is a duplicate.
      ///
      /// @param table: a hash table where the key will be checked.
      /// @param key: the input key to check.
      ///
      void check_duplicate(const hasht_t &table, const Key &key) const;

      /// @brief Pre-process a string line.
      ///
      /// Trim off comment regions and spaces.
      ///
      /// @param line: an input string line.
      /// @param comment: the comment char to strip off.
      ///
      void strip_line(std::string &, const char []) const;

      /// @brief Remove comment sections.
      ///
      /// Remove from position of a comment symbol to the end of line.
      ///
      /// @param line: a string containing line content.
      /// @param symbol: char of comment identifier.
      ///
      void rm_comment(std::string &line, const char []) const;

      /// @brief Check if a line is pure comment line.
      ///
      /// @param line: current line.
      /// @param comment: comment sequence.
      /// @return A bool, true if curr line is pure comment.
      ///
      inline bool is_comment_line(const std::string &line,
                                  const char comment[]) const;

      hasht_t _config;
    };


    /// @ingroup group_utils
    ///
    /// @brief Some constant identifiers for pasta types.
    ///
    namespace tags
    {
      constexpr auto gaussian_tag = "GAUSSIAN";
      constexpr auto runiform_tag = "RUNIFORM";
      constexpr auto deterministic_tag = "DETERMINISTIC";
    }


    /// @brief Formatted output of <key sep val> pair,
    /// such as 'PARAMETER_A = 42.0'.
    ///
    /// @param key: key token.
    /// @param key_len: key length.
    /// @param key_rule: adjustment rule for key.
    /// @param val: val token.
    /// @param val_len: val length.
    /// @param val_rule: adjustment rule for val.
    /// @param sep: sep token.
    /// @param sep_len: sep length.
    /// @param sep_rule: adjustment rule for sep.
    ///
    template<typename Key, typename Val, typename Adj=decltype(std::left)>
    inline void
    format_out_pair(const Key &key, int key_len, Adj key_rule,
                    const Val &val, int val_len, Adj val_rule,
                    char sep='=', int sep_len=1, Adj sep_rule=std::left);

    /// @ingroup group_utils
    ///
    /// @brief Remove all spaces chars.
    ///
    /// Remove all spaces-like chars (including '\t') in an input str.
    ///
    /// @param str: the input / output string.
    ///
    void rm_spaces(std::string &str);

    /// @ingroup group_utils
    ///
    /// @brief Check if the input str is empty or only has whitespaces.
    ///
    /// @param str: the input string.
    /// @return A bool, true if the string is empty.
    ///
    bool is_empty(const std::string &str);

    /// @ingroup group_utils
    ///
    /// @brief Check if an input string strats with a specific sequence.
    ///
    /// Test:
    /// 1. If the current line is empty, test fails directly.
    /// 2. Then check if the line starts with specific head_hint sequence.
    ///
    /// @note The space characters in the beginning of the line are
    /// **skipped**, if there is any.
    ///
    /// @param str: an input string to be examined.
    /// @param head_hint: the desired starting sequence.
    /// @return The boolean check result.
    ///
    bool str_starts_with(const std::string &str, const char head_hint[]);

    /// @ingroup group_utils
    ///
    /// @brief Extract chars prior to a position inside the input string
    /// and return a sub string.
    ///
    /// @note Counted from the beginning until the charactor **before the
    /// input position**, therefore the char at input position is **not
    /// included**.
    ///
    /// @param line: the curr input string.
    /// @param pos: desired position.
    /// @return The extracted sub string.
    ///
    std::string
    extract_pre(const std::string &str, std::string::size_type pos);
    /// @ingroup group_utils
    /// @brief Extract chars after a position inside the input string and
    /// return a sub string.
    ///
    /// @note Counted from the the charactor **after the input position**,
    /// to the end of the string, therefore the char at input position is
    /// **not included**.
    ///
    /// @param str: the curr input string.
    /// @param pos: desired position.
    /// @return The extracted sub string.
    ///
    std::string
    extract_post(const std::string &str, std::string::size_type pos);

    /// @ingroup group_utils
    ///
    /// @brief Tokenize a piped string.
    ///
    /// Break a string into pieces delimited by specified pipe charactor.
    /// Then transform them into a vector of strings.
    ///
    /// For example, when the pipe char is `|`, the following string:
    /// "abc|def|ghi|jkl"
    /// will be tokenized into an `std::vector` with size 4, containing:
    /// "abc", "def", "ghi" and "jkl".
    ///
    /// @warning This function assumes that all spaces chars are stripped
    /// in the input str. So a call to `pasta::utils::rm_spaces` before
    /// hand might be preferred **if the input string contains spaces**.
    ///
    /// @param str: the input string to be tokenized.
    /// @param pipe: the input pipe charactor.
    /// @return A `std::vector<std::string>` containing tokenized words.
    ///
    /// @sa `pasta::utils::rm_spaces`.
    ///
    std::vector<std::string> tokenize(const std::string &str, char pipe='|');

    ///@{
    /// @brief Create rand var from input tokens.
    ///
    /// @param tokens: input tokens, `std::vector<std::string>`.
    /// @return A `std::variant` of random variables.
    ///
    template<typename Scalar>
    auto make_rv(const std::vector<std::string> &tokens)
      -> std::variant<Scalar, rnd::RUniform<Scalar>,
                        rnd::Gaussian<Scalar> >;

    template<typename Scalar, typename BeginItr, typename EndItr>
    auto make_rv(BeginItr token_begin, EndItr token_end)
      -> std::variant<Scalar, rnd::RUniform<Scalar>,
                        rnd::Gaussian<Scalar> >;
    ///@}

    /// @brief Resolve input tokens to a Gaussian random variable.
    ///
    /// @param tokens: the input tokens, `std::vector<std::string>`. It
    /// is expected to have syntax such as `GAUSSIAN | 1.0 | 0.1`.
    /// @return A Gaussian random variable.
    ///
    template<typename Scalar>
    auto resolve_gaussian(const std::vector<std::string> &tokens)
      -> rnd::Gaussian<Scalar>;

    template<typename Scalar, typename BeginItr, typename EndItr>
    auto resolve_gaussian(BeginItr token_begin, EndItr token_end)
      -> rnd::Gaussian<Scalar>;

    /// @brief Resolve input tokens to a real uniform random variable.
    ///
    /// @param tokens: the input tokens, `std::vector<std::string>`. It
    /// is expected to have syntax such as `RUNIFORM | 1.0 | 20.0`.
    /// @return A Gaussian real uniform variable.
    ///
    template<typename Scalar>
    auto resolve_runiform(const std::vector<std::string> &tokens)
      -> rnd::RUniform<Scalar>;

    template<typename Scalar, typename BeginItr, typename EndItr>
    auto resolve_runiform(BeginItr token_begin, EndItr token_end)
      -> rnd::RUniform<Scalar>;

    /// @brief Resolve input tokens to a deterministic value.
    ///
    /// @param tokens: the input tokens, `std::vector<std::string>`. It
    /// is expected to have syntax such as `DETERMINISTIC | 1.0`.
    /// @return A deterministic value.
    ///
    template<typename Scalar>
    Scalar resolve_deterministic(const std::vector<std::string> &tokens);

    template<typename Scalar, typename BeginItr, typename EndItr>
    Scalar resolve_deterministic(BeginItr token_begin, EndItr token_end);

    /// @brief Parse Eigen point from input tokens.
    ///
    /// @param token: input tokens.
    /// @return Eigen point.
    ///
    template<typename Scalar, int Dim>
    auto parse_point(const std::vector<std::string> &tokens)
      -> Eigen::Matrix<Scalar,Dim,1>;

    /// @brief Parse Eigen point from input tokens.
    ///
    /// @param token: input tokens begin iterator.
    /// @return Eigen point.
    ///
    template<typename Scalar, int Dim, typename InputItr>
    Eigen::Matrix<Scalar,Dim,1> parse_point(InputItr in);

    ///@{
    /// @brief Parse an input string token representing a
    /// numeric list to en Eigen column vec.
    ///
    /// @note The Format of the list is fully customizable. The format
    /// is defined by three elements:
    /// - A `beg_id`, which is a character that defines the start of
    ///   a list. It can be any non-null character. Commonly used
    ///   characters are '[' and `{`.
    /// - An `end_id`, which is a character that defines the end of
    ///   a list. It can be any non-null character, not necessarily the
    ///   same as `beg_id`. Commonly used characters are '[' and `{`.
    /// - A `sep`, which is a character that seperates different fields
    ///   of a list. By default the function uses `,`.
    /// A valid list tokens should would look
    /// like: `{1.0, 2.0, 3.0}`.
    ///
    /// @warning This function assumes that all spaces chars are stripped
    /// in the input str. So a call to `pasta::utils::rm_spaces` before
    /// hand might be preferred **if the input string contains spaces**.
    ///
    /// @param token: the input string token.
    /// @param sep: the list seperator character. Defaults to `,`.
    /// @return The parse Eigen column vector. It will be a fixed size
    /// `Eigen` column vector if the template parameter `Dim` is
    /// specified. Otherwise it will be a dynamic `Eigen` column vector.
    ///
    template<typename Scalar, int Dim>
    auto parse_list_as_pt(const std::string &token, char sep=',')
      -> Eigen::Matrix<Scalar,Dim,1>;

    template<typename Scalar>
    auto parse_list_as_pt(const std::string &token, char sep=',')
      -> Eigen::Matrix<Scalar,Eigen::Dynamic,1>;
    ///@}

    namespace detail
    {

      /// @brief Unpack tokens of numbers into an Eigen column vec.
      ///
      template<typename Scalar, int Dim, int ...Indx>
      auto parse_point_impl(const std::vector<std::string> &tokens,
                            indx_seq<Indx...>)
        -> Eigen::Matrix<Scalar,Dim,1>;

      /// @brief Unpack tokens of numbers into an Eigen column vec.
      ///
      template<typename Scalar, int Dim, typename InputItr, int ...Indx>
      auto parse_point_impl(InputItr in, indx_seq<Indx...>)
        -> Eigen::Matrix<Scalar,Dim,1>;

      /// @brief Assign values indicated in a list token to an
      /// `Eigen` column vector.
      ///
      template<typename Scalar, typename PointType>
      void
      parse_list_as_pt_impl(const std::string &token, PointType &pt,
                            char sep);
    }

  }
}


# include "ConfigParser.hxx"
#endif
