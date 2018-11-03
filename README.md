
--------------------------------------------------------------------------------

## `pasta`: header-only C++17 library for `p`robability `a`nd `sta`tistics.

Zhijin Li [jonathan.zj.lee@gmail.com](jonathan.zj.lee@gmail.com)

--------------------------------------------------------------------------------
   <br><br>


--------------------------------------------------------------------------------

### Entries

--------------------------------------------------------------------------------

* Soure code on [GitHub](https://github.com/jonathanzjl)

* Documentation page

   <br><br>


--------------------------------------------------------------------------------

### Dependencies

--------------------------------------------------------------------------------

- `C++17` conformant compiler. Tested with `g++-8.2.0` on `macOS Majave`
  (`MacPorts`).
- [`eigen3`](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [`boost`](https://www.boost.org/) (`boost::lexical_cast`)

    <br><br>


--------------------------------------------------------------------------------

### Install

--------------------------------------------------------------------------------

Installation can be performed by copying `pasta` headers to a
directory recognizable as an include path. Additionally, the
**standard `autotools` utilities** are also provided, more
particularly for automatic testing & documentation generation. To
generate a `configure` script, run the following command from an
**out-of-source** directory. This will setup the build environment.

    /absolute/path/to/pasta/bootstrap.sh [install-prefix] <extra flags>

+ The `bootstrap.sh` script launches `autoreconf` to generate project
  `configure` file and run it with follow-up commands.
+ `install-prefix` refers to the prefix path to install pasta headers.
+ `<extra flags>` specifies non-standard `eigen3` and `boost`
  installation directories:
  + `--with-eigen`, path to `eigen3`
  + `--with-boost`, path to `boost`.
+ You can **prepend the above command** with `CXX=<compiler>` to
  specify a compiler to use.

- Use `make check` to run unit-tests.
- Use `make doc` to generate the API documentation.
- Use `make install` to copy header files under `prefix/include/pasta` and
  doc files under `prefix/share/doc/pasta`.
- Use `make uninstall` to remove installed headers and docs.
- Use `make clean` and `make distclean` for a clean re-build.

   <br><br>


--------------------------------------------------------------------------------

### Usage

--------------------------------------------------------------------------------

**Note**: the `pasta/Utilities/io.hh` header uses `std::filesystem`. In `gcc-8` a
linker flag `-stdc++fs` needs to be added for any code that includes
`pasta/Utilities/io.hh` to compile.
