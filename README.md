
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
- [`boost`](https://www.boost.org/)
-

    <br><br>


--------------------------------------------------------------------------------

### Install

--------------------------------------------------------------------------------

#### `automake`

The **standard `autotools` utilities** are also provided, more particularly for
automatic testing & documentation generation. To generate a `configure` script,
run the following command **from out-of-source** directory. This will setup the
build environment.

    /absolute/path/to/pasta/bootstrap.sh [install-prefix] <extra flags>

+ `install-prefix` refers to the prefix path to install pasta headers.
+ `<extra flags>` specifies non-standard `eigen3` and `boost`
  installation directories:
  + `--with-eigen`, path to `eigen3`
  + `--with-boost`, path to `boost`.

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
