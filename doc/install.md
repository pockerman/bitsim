\page bitrl_install Installing bitrl

# Installing bitrl

Ubuntu 24.04 is the reference platform. The CI builds bitrl there with the same scripts
this page uses, so they are known to work.

## Dependencies

| Dependency | Needed for | Installed by |
| ---------- | ---------- | ------------ |
| CMake 3.20 or newer, a C++20 compiler | everything | `ci/install_deps.sh` |
| Boost 1.74 or newer, Eigen3 | everything | `ci/install_deps.sh` |
| OpenCV | `ENABLE_OPENCV` | `ci/install_deps.sh` |
| Paho MQTT C and C++ | `ENABLE_MQTT` | `ci/install_deps.sh`, built from source |
| GoogleTest | `ENABLE_TESTS` | `ci/install_deps.sh` |
| Project Chrono with its Irrlicht module | `ENABLE_CHRONO`, `BITRL_IRRLICHT` | `ci/build_chrono.sh` |

bitrl also uses <a href="https://github.com/nlohmann/json">nlohmann/json</a> and
<a href="https://github.com/elnormous/HTTPRequest">HTTPRequest</a>, which are included
in `src/bitrl/extern`.

Install everything except Chrono with

@code{.sh}
./ci/install_deps.sh
@endcode

## Project Chrono

bitrl needs a specific commit of Chrono's main branch; the 9.0.1 release lacks the
CMake targets and API that bitrl uses. To build that commit, first install the packages
Chrono needs and then build it under a prefix of your choice:

@code{.sh}
./ci/install_deps.sh --with-chrono-deps
./ci/build_chrono.sh $HOME/opt/chrono
@endcode

`--with-chrono-deps` also patches `/usr/include/irrlicht/irrXML.h`, which does not
compile as C++20.

Skip this section and configure with `-DENABLE_CHRONO=OFF` if you do not need the
simulations.

## Build

@code{.sh}
cmake -B build -DCMAKE_BUILD_TYPE=Release -DENABLE_TESTS=ON -DCMAKE_PREFIX_PATH=$HOME/opt/chrono
cmake --build build --parallel
ctest --test-dir build --output-on-failure
@endcode

| Option | Default | Description |
| ------ | ------- | ----------- |
| `ENABLE_CHRONO` | `ON` | Project Chrono simulation. Add `-DCMAKE_PREFIX_PATH=<prefix>` for a non-system install |
| `ENABLE_MQTT` | `ON` | Paho MQTT support |
| `ENABLE_OPENCV` | `ON` | OpenCV support |
| `ENABLE_WEBOTS` | `OFF` | Webots support |
| `ENABLE_EXAMPLES` | `ON` | Build the examples, see \ref bitrl_examples |
| `ENABLE_TESTS` | `OFF` | Build the tests |
| `BITRL_IRRLICHT` | `ON` | Irrlicht visualization. Requires `ENABLE_CHRONO` |
| `BITRL_LOG` | `ON` | Library logging |
| `BITRL_WERROR` | `OFF` | Treat warnings as errors in Debug builds |
| `BITRL_SANITIZE` | `OFF` | Build with AddressSanitizer and UndefinedBehaviorSanitizer. Run the tests with `LSAN_OPTIONS=suppressions=ci/lsan.supp` |

## Install

@code{.sh}
cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/opt/bitrl
cmake --build build --parallel
cmake --install build
@endcode

Without `-DCMAKE_INSTALL_PREFIX` bitrl installs into its source tree, `install/opt` for a
Release build and `install/dbg` for a Debug build. Under the prefix you get

| Path | What |
| ---- | ---- |
| `lib/libbitrllib.so` | the library |
| `include/bitrl/` | the headers, with the generated `bitrl_config.h` and `bitrl_version.h` |
| `share/bitrl/` | the robot, sensor and world definitions: `robots/`, `sensors/`, `worlds/` |
| `lib/cmake/bitrl/` | the CMake package |

## Use bitrl from another project

@code{.cmake}
find_package(bitrl 2 REQUIRED)
target_link_libraries(my_target PRIVATE bitrl::bitrl)
@endcode

Configure the project with `-DCMAKE_PREFIX_PATH=$HOME/opt/bitrl`, and add the Chrono prefix to
it if bitrl was built with Chrono, for example `-DCMAKE_PREFIX_PATH="$HOME/opt/bitrl;$HOME/opt/chrono"`.
`bitrl::bitrl` brings the include directories, the compile definitions and the libraries
bitrl needs. The package also sets `bitrl_WITH_CHRONO`, `bitrl_WITH_IRRLICHT`,
`bitrl_WITH_MQTT`, `bitrl_WITH_OPENCV` and `bitrl_WITH_WEBOTS` to say what bitrl was built
with, and `bitrl_DATA_DIR` to the installed definitions.

`bitrl::consts::ROBOTS_DIR`, `SENSORS_DIR` and `WORLDS_DIR` point into the directory
returned by `bitrl::consts::data_dir()`. It is

1. the directory in the environment variable `BITRL_DATA_DIR`, if it is set;
2. otherwise the source tree bitrl was built from, if it still exists, so that a build
   uses the definitions you edit;
3. otherwise `share/bitrl` under the install prefix.
