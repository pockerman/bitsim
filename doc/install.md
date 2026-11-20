\page bitsim_install Installing bitsim

# Installing bitsim

Ubuntu 24.04 is the reference platform. The CI builds bitsim there with the same scripts
this page uses, so they are known to work.

## Dependencies

| Dependency | Needed for | Installed by |
| ---------- | ---------- | ------------ |
| CMake 3.20 or newer, a C++20 compiler | everything | `ci/install_deps.sh` |
| Boost 1.74 or newer, Eigen3 | everything | `ci/install_deps.sh` |
| OpenCV | `ENABLE_OPENCV` | `ci/install_deps.sh` |
| Paho MQTT C and C++ | `ENABLE_MQTT` | `ci/install_deps.sh`, built from source |
| GoogleTest | `ENABLE_TESTS` | `ci/install_deps.sh` |
| Project Chrono with its Irrlicht module | `ENABLE_CHRONO`, `BITSIM_IRRLICHT` | `ci/build_chrono.sh` |

bitsim also uses <a href="https://github.com/nlohmann/json">nlohmann/json</a> and
<a href="https://github.com/elnormous/HTTPRequest">HTTPRequest</a>, which are included
in `src/bitsim/extern`.

Install everything except Chrono with

@code{.sh}
./ci/install_deps.sh
@endcode

## Project Chrono

bitsim needs a specific commit of Chrono's main branch; the 9.0.1 release lacks the
CMake targets and API that bitsim uses. To build that commit, first install the packages
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
| `ENABLE_EXAMPLES` | `ON` | Build the examples, see \ref bitsim_examples |
| `ENABLE_TESTS` | `OFF` | Build the tests |
| `BITSIM_IRRLICHT` | `ON` | Irrlicht visualization. Requires `ENABLE_CHRONO` |
| `BITSIM_LOG` | `ON` | Library logging |
| `BITSIM_WERROR` | `OFF` | Treat warnings as errors in Debug builds |
| `BITSIM_SANITIZE` | `OFF` | Build with AddressSanitizer and UndefinedBehaviorSanitizer. Run the tests with `LSAN_OPTIONS=suppressions=ci/lsan.supp` |

## Install

@code{.sh}
cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/opt/bitsim
cmake --build build --parallel
cmake --install build
@endcode

Without `-DCMAKE_INSTALL_PREFIX` bitsim installs into its source tree, `install/opt` for a
Release build and `install/dbg` for a Debug build. Under the prefix you get

| Path | What |
| ---- | ---- |
| `lib/libbitsimlib.so` | the library |
| `include/bitsim/` | the headers, with the generated `bitsim_config.h` and `bitsim_version.h` |
| `share/bitsim/` | the robot, sensor and world definitions: `robots/`, `sensors/`, `worlds/` |
| `lib/cmake/bitsim/` | the CMake package |

## Use bitsim from another project

@code{.cmake}
find_package(bitsim 2 REQUIRED)
target_link_libraries(my_target PRIVATE bitsim::bitsim)
@endcode

Configure the project with `-DCMAKE_PREFIX_PATH=$HOME/opt/bitsim`, and add the Chrono prefix to
it if bitsim was built with Chrono, for example `-DCMAKE_PREFIX_PATH="$HOME/opt/bitsim;$HOME/opt/chrono"`.
`bitsim::bitsim` brings the include directories, the compile definitions and the libraries
bitsim needs. The package also sets `bitsim_WITH_CHRONO`, `bitsim_WITH_IRRLICHT`,
`bitsim_WITH_MQTT` and `bitsim_WITH_OPENCV` to say what bitsim was built
with, and `bitsim_DATA_DIR` to the installed definitions.

`bitsim::consts::ROBOTS_DIR`, `SENSORS_DIR` and `WORLDS_DIR` point into the directory
returned by `bitsim::consts::data_dir()`. It is

1. the directory in the environment variable `BITSIM_DATA_DIR`, if it is set;
2. otherwise the source tree bitsim was built from, if it still exists, so that a build
   uses the definitions you edit;
3. otherwise `share/bitsim` under the install prefix.
