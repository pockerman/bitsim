#!/usr/bin/env bash
# Install bitsim from a build directory into a temporary prefix, then build and
# run tests/install_consumer against it the way another project would.
# Usage: ci/check_install.sh <build-dir> [prefix of a dependency, e.g. Chrono]
set -euo pipefail

build_dir="$(realpath "${1:?usage: check_install.sh <build-dir> [dependency-prefix]}")"
dependency_prefix="${2:-}"
root="$(cd "$(dirname "$0")/.." && pwd)"

work_dir="$(mktemp -d)"
trap 'rm -rf "$work_dir"' EXIT

cmake --install "$build_dir" --prefix "$work_dir/prefix" > /dev/null
cmake -S "$root/tests/install_consumer" -B "$work_dir/build" \
    -DCMAKE_PREFIX_PATH="$work_dir/prefix${dependency_prefix:+;$dependency_prefix}"
cmake --build "$work_dir/build"

# Without BITSIM_DATA_DIR the library would use the source tree, which is still
# here, so point it at the installed robot, sensor and world files
BITSIM_DATA_DIR="$work_dir/prefix/share/bitsim" "$work_dir/build/install_consumer"
echo "The installed bitsim works"
