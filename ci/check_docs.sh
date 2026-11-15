#!/usr/bin/env bash
# Check the documentation that lives in bitrl: every example has a page that
# the examples index links to, and Doxygen builds the example and doc pages
# without warnings. The API documentation in src/ is not checked yet.
# Usage: ci/check_docs.sh
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

python3 ci/check_example_docs.py "$root"

out_dir="$(mktemp -d)"
trap 'rm -rf "$out_dir"' EXIT

# The same inputs as the documentation site, bitrl_cuberl_docs, uses for bitrl
{
    cat Doxyfile
    echo "INPUT = src/bitrl examples doc"
    echo "RECURSIVE = YES"
    echo "EXCLUDE = src/bitrl/extern"
    echo "OUTPUT_DIRECTORY = $out_dir"
    echo "WARN_IF_UNDOCUMENTED = NO"
    echo "WARN_LOGFILE = $out_dir/warnings.log"
    echo "GENERATE_LATEX = NO"
    echo "QUIET = YES"
} | doxygen - > /dev/null

if grep -E "^$root/(examples|doc)/" "$out_dir/warnings.log"; then
    echo "error: fix the Doxygen warnings above"
    exit 1
fi
echo "Doxygen built the example and doc pages without warnings"
