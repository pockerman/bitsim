#!/usr/bin/env bash
# Check the documentation that lives in bitrl: every example has a page that
# the examples index links to, and Doxygen builds the API documentation and
# the example and doc pages without warnings.
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
    echo "USE_MATHJAX = YES"
    echo "WARN_LOGFILE = $out_dir/warnings.log"
    echo "GENERATE_LATEX = NO"
    echo "QUIET = YES"
} | doxygen - > /dev/null

# Doxygen cannot match some out-of-class definitions of templates to their
# declarations, for example when a parameter type is
# typename Class<T>::type. These warnings are about Doxygen, not the comments
false_positive="warning: no (uniquely )?matching (class|file) member found for"

if grep -E "^$root/(src|examples|doc)/" "$out_dir/warnings.log" | grep -vE "$false_positive"; then
    echo "error: fix the Doxygen warnings above"
    exit 1
fi
echo "Doxygen built the API documentation and the example and doc pages without warnings"
