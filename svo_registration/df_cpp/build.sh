#!/usr/bin/env bash
# FYP Phase 8: build the fyp_df Python module with the system toolchain and system Python 3.8.
# Linuxbrew is removed from PATH for the same reason as scripts/build.sh (its ld/gcc are not the
# system ones). pybind11 (header-only) comes from the analysis venv ~/venvs/fyp.
# Output: svo_registration/scripts/fyp_df.cpython-38-x86_64-linux-gnu.so (imported by the node).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
BUILD="$HERE/build"

export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v linuxbrew | paste -sd:)"
unset HOMEBREW_PREFIX HOMEBREW_CELLAR HOMEBREW_REPOSITORY
PYBIND11_DIR="$("$HOME/venvs/fyp/bin/python" -c 'import pybind11; print(pybind11.get_cmake_dir())')"

cmake -S "$HERE" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
  -DPython_EXECUTABLE=/usr/bin/python3 -Dpybind11_DIR="$PYBIND11_DIR"
cmake --build "$BUILD" -j4 -- VERBOSE=1 | tee "$BUILD/build.log" | grep -E "error|warning|Linking|Built" || true
SO="$(ls "$BUILD"/fyp_df.cpython-38-*.so)"
cp "$SO" "$HERE/../scripts/"
echo "installed $(basename "$SO") md5 $(md5sum "$SO" | cut -c1-12)"
if grep -E "fast-math|fno-signed-zeros" "$BUILD/build.log"; then
  echo "ERROR: fast-math flags found in the compile commands" >&2; exit 1
fi
