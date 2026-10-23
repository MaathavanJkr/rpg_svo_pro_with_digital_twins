#!/usr/bin/env bash
# Regression check for the Eigen 3.3.7 + SVO compiler-flag issue (docs/pipeline_notes.md, "Eigen
# Matrix4d::inverse() under -fno-signed-zeros"). Compiles with the exact SVO Release flags
# (svo_cmake/cmake/Modules/SvoSetup.cmake) and with -O0, and reports:
#   1. Matrix4d::inverse() error (expected: WRONG under SVO flags, ~0 otherwise),
#   2. other Eigen double ops used by SVO/minkindr (expected: identical to -O0).
set -e
D="$(cd "$(dirname "$0")" && pwd)"; T="$(mktemp -d)"
SVO="-march=native -mmmx -msse -msse2 -msse3 -mssse3 -mno-avx -O3 -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -ffast-math -fno-finite-math-only"
B="-std=c++14 -I/usr/include/eigen3"
/usr/bin/g++ $B $SVO "$D/eigen_inverse_fastmath.cpp" -o "$T/inv_svo"; /usr/bin/g++ $B -O0 "$D/eigen_inverse_fastmath.cpp" -o "$T/inv_ref"
echo "Matrix4d::inverse(), SVO flags: $("$T/inv_svo")"
echo "Matrix4d::inverse(), -O0:       $("$T/inv_ref")"
/usr/bin/g++ $B $SVO "$D/eigen_ops_fastmath.cpp" -o "$T/ops_svo"; /usr/bin/g++ $B -O0 "$D/eigen_ops_fastmath.cpp" -o "$T/ops_ref"
if diff <("$T/ops_svo") <("$T/ops_ref") > /dev/null; then echo "Other Eigen ops: identical to -O0"; else echo "Other Eigen ops: DIFFER from -O0"; fi
rm -rf "$T"
