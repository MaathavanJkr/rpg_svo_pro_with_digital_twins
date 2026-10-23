#!/usr/bin/env bash
# FYP: build the catkin workspace with the system toolchain.
# ~/.zshrc puts Linuxbrew first on PATH; its ld (binutils 2.47) cannot link svo_node against the
# system libraries, and a failed link deletes the previous binary (docs/environment.md).
# Usage: scripts/build.sh [catkin build args...]   e.g. scripts/build.sh svo_ros --no-deps
#        (no args = catkin build of the whole workspace)
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
WS="$(cd "$REPO/../catkin_ws" && pwd)"

export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v linuxbrew | paste -sd:)"
unset HOMEBREW_PREFIX HOMEBREW_CELLAR HOMEBREW_REPOSITORY
if [ "$(command -v ld)" != "/usr/bin/ld" ]; then
  echo "Expected /usr/bin/ld, got $(command -v ld); aborting." >&2; exit 1
fi

source /opt/ros/noetic/setup.bash
cd "$WS"
echo "Building in $WS with $(ld --version | head -1)"
catkin build "$@"

LIB="$WS/devel/.private/svo_ros/lib/libsvo_ros.so"
BIN="$(readlink -f "$WS/devel/lib/svo_ros/svo_node")"
[ -e "$BIN" ] && echo "svo_node:   $(stat -c %y "$BIN")"
[ -e "$LIB" ] && echo "libsvo_ros: $(stat -c %y "$LIB") md5 $(md5sum "$LIB" | cut -c1-12)"
