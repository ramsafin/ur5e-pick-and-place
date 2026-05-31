#!/usr/bin/env bash
# Run rosdep without setuptools pkg_resources DeprecationWarning on /usr/bin/rosdep.
# Upstream: https://github.com/ros-infrastructure/rosdep/issues/1011
set -eo pipefail

export PYTHONWARNINGS="${PYTHONWARNINGS:+$PYTHONWARNINGS,}ignore::DeprecationWarning"

if command -v rosdep >/dev/null 2>&1; then
  exec rosdep "$@"
fi

echo "rosdep not found; install: sudo apt install python3-rosdep" >&2
exit 1
