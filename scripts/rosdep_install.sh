#!/usr/bin/env bash
# Install workspace ROS dependencies via rosdep (quiet wrapper).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${WS_ROOT}"
exec "${SCRIPT_DIR}/rosdep.sh" install --from-paths src --ignore-src -r -y "$@"
