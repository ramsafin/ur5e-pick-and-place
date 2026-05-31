#!/usr/bin/env bash
# Generate flat URDF for ikpy / offline tools (not committed).
set -eo pipefail

if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "${WS_ROOT}/install/setup.bash"
fi

OUT="${1:-${WS_ROOT}/ur5e_full.urdf}"
XACRO="$(ros2 pkg prefix ur5e_description)/share/ur5e_description/urdf/ur5e.urdf.xacro"

xacro "${XACRO}" use_mock_hardware:=false -o "${OUT}"
echo "Wrote ${OUT}"
