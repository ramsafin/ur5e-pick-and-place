#!/usr/bin/env bash
# Static launch-file checks: verify launch descriptions generate without errors.
set -eo pipefail

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  source "${WS_ROOT}/install/setup.bash"
fi

LAUNCHES=(
  "ur5e_gazebo_demo pick_moveit.launch.py"
  "ur5e_gazebo_demo ur5e_sim.launch.py"
  "ur5e_gazebo_demo run_pick.launch.py"
  "ur5e_gazebo_demo vision_pick.launch.py"
  "ur5e_moveit_config demo.launch.py"
)

FAILED=0
for entry in "${LAUNCHES[@]}"; do
  pkg="${entry%% *}"
  launch="${entry#* }"
  echo "==> ros2 launch ${pkg} ${launch} --show-args"
  if ! ros2 launch "${pkg}" "${launch}" --show-args >/dev/null 2>&1; then
    echo "FAILED: ${pkg} ${launch}"
    ros2 launch "${pkg}" "${launch}" --show-args 2>&1 || true
    FAILED=1
  fi
done

if [ "${FAILED}" -ne 0 ]; then
  echo "Launch validation failed."
  exit 1
fi

echo "All launch files parsed successfully."
