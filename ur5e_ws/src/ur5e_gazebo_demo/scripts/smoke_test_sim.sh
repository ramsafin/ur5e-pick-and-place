#!/usr/bin/env bash
# Local runtime smoke tests (not run in CI). Requires a sourced ROS 2 + workspace.
set -eo pipefail

MODE="${1:-mock}"
TIMEOUT="${SMOKE_TIMEOUT:-120}"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  source "${WS_ROOT}/install/setup.bash"
fi

pkill -f 'gz sim.*camera_world' 2>/dev/null || true
pkill -f 'moveit_ros_move_group/move_group' 2>/dev/null || true
sleep 2

cleanup() {
  if [ -n "${STACK_PID:-}" ] && kill -0 "${STACK_PID}" 2>/dev/null; then
    kill "${STACK_PID}" 2>/dev/null || true
    wait "${STACK_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

wait_for_controllers() {
  local deadline=$((SECONDS + 45))
  while [ "${SECONDS}" -lt "${deadline}" ]; do
    if ros2 control list_controllers 2>/dev/null | grep -q 'arm_controller.*active' \
      && ros2 control list_controllers 2>/dev/null | grep -q 'gripper_controller.*active'; then
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for controllers."
  ros2 control list_controllers 2>&1 || true
  return 1
}

assert_move_action() {
  if ! ros2 action list 2>/dev/null | grep -q '/move_action'; then
    echo "Missing /move_action action server."
    ros2 action list 2>&1 || true
    return 1
  fi
}

smoke_mock() {
  echo "Starting MoveIt demo + spawn_controllers (mock hardware)..."
  ros2 launch ur5e_moveit_config demo.launch.py &
  STACK_PID=$!
  sleep 8
  ros2 launch ur5e_moveit_config spawn_controllers.launch.py &
  SPAWN_PID=$!
  sleep 5
  wait_for_controllers
  ros2 control list_controllers
  assert_move_action
  kill "${SPAWN_PID}" 2>/dev/null || true
  echo "Mock-hardware smoke test passed."
}

smoke_gazebo() {
  echo "Starting Gazebo + MoveIt stack (headless server mode)..."
  SIM_LOG="$(mktemp)"
  timeout "${TIMEOUT}" ros2 launch ur5e_gazebo_demo pick_moveit.launch.py headless:=true 2>&1 | tee "${SIM_LOG}" &
  STACK_PID=$!
  sleep 20
  wait_for_controllers
  ros2 topic hz /clock --window 5 || true
  assert_move_action
  if grep -q 'camera_world.*is not a model' "${SIM_LOG}"; then
    echo "ERROR: world-level gz_ros2_control plugin still present."
    exit 1
  fi
  echo "Gazebo smoke test passed (controllers + move_action available)."
}

smoke_pick() {
  echo "Starting headless Gazebo + MoveIt + pick-and-place..."
  SIM_LOG="$(mktemp)"
  PICK_LOG="$(mktemp)"
  timeout "${TIMEOUT}" ros2 launch ur5e_gazebo_demo pick_moveit.launch.py headless:=true 2>&1 | tee "${SIM_LOG}" &
  STACK_PID=$!
  sleep 20
  wait_for_controllers
  assert_move_action
  if grep -q 'camera_world.*is not a model' "${SIM_LOG}"; then
    echo "ERROR: world-level gz_ros2_control plugin still present."
    exit 1
  fi
  timeout 180 ros2 run ur5e_gazebo_demo pick_and_place_moveit --ros-args -p use_sim_time:=true 2>&1 | tee "${PICK_LOG}"
  if grep -q '=== DONE ===' "${PICK_LOG}"; then
    echo "Pick-and-place smoke test passed."
  else
    echo "Pick-and-place smoke test failed."
    grep -E 'failed \(code=|ERROR' "${PICK_LOG}" || true
    exit 1
  fi
}

case "${MODE}" in
  mock)
    smoke_mock
    ;;
  gazebo)
    smoke_gazebo
    ;;
  pick)
    smoke_pick
    ;;
  *)
    echo "Usage: $0 [mock|gazebo|pick]"
    exit 2
    ;;
esac
