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

assert_camera_publishing() {
  if timeout 15 ros2 topic echo /top_camera/image --once >/dev/null 2>&1; then
    echo "Camera topic /top_camera/image is publishing."
    return 0
  fi
  local deadline=$((SECONDS + 20))
  while [ "${SECONDS}" -lt "${deadline}" ]; do
    if timeout 8 ros2 topic hz /top_camera/image --window 3 2>&1 | grep -q 'average rate'; then
      echo "Camera topic /top_camera/image is publishing."
      return 0
    fi
    sleep 2
  done
  echo "ERROR: /top_camera/image not publishing."
  ros2 topic list 2>&1 | grep top_camera || true
  return 1
}

assert_vision_detection() {
  local vision_log
  vision_log="$(mktemp)"
  if ! timeout 20 ros2 run ur5e_pick_and_place vision_pick \
    --ros-args -p use_sim_time:=true \
    --params-file "${WS_ROOT}/src/ur5e_bringup/config/pick_targets.yaml" >"${vision_log}" 2>&1; then
    echo "ERROR: vision_pick exited with failure."
    tail -20 "${vision_log}" || true
    return 1
  fi
  if ! grep -q 'Object pixel:' "${vision_log}"; then
    echo "ERROR: vision_pick did not detect the red box."
    tail -20 "${vision_log}" || true
    return 1
  fi
  if ! grep -E 'Base link target: \(-?[0-9]+\.[0-9]+, 0\.7[0-9]+\)' "${vision_log}" >/dev/null; then
    echo "ERROR: vision calibration target out of range (expected base_link y ~ 0.75)."
    grep 'Base link target:' "${vision_log}" || true
    return 1
  fi
  grep 'Object pixel:' "${vision_log}" || true
  grep 'Base link target:' "${vision_log}" || true
  echo "Vision detection smoke check passed."
}

smoke_mock() {
  echo "Starting MoveIt demo + spawn_controllers (mock hardware)..."
  ros2 launch ur5e_bringup moveit_mock.launch.py &
  STACK_PID=$!
  sleep 12
  SPAWN_PID=$!
  sleep 5
  wait_for_controllers
  ros2 control list_controllers
  assert_move_action
  echo "Mock-hardware smoke test passed."
}

smoke_camera() {
  echo "Starting headless Gazebo sim (camera + vision checks)..."
  SIM_LOG="$(mktemp)"
  timeout "${TIMEOUT}" ros2 launch ur5e_bringup gz_sim.launch.py headless:=true 2>&1 | tee "${SIM_LOG}" &
  STACK_PID=$!
  sleep 20
  assert_camera_publishing
  assert_vision_detection
  echo "Camera smoke test passed."
}

smoke_gazebo() {
  echo "Starting Gazebo + MoveIt stack (headless server mode)..."
  SIM_LOG="$(mktemp)"
  timeout "${TIMEOUT}" ros2 launch ur5e_bringup gz_moveit.launch.py headless:=true 2>&1 | tee "${SIM_LOG}" &
  STACK_PID=$!
  sleep 45
  wait_for_controllers
  timeout 10 ros2 topic hz /clock --window 5 2>/dev/null || true
  assert_move_action
  assert_camera_publishing
  assert_vision_detection
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
  timeout "${TIMEOUT}" ros2 launch ur5e_bringup gz_moveit.launch.py headless:=true 2>&1 | tee "${SIM_LOG}" &
  STACK_PID=$!
  sleep 35
  wait_for_controllers
  assert_move_action
  assert_camera_publishing
  assert_vision_detection
  if grep -q 'camera_world.*is not a model' "${SIM_LOG}"; then
    echo "ERROR: world-level gz_ros2_control plugin still present."
    exit 1
  fi
  timeout 180 ros2 run ur5e_pick_and_place pick_and_place_moveit --ros-args -p use_sim_time:=true 2>&1 | tee "${PICK_LOG}"
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
  camera)
    smoke_camera
    ;;
  gazebo)
    smoke_gazebo
    ;;
  pick)
    smoke_pick
    ;;
  *)
    echo "Usage: $0 [mock|camera|gazebo|pick]"
    exit 2
    ;;
esac
