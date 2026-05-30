"""Gripper control via FollowJointTrajectory action."""

import time

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class GripperClient:
    """Send gripper goals through the ros2_control action interface."""

    def __init__(self, node, joint_name, action_name='/gripper_controller/follow_joint_trajectory'):
        self._node = node
        self._joint_name = joint_name
        self._client = ActionClient(node, FollowJointTrajectory, action_name)

    def wait_for_server(self, timeout_sec=10.0):
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(f'Gripper action server not available: {self._client._action_name}')

    def _spin_until_done(self, future, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        return future.done()

    def move_to(self, position, duration_sec=2.0):
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.header.stamp = self._node.get_clock().now().to_msg()
        traj.joint_names = [self._joint_name]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        traj.points.append(point)
        goal.trajectory = traj

        send_future = self._client.send_goal_async(goal)
        self._node.get_logger().info(f'Gripper → {position:.2f}')
        if not self._spin_until_done(send_future, timeout_sec=15.0):
            self._node.get_logger().error('Timed out sending gripper goal')
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._node.get_logger().error('Gripper goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        if not self._spin_until_done(result_future, timeout_sec=duration_sec + 20.0):
            self._node.get_logger().error(
                'Timed out waiting for gripper result; check controllers are active '
                'and simulation is running (unpaused, /clock publishing)'
            )
            return False
        return True
