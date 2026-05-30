#!/usr/bin/env python3
"""Move arm and gripper to home once after simulation controllers are ready."""

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ur5e_gazebo_demo.gripper_client import GripperClient

ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]


class GoHome(Node):

    def __init__(self):
        super().__init__('go_home')
        self.declare_parameter('home_joints', [0.0, -1.5707, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('gripper_joint', 'robotiq_85_left_knuckle_joint')
        self.declare_parameter('gripper_open', 0.0)
        self.declare_parameter('duration_sec', 3.0)

        self._arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        self._gripper = GripperClient(
            self, self.get_parameter('gripper_joint').value
        )

    def _wait_for_joint_states(self, timeout_sec=60.0):
        seen = {'ready': False}

        def _cb(msg):
            if len(msg.name) >= 8:
                seen['ready'] = True

        sub = self.create_subscription(JointState, '/joint_states', _cb, 10)
        deadline = time.monotonic() + timeout_sec
        try:
            while rclpy.ok() and not seen['ready'] and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.2)
        finally:
            self.destroy_subscription(sub)

        if not seen['ready']:
            raise RuntimeError('Timed out waiting for /joint_states before homing')

    def _spin_until_done(self, future, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def _move_arm_home(self):
        duration = float(self.get_parameter('duration_sec').value)
        home = list(self.get_parameter('home_joints').value)

        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in home]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        traj.points.append(point)
        goal.trajectory = traj

        if not self._arm_client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('arm_controller action server not available')

        send_future = self._arm_client.send_goal_async(goal)
        if not self._spin_until_done(send_future, timeout_sec=15.0):
            raise RuntimeError('Timed out sending arm home goal')

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('Arm home goal rejected')

        result_future = goal_handle.get_result_async()
        if not self._spin_until_done(result_future, timeout_sec=duration + 15.0):
            raise RuntimeError('Timed out waiting for arm home result')

        self.get_logger().info('Arm at home position')

    def run(self):
        self.get_logger().info('Homing manipulator to startup pose...')
        self._wait_for_joint_states()
        self._gripper.wait_for_server()
        self._gripper.move_to(self.get_parameter('gripper_open').value)
        self._move_arm_home()
        self.get_logger().info('Startup homing complete')


def main():
    rclpy.init()
    node = GoHome()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
