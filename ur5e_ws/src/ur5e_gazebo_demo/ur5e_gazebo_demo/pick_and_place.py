#!/usr/bin/env python3

import time
import rclpy
from ikpy.chain import Chain
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ur5e_gazebo_demo.frames import world_to_base
from ur5e_gazebo_demo.urdf_paths import get_ik_urdf_path


class PickPlace(Node):

    def __init__(self):
        super().__init__('pick_place_demo')

        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        self.declare_parameter('gripper_close', 0.8)
        self.declare_parameter('home_shoulder_lift', -1.5707)

        urdf_path = get_ik_urdf_path()
        self.get_logger().info(f'Loading IK chain from {urdf_path}')
        self.chain = Chain.from_urdf_file(urdf_path)

        shoulder_lift = self.get_parameter('home_shoulder_lift').value
        self.home_seed = [0.0, 0.0, 0.0, shoulder_lift, 0.0, 0.0, 0.0, 0.0, 0.0]

        time.sleep(2)

    def move_arm(self, positions, sec=5):
        msg = JointTrajectory()
        msg.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
        ]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = sec
        msg.points.append(point)
        self.arm_pub.publish(msg)
        time.sleep(sec + 1)

    def ik_move_world(self, world_x, world_y, world_z, sec=5):
        base_x, base_y, base_z = world_to_base(world_x, world_y, world_z)
        tool_x = 0.32
        tool_z = 0.18
        target_x = base_x - tool_x
        target_y = base_y
        target_z = base_z + tool_z

        self.get_logger().info(
            f'IK target base_link: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})'
        )

        ik = self.chain.inverse_kinematics(
            target_position=[target_x, target_y, target_z],
            initial_position=self.home_seed,
        )
        joints = [ik[2], ik[3], ik[4], ik[5], 0.0, 0.0]
        self.move_arm(joints, sec)

    def move_gripper(self, position, sec=2):
        msg = JointTrajectory()
        msg.joint_names = ['robotiq_85_left_knuckle_joint']
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = sec
        msg.points.append(point)
        self.gripper_pub.publish(msg)
        time.sleep(sec + 1)

    def run(self):
        close_pos = self.get_parameter('gripper_close').value
        shoulder_lift = self.get_parameter('home_shoulder_lift').value

        self.move_gripper(close_pos)
        self.move_arm([0.0, shoulder_lift, 0.0, 0.0, 0.0, 0.0])
        self.ik_move_world(0.75, 0.0, 1.15, 5)
        self.ik_move_world(0.75, 0.0, 0.925, 5)
        self.get_logger().info('TEST DONE')


def main():
    rclpy.init()
    node = PickPlace()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
