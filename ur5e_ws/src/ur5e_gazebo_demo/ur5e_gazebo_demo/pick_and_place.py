#!/usr/bin/env python3

import time
import rclpy
import numpy as np
from ikpy.chain import Chain

from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class PickPlace(Node):

    def __init__(self):

        super().__init__('pick_place_demo')

        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

        time.sleep(2)

        self.chain = Chain.from_urdf_file(
            "/home/sun_boy/ur5e_ws/ur5e_full.urdf"
        )

        # 记住home姿态
        self.home_seed = [
            0.0,          # fixed
            0.0,          # fixed
            0.0,          # shoulder_pan
            -1.633628,    # shoulder_lift
            0.0,          # elbow
            0.0,          # wrist1
            0.0,          # wrist2
            0.0,          # wrist3
            0.0           # fixed
        ]

    # ============================================
    # ARM
    # ============================================

    def move_arm(self, positions, sec=5):

        msg = JointTrajectory()

        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = sec

        msg.points.append(point)

        self.arm_pub.publish(msg)

        time.sleep(sec + 1)

    # ============================================
    # IK
    # ============================================

    def ik_move(self, x, y, z, sec=5):

        tool_x = 0.32
        tool_z = 0.18

        target_x = x-tool_x
        target_y = y
        target_z = z+tool_z

        self.get_logger().info(
            f'IK target tcp: {target_x:.3f} {target_y:.3f} {target_z:.3f}'
        )

        ik = self.chain.inverse_kinematics(
            target_position=[target_x, target_y, target_z],
            initial_position=self.home_seed
        )

        joints = [
            ik[2],
            ik[3],
            ik[4],
            ik[5],
            0.0,    # 固定 wrist_2
            0.0     # 固定 wrist_3
        ]

        self.get_logger().info(
            f'IK joints: {joints}'
        )

        self.move_arm(joints, sec)

    # ============================================
    # GRIPPER
    # ============================================

    def move_gripper(self, position, sec=2):

        msg = JointTrajectory()

        msg.joint_names = [
            'robotiq_85_left_knuckle_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = sec

        msg.points.append(point)

        self.gripper_pub.publish(msg)

        time.sleep(sec + 1)

    # ============================================
    # TEST
    # ============================================

    def run(self):

        self.get_logger().info('OPEN')
        self.move_gripper(0.8)

        self.get_logger().info('HOME')

        self.move_arm([
            0.0,
            -1.633628,
            0.0,
            0.0,
            0.0,
            0.0
        ])

        time.sleep(2)

        # 红色块
        # center:
        # x=0.65
        # y=0.25
        # z=0.90

        # 先上方
        self.get_logger().info(
            'ABOVE OBJECT'
        )

        self.ik_move(
            0.65,
            0.25,
            1.08,
            5
        )

        time.sleep(2)

        # 再下降
        self.get_logger().info(
            'DOWN TO OBJECT'
        )

        self.ik_move(
            0.65,
            0.25,
            1.00,
            5
        )

        self.get_logger().info(
            'TEST DONE'
        )


def main():

    rclpy.init()

    node = PickPlace()
    node.run()

    rclpy.shutdown()


if __name__ == '__main__':
    main()