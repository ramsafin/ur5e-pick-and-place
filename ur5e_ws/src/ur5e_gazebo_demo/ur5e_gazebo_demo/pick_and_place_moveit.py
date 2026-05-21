#!/usr/bin/env python3
"""
Pick and Place via MoveIt2 /move_action interface.

Coordinate notes:
  base_link is at world (0, 0, 0.80), rotated yaw=-90deg.
  Conversion:  base_x = -world_y
               base_y =  world_x
               base_z =  world_z - 0.80

  Red box (world): center (0.75, 0, 0.925), size 0.06x0.06x0.20
  Red box (base):  center (0, 0.75, 0.125)

  Place target (world): (0.45, 0.30, 0.925) -- another spot on the table
  Place target (base):  (-0.30, 0.45, 0.125)
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PositionConstraint,
    PlanningScene,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header


JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]
HOME_JOINTS = [0.0, -1.5707, 0.0, 0.0, 0.0, 0.0]

# 物体在 base_link 中的坐标
BOX_X, BOX_Y      = 0.0,  0.75   # 方块 XY（世界0.75,0 → base 0,0.75）
BOX_GRASP_Z       = 0.125         # 抓取高度（方块中心，世界0.925-0.80）
BOX_PRE_Z         = 0.35          # 预抓取高度

# 放置位置（世界 0.45,0.30,0.925 → base -0.30,0.45,0.125）
PLACE_X, PLACE_Y  = -0.30, 0.45
PLACE_Z           = 0.125
PLACE_PRE_Z       = 0.35

TABLE_Z           = 0.025         # 桌面在 base_link 中的 z（仅参考）


class PickAndPlaceMoveIt(Node):

    def __init__(self):
        super().__init__('pick_and_place_moveit')

        self._client = ActionClient(self, MoveGroup, '/move_action')

        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

        self.scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        self.get_logger().info('Waiting for move_group action server...')
        self._client.wait_for_server()
        self.get_logger().info('Connected to move_group!')
        time.sleep(1.0)
        self._publish_scene()

    # ─────────────────────────────────────────
    # 碰撞场景（暂不添加桌面，避免与机械臂底座重叠导致规划失败）
    # ─────────────────────────────────────────

    def _publish_scene(self):
        self.get_logger().info('Planning scene: no extra objects')

    # ─────────────────────────────────────────
    # 发送 MoveGroup goal（规划 + 执行）
    # ─────────────────────────────────────────

    def _send_goal(self, constraints, label=''):
        goal = MoveGroup.Goal()
        goal.request.group_name = 'ur_manipulator'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False

        self.get_logger().info(f'Planning → {label}')
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()

        if not handle.accepted:
            self.get_logger().error(f'{label}: rejected')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        code = result_future.result().result.error_code.val

        if code == 1:
            self.get_logger().info(f'{label}: OK')
            return True
        self.get_logger().error(f'{label}: failed (code={code})')
        return False

    # ─────────────────────────────────────────
    # 关节空间运动
    # ─────────────────────────────────────────

    def move_joints(self, positions, label='joint'):
        c = Constraints()
        for name, pos in zip(JOINT_NAMES, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return self._send_goal(c, label)

    # ─────────────────────────────────────────
    # 笛卡尔位置运动（仅位置约束，不限姿态）
    # ─────────────────────────────────────────

    def move_pose(self, x, y, z, label='pose'):
        c = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = 'tool0'
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.02]           # 2cm 容差球
        target = Pose()
        target.position.x = float(x)
        target.position.y = float(y)
        target.position.z = float(z)
        target.orientation.w = 1.0
        pc.constraint_region.primitives.append(sp)
        pc.constraint_region.primitive_poses.append(target)
        pc.weight = 1.0
        c.position_constraints.append(pc)

        return self._send_goal(c, label)

    # ─────────────────────────────────────────
    # 夹爪
    # ─────────────────────────────────────────

    def open_gripper(self):
        self.get_logger().info('GRIPPER open')
        self._gripper(0.0)

    def close_gripper(self):
        self.get_logger().info('GRIPPER close')
        self._gripper(0.70)

    def _gripper(self, pos, sec=2):
        msg = JointTrajectory()
        msg.joint_names = ['robotiq_85_left_knuckle_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [pos]
        pt.time_from_start.sec = sec
        msg.points.append(pt)
        self.gripper_pub.publish(msg)
        time.sleep(sec + 0.5)

    # ─────────────────────────────────────────
    # 主流程
    # ─────────────────────────────────────────

    def run(self):
        self.get_logger().info('=== START PICK AND PLACE ===')

        # 1. 初始化
        self.open_gripper()
        if not self.move_joints(HOME_JOINTS, 'home'):
            return

        # 2. 移到物体上方
        if not self.move_pose(BOX_X, BOX_Y, BOX_PRE_Z, 'pre-grasp'):
            return

        # 3. 下降到抓取高度
        if not self.move_pose(BOX_X, BOX_Y, BOX_GRASP_Z, 'grasp'):
            return

        # 4. 夹取
        self.close_gripper()

        # 5. 抬起
        if not self.move_pose(BOX_X, BOX_Y, BOX_PRE_Z, 'lift'):
            return

        # 6. 移到放置点上方
        if not self.move_pose(PLACE_X, PLACE_Y, PLACE_PRE_Z, 'pre-place'):
            return

        # 7. 下降放置
        if not self.move_pose(PLACE_X, PLACE_Y, PLACE_Z, 'place'):
            return

        # 8. 松开
        self.open_gripper()

        # 9. 退出并回 home
        self.move_pose(PLACE_X, PLACE_Y, PLACE_PRE_Z, 'retreat')
        self.move_joints(HOME_JOINTS, 'home')

        self.get_logger().info('=== DONE ===')


def main():
    rclpy.init()
    node = PickAndPlaceMoveIt()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
