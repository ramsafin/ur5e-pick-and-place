#!/usr/bin/env python3
"""Pick and place via MoveIt2 /move_action with configurable targets."""

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    PlanningScene,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from ur5e_gazebo_demo.gripper_client import GripperClient


JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

BOX_ID = 'red_box'
BOX_DIMENSIONS = [0.06, 0.06, 0.20]
GRIPPER_TOUCH_LINKS = [
    'robotiq_85_base_link',
    'robotiq_85_left_finger_link',
    'robotiq_85_right_finger_link',
    'robotiq_85_left_finger_tip_link',
    'robotiq_85_right_finger_tip_link',
    'robotiq_85_left_inner_knuckle_link',
    'robotiq_85_right_inner_knuckle_link',
    'robotiq_85_left_knuckle_link',
    'robotiq_85_right_knuckle_link',
    'tool0',
    'wrist_1_link',
    'wrist_2_link',
    'wrist_3_link',
    'forearm_link',
]


class PickAndPlaceMoveIt(Node):

    def __init__(self):
        super().__init__('pick_and_place_moveit')

        self.declare_parameter('tcp_link', 'robotiq_85_base_link')
        self.declare_parameter('gripper_joint', 'robotiq_85_left_knuckle_joint')
        self.declare_parameter('gripper_open', 0.0)
        self.declare_parameter('gripper_close', 0.8)
        self.declare_parameter('home_joints', [0.0, -1.5707, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('box_x', 0.0)
        self.declare_parameter('box_y', 0.75)
        self.declare_parameter('box_grasp_z', 0.125)
        self.declare_parameter('box_pre_z', 0.35)
        self.declare_parameter('place_x', -0.30)
        self.declare_parameter('place_y', 0.45)
        self.declare_parameter('place_z', 0.125)
        self.declare_parameter('place_pre_z', 0.35)
        self.declare_parameter('robot_mount_z', 0.80)
        self.declare_parameter('publish_collision_objects', True)

        self._client = ActionClient(self, MoveGroup, '/move_action')
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        gripper_joint = self.get_parameter('gripper_joint').value
        self._gripper = GripperClient(self, gripper_joint)

        self._wait_for_sim_clock()
        self._wait_for_active_controllers()
        self.get_logger().info('Waiting for move_group action server...')
        self._client.wait_for_server()
        self._gripper.wait_for_server()
        self.get_logger().info('Connected to move_group and gripper controller')

    def _collision_objects_enabled(self):
        return self.get_parameter('publish_collision_objects').value

    def _wait_for_sim_clock(self):
        if not self.get_clock().ros_time_is_active:
            return

        self.get_logger().info('Waiting for /clock (sim time)...')
        seen = {'value': False}

        def _on_clock(_msg):
            seen['value'] = True

        sub = self.create_subscription(Clock, '/clock', _on_clock, 10)
        deadline = time.monotonic() + 30.0
        try:
            while rclpy.ok() and not seen['value'] and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self.destroy_subscription(sub)

        if not seen['value']:
            self.get_logger().warn('Timed out waiting for /clock; TF timestamps may be inconsistent')

    def _wait_for_active_controllers(self, timeout_sec=90.0):
        """Wait until ros2_control is driving the robot (joint_states publishing)."""
        self.get_logger().info('Waiting for /joint_states from ros2_control...')
        seen = {'ready': False}

        def _on_joint_states(msg):
            if len(msg.name) >= 8:
                seen['ready'] = True

        sub = self.create_subscription(JointState, '/joint_states', _on_joint_states, 10)
        deadline = time.monotonic() + timeout_sec
        try:
            while rclpy.ok() and not seen['ready'] and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.2)
        finally:
            self.destroy_subscription(sub)

        if not seen['ready']:
            raise RuntimeError(
                'Timed out waiting for /joint_states. Start pick_moveit.launch.py first, '
                'wait until controllers are active (ros2 control list_controllers), '
                'then run run_pick.launch.py.'
            )
        self.get_logger().info('/joint_states ready (controllers active)')

    def _scene_stamp(self):
        stamp = self.get_clock().now().to_msg()
        return stamp

    def _make_box_object(self):
        box = CollisionObject()
        box.id = BOX_ID
        box.header.frame_id = self.get_parameter('tcp_link').value
        box.header.stamp = self._scene_stamp()
        box.primitives.append(
            SolidPrimitive(type=SolidPrimitive.BOX, dimensions=BOX_DIMENSIONS)
        )
        pose = Pose()
        pose.position.z = -0.14
        pose.orientation.w = 1.0
        box.primitive_poses.append(pose)
        box.operation = CollisionObject.ADD
        return box

    def _attach_box(self):
        if not self._collision_objects_enabled():
            return

        attached = AttachedCollisionObject()
        attached.link_name = self.get_parameter('tcp_link').value
        attached.object = self._make_box_object()
        attached.touch_links = list(GRIPPER_TOUCH_LINKS)

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        self.scene_pub.publish(scene)
        rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info('Planning scene: attached red_box to gripper')

    def _remove_box(self):
        if not self._collision_objects_enabled():
            return

        remove_attached = AttachedCollisionObject()
        remove_attached.link_name = self.get_parameter('tcp_link').value
        remove_attached.object.id = BOX_ID
        remove_attached.object.operation = CollisionObject.REMOVE
        remove_attached.object.header.stamp = self._scene_stamp()

        remove_world = CollisionObject()
        remove_world.id = BOX_ID
        remove_world.operation = CollisionObject.REMOVE
        remove_world.header.stamp = self._scene_stamp()

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(remove_attached)
        scene.world.collision_objects.append(remove_world)
        self.scene_pub.publish(scene)
        self.get_logger().info('Planning scene: removed red_box')

    def set_pick_target(self, box_x, box_y):
        self.set_parameters([
            rclpy.parameter.Parameter('box_x', rclpy.Parameter.Type.DOUBLE, float(box_x)),
            rclpy.parameter.Parameter('box_y', rclpy.Parameter.Type.DOUBLE, float(box_y)),
        ])

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

    def move_pose(self, x, y, z, label='pose'):
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = 'base_link'
        pc.link_name = self.get_parameter('tcp_link').value
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.02]
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

    def open_gripper(self):
        self.get_logger().info('GRIPPER open')
        if not self._gripper.move_to(self.get_parameter('gripper_open').value):
            raise RuntimeError('Failed to open gripper')

    def close_gripper(self):
        self.get_logger().info('GRIPPER close')
        if not self._gripper.move_to(self.get_parameter('gripper_close').value):
            raise RuntimeError('Failed to close gripper')

    def run(self):
        box_x = self.get_parameter('box_x').value
        box_y = self.get_parameter('box_y').value
        box_grasp_z = self.get_parameter('box_grasp_z').value
        box_pre_z = self.get_parameter('box_pre_z').value
        place_x = self.get_parameter('place_x').value
        place_y = self.get_parameter('place_y').value
        place_z = self.get_parameter('place_z').value
        place_pre_z = self.get_parameter('place_pre_z').value
        home = list(self.get_parameter('home_joints').value)

        self.get_logger().info('=== START PICK AND PLACE ===')
        self.get_logger().info(f'Pick target base_link: ({box_x:.3f}, {box_y:.3f}, {box_grasp_z:.3f})')

        self.open_gripper()
        if not self.move_joints(home, 'home'):
            return

        if not self.move_pose(box_x, box_y, box_pre_z, 'pre-grasp'):
            return
        if not self.move_pose(box_x, box_y, box_grasp_z, 'grasp'):
            return

        self.close_gripper()

        if not self.move_pose(box_x, box_y, box_pre_z, 'lift'):
            return

        self._attach_box()

        if not self.move_pose(place_x, place_y, place_pre_z, 'pre-place'):
            return
        if not self.move_pose(place_x, place_y, place_z, 'place'):
            return

        self.open_gripper()
        self._remove_box()
        self.move_pose(place_x, place_y, place_pre_z, 'retreat')
        self.move_joints(home, 'home')
        self.get_logger().info('=== DONE ===')


def main():
    rclpy.init()
    node = PickAndPlaceMoveIt()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
