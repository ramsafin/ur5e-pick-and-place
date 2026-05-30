#!/usr/bin/env python3
"""OpenCV red-object detection prototype (arm-only moves)."""

import time
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ur5e_gazebo_demo.frames import pixel_to_base


class VisionPick(Node):

    def __init__(self):
        super().__init__('vision_pick')
        self.declare_parameter('vision_image_topic', '/top_camera/image')
        self.declare_parameter('vision_image_center_x', 320)
        self.declare_parameter('vision_image_center_y', 240)
        self.declare_parameter('vision_world_x_at_center', 0.0)
        self.declare_parameter('vision_world_y_at_center', 0.0)
        self.declare_parameter('vision_meters_per_pixel_x', 0.00283)
        self.declare_parameter('vision_meters_per_pixel_y', 0.00283)
        self.declare_parameter('vision_table_world_z', 0.925)

        self._bridge = CvBridge()
        self._image = None
        topic = self.get_parameter('vision_image_topic').value
        self.create_subscription(Image, topic, self._image_callback, 10)
        self._arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

    def _image_callback(self, msg):
        self._image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def move_arm(self, positions, sec=3):
        traj = JointTrajectory()
        traj.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
        ]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = sec
        traj.points.append(point)
        self._arm_pub.publish(traj)
        time.sleep(sec + 1)

    def detect_red_object(self):
        self.get_logger().info('Waiting for camera image...')
        while self._image is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._image is None:
            return None

        hsv = cv2.cvtColor(self._image.copy(), cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask += cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().error('No red object detected')
            return None

        contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(contour)
        cx = x + w // 2
        cy = y + h // 2
        self.get_logger().info(f'Object pixel: ({cx}, {cy})')
        return cx, cy

    def run(self):
        detection = self.detect_red_object()
        if detection is None:
            return

        cx, cy = detection
        box_x, box_y = pixel_to_base(
            cx, cy,
            self.get_parameter('vision_image_center_x').value,
            self.get_parameter('vision_image_center_y').value,
            self.get_parameter('vision_world_x_at_center').value,
            self.get_parameter('vision_world_y_at_center').value,
            self.get_parameter('vision_meters_per_pixel_x').value,
            self.get_parameter('vision_meters_per_pixel_y').value,
            self.get_parameter('vision_table_world_z').value,
        )
        self.get_logger().info(f'Base link target: ({box_x:.3f}, {box_y:.3f})')
        self.get_logger().info('Use vision_pick_moveit for full pick-and-place with MoveIt')


def main(args=None):
    rclpy.init(args=args)
    node = VisionPick()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
