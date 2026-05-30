#!/usr/bin/env python3
"""Vision-guided pick and place using MoveIt2."""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from ur5e_gazebo_demo.frames import pixel_to_base
from ur5e_gazebo_demo.pick_and_place_moveit import PickAndPlaceMoveIt


class VisionPickMoveIt(PickAndPlaceMoveIt):

    def __init__(self):
        super().__init__()
        self.declare_parameter('use_vision', True)
        self.declare_parameter('vision_image_topic', '/top_camera/image')
        self.declare_parameter('vision_image_center_x', 320)
        self.declare_parameter('vision_image_center_y', 240)
        self.declare_parameter('vision_world_x_at_center', 0.75)
        self.declare_parameter('vision_world_y_at_center', 0.0)
        self.declare_parameter('vision_meters_per_pixel_x', 0.0015)
        self.declare_parameter('vision_meters_per_pixel_y', 0.0015)
        self.declare_parameter('vision_table_world_z', 0.925)

        self._bridge = CvBridge()
        self._image = None
        topic = self.get_parameter('vision_image_topic').value
        self.create_subscription(Image, topic, self._image_callback, 10)
        self.get_logger().info(f'VisionPickMoveIt listening on {topic}')

    def _image_callback(self, msg):
        self._image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def detect_red_object(self):
        self.get_logger().info('Waiting for camera image...')
        while self._image is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        if self._image is None:
            self.get_logger().error('No camera image received')
            return None

        hsv = cv2.cvtColor(self._image.copy(), cv2.COLOR_BGR2HSV)
        lower1 = np.array([0, 120, 70])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([170, 120, 70])
        upper2 = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower1, upper1) + cv2.inRange(hsv, lower2, upper2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().error('No red object detected')
            return None

        contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(contour)
        cx = x + w // 2
        cy = y + h // 2
        self.get_logger().info(f'Detected pixel centroid: ({cx}, {cy})')
        return cx, cy

    def resolve_pick_target(self):
        if not self.get_parameter('use_vision').value:
            return self.get_parameter('box_x').value, self.get_parameter('box_y').value

        detection = self.detect_red_object()
        if detection is None:
            return None

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
            self.get_parameter('robot_mount_z').value,
        )
        self.get_logger().info(f'Vision pick target base_link: ({box_x:.3f}, {box_y:.3f})')
        return box_x, box_y

    def run(self):
        target = self.resolve_pick_target()
        if target is None:
            return
        self.set_pick_target(*target)
        super().run()


def main():
    rclpy.init()
    node = VisionPickMoveIt()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
