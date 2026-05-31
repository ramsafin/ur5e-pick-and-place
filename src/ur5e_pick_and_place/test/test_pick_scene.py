"""Tests for pick-and-place planning scene collision object handling."""

from unittest.mock import MagicMock, patch

import rclpy
from shape_msgs.msg import SolidPrimitive

from ur5e_pick_and_place.pick_and_place_moveit import (
    BOX_DIMENSIONS,
    BOX_ID,
    GRIPPER_TOUCH_LINKS,
    PickAndPlaceMoveIt,
)


def _make_node():
    rclpy.init()
    try:
        with patch.object(PickAndPlaceMoveIt, '__init__', lambda self: None):
            node = PickAndPlaceMoveIt()
        node.get_logger = MagicMock()
        node.get_parameter = MagicMock(
            side_effect=lambda name: MagicMock(
                value={
                    'tcp_link': 'robotiq_85_base_link',
                    'publish_collision_objects': True,
                }[name]
            )
        )
        node.scene_pub = MagicMock()
        return node
    except Exception:
        rclpy.shutdown()
        raise


def test_init_does_not_publish_scene():
    assert hasattr(PickAndPlaceMoveIt, '_attach_box')
    assert hasattr(PickAndPlaceMoveIt, '_remove_box')
    assert not hasattr(PickAndPlaceMoveIt, '_publish_scene')


def test_make_box_object_uses_tcp_frame_and_dimensions():
    node = _make_node()
    try:
        box = node._make_box_object()
        assert box.id == BOX_ID
        assert box.header.frame_id == 'robotiq_85_base_link'
        assert box.primitives[0].type == SolidPrimitive.BOX
        assert list(box.primitives[0].dimensions) == BOX_DIMENSIONS
    finally:
        rclpy.shutdown()


def test_attach_box_publishes_attached_collision_object():
    node = _make_node()
    try:
        node._attach_box()
        assert node.scene_pub.publish.called
        scene = node.scene_pub.publish.call_args[0][0]
        assert len(scene.robot_state.attached_collision_objects) == 1
        attached = scene.robot_state.attached_collision_objects[0]
        assert attached.link_name == 'robotiq_85_base_link'
        assert attached.object.id == BOX_ID
        assert attached.touch_links == GRIPPER_TOUCH_LINKS
    finally:
        rclpy.shutdown()


def test_remove_box_publishes_remove_operations():
    node = _make_node()
    try:
        node._remove_box()
        scene = node.scene_pub.publish.call_args[0][0]
        assert scene.robot_state.attached_collision_objects[0].object.id == BOX_ID
        assert scene.world.collision_objects[0].id == BOX_ID
    finally:
        rclpy.shutdown()
