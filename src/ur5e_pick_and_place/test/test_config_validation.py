"""Tests for static robot/MoveIt configuration validation."""

import pytest

from ur5e_pick_and_place.config_validation import (
    ARM_JOINTS,
    GRIPPER_CONTROLLER_JOINT,
    chain_joint_names,
    expand_xacro,
    parse_moveit_controller_joints,
    parse_ros2_controller_joints,
    resolve_config_paths,
    urdf_joint_names,
    validate_all,
    validate_mount_transform,
    validate_urdf_structure,
    ValidationResult,
)


@pytest.fixture(scope='module')
def paths():
    return resolve_config_paths()


@pytest.fixture(scope='module')
def expanded_urdfs(paths):
    gazebo_xml, _ = expand_xacro(
        paths.gazebo_xacro,
        ['use_mock_hardware:=false'],
    )
    moveit_xml, _ = expand_xacro(paths.moveit_xacro)
    return gazebo_xml, moveit_xml


def test_config_files_exist(paths):
    for path in (
        paths.gazebo_xacro,
        paths.moveit_xacro,
        paths.srdf,
        paths.moveit_controllers,
        paths.ros2_controllers,
        paths.pick_targets,
    ):
        assert path.is_file(), str(path)


def test_xacro_expansion(expanded_urdfs):
    gazebo_xml, moveit_xml = expanded_urdfs
    assert '<robot' in gazebo_xml
    assert '<robot' in moveit_xml


def test_moveit_controller_joints(paths):
    arm, gripper = parse_moveit_controller_joints(paths.moveit_controllers)
    assert set(arm) == ARM_JOINTS
    assert gripper == [GRIPPER_CONTROLLER_JOINT]


def test_ros2_controller_joints(paths):
    arm, gripper = parse_ros2_controller_joints(paths.ros2_controllers)
    assert set(arm) == ARM_JOINTS
    assert gripper == [GRIPPER_CONTROLLER_JOINT]


def test_manipulator_chain(expanded_urdfs):
    from ur5e_pick_and_place.config_validation import parse_urdf

    gazebo_root = parse_urdf(expanded_urdfs[0])
    chain = chain_joint_names(gazebo_root, 'base_link', 'robotiq_85_base_link')
    assert chain
    assert set(chain) <= urdf_joint_names(gazebo_root)


def test_mount_transform(expanded_urdfs):
    from ur5e_pick_and_place.config_validation import parse_urdf

    result = ValidationResult()
    for label, xml in (('Gazebo', expanded_urdfs[0]), ('MoveIt', expanded_urdfs[1])):
        validate_mount_transform(parse_urdf(xml), label, result)
    assert not any(issue.severity == 'error' for issue in result.issues)


def test_check_urdf(expanded_urdfs):
    result = ValidationResult()
    validate_urdf_structure(expanded_urdfs[0], 'Gazebo URDF', result)
    validate_urdf_structure(expanded_urdfs[1], 'MoveIt URDF', result)
    assert not any(issue.severity == 'error' for issue in result.issues)


def test_validate_all_runs():
    result = validate_all()
    assert result.issues is not None
    for issue in result.issues:
        assert issue.severity in ('error', 'warn')
        assert issue.message
