"""Static validation of URDF, SRDF, controllers, and MoveIt config consistency."""

from __future__ import annotations

import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

import yaml

REQUIRED_LINKS = frozenset({'world', 'base_link', 'tool0', 'robotiq_85_base_link'})
EXPECTED_MOUNT_XYZ = (0.0, 0.0, 0.80)
EXPECTED_MOUNT_RPY = (0.0, 0.0, -1.5708)
MOUNT_TOLERANCE = 1e-3

ARM_JOINTS = frozenset({
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
})
GRIPPER_CONTROLLER_JOINT = 'robotiq_85_left_knuckle_joint'
MANIPULATOR_TIP_LINK = 'robotiq_85_base_link'
MANIPULATOR_BASE_LINK = 'base_link'


@dataclass
class ValidationIssue:
    severity: str
    message: str
    context: str = ''


@dataclass
class ValidationResult:
    issues: list[ValidationIssue] = field(default_factory=list)

    def error(self, message: str, context: str = '') -> None:
        self.issues.append(ValidationIssue('error', message, context))

    def warn(self, message: str, context: str = '') -> None:
        self.issues.append(ValidationIssue('warn', message, context))

    @property
    def ok(self) -> bool:
        return not any(issue.severity == 'error' for issue in self.issues)

    def merge(self, other: ValidationResult) -> None:
        self.issues.extend(other.issues)


@dataclass
class ConfigPaths:
    gazebo_xacro: Path
    moveit_xacro: Path
    srdf: Path
    moveit_controllers: Path
    ros2_controllers: Path
    gazebo_controllers: Path
    pick_targets: Path
    moveit_package_xml: Path


def resolve_config_paths() -> ConfigPaths:
    """Resolve config file paths from installed share dirs or source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory

        gz_share = Path(get_package_share_directory('ur5e_gazebo_demo'))
        mv_share = Path(get_package_share_directory('ur5e_moveit_config'))
        src_root = None
    except (ImportError, LookupError):
        src_root = Path(__file__).resolve().parents[2]
        gz_share = src_root / 'ur5e_gazebo_demo'
        mv_share = src_root / 'ur5e_moveit_config'

    return ConfigPaths(
        gazebo_xacro=gz_share / 'urdf' / 'ur5e.urdf.xacro',
        moveit_xacro=mv_share / 'config' / 'ur5e.urdf.xacro',
        srdf=mv_share / 'config' / 'ur5e.srdf',
        moveit_controllers=mv_share / 'config' / 'moveit_controllers.yaml',
        ros2_controllers=mv_share / 'config' / 'ros2_controllers.yaml',
        gazebo_controllers=gz_share / 'config' / 'controllers.yaml',
        pick_targets=gz_share / 'config' / 'pick_targets.yaml',
        moveit_package_xml=(
            mv_share / 'package.xml'
            if (mv_share / 'package.xml').is_file()
            else Path(__file__).resolve().parents[2]
            / 'ur5e_moveit_config'
            / 'package.xml'
        ),
    )


def expand_xacro(xacro_path: Path, extra_args: Iterable[str] | None = None) -> tuple[str, str]:
    """Expand xacro and return (urdf_xml, stderr)."""
    cmd = ['xacro', str(xacro_path)]
    if extra_args:
        cmd.extend(extra_args)
    proc = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        raise RuntimeError(
            f'xacro failed for {xacro_path} (exit {proc.returncode}): {proc.stderr.strip()}'
        )
    return proc.stdout, proc.stderr


def check_urdf(urdf_xml: str, label: str, result: ValidationResult) -> None:
    """Run check_urdf on expanded URDF."""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as handle:
        handle.write(urdf_xml)
        temp_path = handle.name
    try:
        proc = subprocess.run(
            ['check_urdf', temp_path],
            capture_output=True,
            text=True,
            check=False,
        )
        if proc.returncode != 0:
            result.error(
                f'check_urdf failed for {label}: {proc.stderr.strip() or proc.stdout.strip()}',
                label,
            )
    finally:
        os.unlink(temp_path)


def parse_urdf(urdf_xml: str) -> ET.Element:
    return ET.fromstring(urdf_xml)


def urdf_joint_names(root: ET.Element) -> set[str]:
    return {joint.get('name') for joint in root.findall('joint') if joint.get('name')}


def urdf_link_names(root: ET.Element) -> set[str]:
    return {link.get('name') for link in root.findall('link') if link.get('name')}


def ros2_control_blocks(root: ET.Element) -> list[ET.Element]:
    return root.findall('ros2_control')


def ros2_control_command_joints(root: ET.Element) -> set[str]:
    joints: set[str] = set()
    for block in ros2_control_blocks(root):
        for joint in block.findall('joint'):
            name = joint.get('name')
            if not name:
                continue
            for cmd in joint.findall('command_interface'):
                if cmd.get('name') == 'position':
                    joints.add(name)
    return joints


def _joint_link_name(element: ET.Element | None) -> str | None:
    if element is None:
        return None
    return element.get('link') or element.get('name')


def build_joint_graph(root: ET.Element) -> dict[str, tuple[str, str]]:
    """Map child link -> (joint name, parent link)."""
    graph: dict[str, tuple[str, str]] = {}
    for joint in root.findall('joint'):
        name = joint.get('name')
        parent = joint.find('parent')
        child = joint.find('child')
        parent_link = _joint_link_name(parent)
        child_link = _joint_link_name(child)
        if name and parent_link and child_link:
            graph[child_link] = (name, parent_link)
    return graph


def chain_joint_names(root: ET.Element, base_link: str, tip_link: str) -> list[str]:
    """Return joint names along the kinematic chain from base to tip."""
    graph = build_joint_graph(root)
    joints: list[str] = []
    link = tip_link
    while link and link != base_link:
        if link not in graph:
            return []
        joint_name, parent_link = graph[link]
        joints.append(joint_name)
        link = parent_link
    if link != base_link:
        return []
    joints.reverse()
    return joints


def parse_floats(text: str | None) -> tuple[float, ...]:
    if not text:
        return ()
    return tuple(float(part) for part in text.split())


def find_world_mount_origin(root: ET.Element) -> tuple[tuple[float, ...], tuple[float, ...]] | None:
    """Find origin on the joint connecting world to the robot base."""
    for joint in root.findall('joint'):
        parent = joint.find('parent')
        child = joint.find('child')
        if _joint_link_name(parent) == 'world' and _joint_link_name(child) == 'base_link':
            origin = joint.find('origin')
            if origin is not None:
                return parse_floats(origin.get('xyz')), parse_floats(origin.get('rpy'))
    return None


def origins_close(
    actual: tuple[float, ...],
    expected: tuple[float, ...],
    tolerance: float = MOUNT_TOLERANCE,
) -> bool:
    if len(actual) != len(expected):
        return False
    return all(abs(a - e) <= tolerance for a, e in zip(actual, expected))


def parse_srdf_joints(srdf_path: Path) -> dict[str, set[str]]:
    root = ET.parse(srdf_path).getroot()
    groups: dict[str, set[str]] = {}
    for group in root.findall('group'):
        name = group.get('name')
        if not name:
            continue
        joints: set[str] = set()
        for joint in group.findall('joint'):
            joint_name = joint.get('name')
            if joint_name:
                joints.add(joint_name)
        groups[name] = joints

    states: dict[str, dict[str, float]] = {}
    for state in root.findall('group_state'):
        state_name = state.get('name')
        if not state_name:
            continue
        values: dict[str, float] = {}
        for joint in state.findall('joint'):
            joint_name = joint.get('name')
            value = joint.get('value')
            if joint_name and value is not None:
                values[joint_name] = float(value)
        states[state_name] = values
    return {'groups': groups, 'states': states}


def parse_moveit_controller_joints(yaml_path: Path) -> tuple[list[str], list[str]]:
    with yaml_path.open(encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    manager = data.get('moveit_simple_controller_manager', {})
    arm = manager.get('arm_controller', {}).get('joints', [])
    gripper = manager.get('gripper_controller', {}).get('joints', [])
    return list(arm), list(gripper)


def parse_ros2_controller_joints(yaml_path: Path) -> tuple[list[str], list[str]]:
    with yaml_path.open(encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    arm = data.get('arm_controller', {}).get('ros__parameters', {}).get('joints', [])
    gripper = data.get('gripper_controller', {}).get('ros__parameters', {}).get('joints', [])
    return list(arm), list(gripper)


def normalize_controller_yaml(yaml_path: Path) -> dict:
    with yaml_path.open(encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def parse_pick_targets(yaml_path: Path) -> dict:
    with yaml_path.open(encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    return data.get('/**', {}).get('ros__parameters', data)


def check_xacro_stderr(stderr: str, label: str, result: ValidationResult) -> None:
    if not stderr.strip():
        return
    lowered = stderr.lower()
    if 'unknown property' in lowered or 'unused' in lowered or 'warning' in lowered:
        result.warn(f'xacro stderr for {label}: {stderr.strip()}', label)


def validate_urdf_structure(
    urdf_xml: str,
    label: str,
    result: ValidationResult,
) -> ET.Element:
    check_urdf(urdf_xml, label, result)
    root = parse_urdf(urdf_xml)
    links = urdf_link_names(root)
    missing_links = REQUIRED_LINKS - links
    if missing_links:
        result.error(
            f'{label} missing required links: {sorted(missing_links)}',
            label,
        )
    return root


def validate_mount_transform(root: ET.Element, label: str, result: ValidationResult) -> None:
    mount = find_world_mount_origin(root)
    if mount is None:
        result.error(f'{label} has no world->base_link joint origin', label)
        return
    xyz, rpy = mount
    if not origins_close(xyz, EXPECTED_MOUNT_XYZ):
        result.error(
            f'{label} mount xyz {xyz} != expected {EXPECTED_MOUNT_XYZ}',
            label,
        )
    if not origins_close(rpy, EXPECTED_MOUNT_RPY):
        result.error(
            f'{label} mount rpy {rpy} != expected {EXPECTED_MOUNT_RPY}',
            label,
        )


def validate_joint_consistency(
    gazebo_root: ET.Element,
    moveit_root: ET.Element,
    paths: ConfigPaths,
    result: ValidationResult,
) -> None:
    srdf_data = parse_srdf_joints(paths.srdf)
    srdf_groups = srdf_data['groups']
    srdf_states = srdf_data['states']

    gazebo_joints = urdf_joint_names(gazebo_root)
    moveit_joints = urdf_joint_names(moveit_root)

    chain_joints = chain_joint_names(
        gazebo_root,
        MANIPULATOR_BASE_LINK,
        MANIPULATOR_TIP_LINK,
    )
    if not chain_joints:
        result.error(
            'Could not resolve ur_manipulator chain in Gazebo URDF',
            'srdf',
        )
    else:
        for joint_name in chain_joints:
            if joint_name not in gazebo_joints:
                result.error(
                    f'Chain joint {joint_name} missing from Gazebo URDF',
                    'srdf',
                )
            if joint_name not in moveit_joints:
                result.error(
                    f'Chain joint {joint_name} missing from MoveIt URDF',
                    'srdf',
                )

    gripper_group = srdf_groups.get('gripper', set())
    for joint_name in gripper_group:
        if joint_name not in gazebo_joints:
            result.error(f'SRDF gripper joint {joint_name} missing from Gazebo URDF', 'srdf')
        if joint_name not in moveit_joints:
            result.error(f'SRDF gripper joint {joint_name} missing from MoveIt URDF', 'srdf')

    moveit_arm, moveit_gripper = parse_moveit_controller_joints(paths.moveit_controllers)
    demo_arm, demo_gripper = parse_ros2_controller_joints(paths.ros2_controllers)
    sim_arm, sim_gripper = parse_ros2_controller_joints(paths.gazebo_controllers)

    if set(moveit_arm) != ARM_JOINTS:
        result.error(
            f'moveit_controllers arm joints {moveit_arm} != expected {sorted(ARM_JOINTS)}',
            'moveit_controllers.yaml',
        )
    if moveit_gripper != [GRIPPER_CONTROLLER_JOINT]:
        result.error(
            f'moveit_controllers gripper joints {moveit_gripper} != [{GRIPPER_CONTROLLER_JOINT}]',
            'moveit_controllers.yaml',
        )

    for name, arm, gripper in (
        ('ros2_controllers.yaml', demo_arm, demo_gripper),
        ('controllers.yaml', sim_arm, sim_gripper),
    ):
        if set(arm) != ARM_JOINTS:
            result.error(f'{name} arm joints {arm} != expected {sorted(ARM_JOINTS)}', name)
        if gripper != [GRIPPER_CONTROLLER_JOINT]:
            result.error(
                f'{name} gripper joints {gripper} != [{GRIPPER_CONTROLLER_JOINT}]',
                name,
            )

    if normalize_controller_yaml(paths.ros2_controllers) != normalize_controller_yaml(
        paths.gazebo_controllers
    ):
        result.error(
            'controllers.yaml and ros2_controllers.yaml differ; keep sim and demo paths in sync',
            'controllers',
        )

    gazebo_rc_joints = ros2_control_command_joints(gazebo_root)
    moveit_rc_joints = ros2_control_command_joints(moveit_root)
    for joint_name in moveit_arm + moveit_gripper:
        if joint_name not in gazebo_rc_joints:
            result.error(
                f'Controller joint {joint_name} missing from Gazebo ros2_control command interfaces',
                'ros2_control',
            )
        if joint_name not in moveit_rc_joints:
            result.error(
                f'Controller joint {joint_name} missing from MoveIt ros2_control command interfaces',
                'ros2_control',
            )

    close_state = srdf_states.get('close', {})
    open_state = srdf_states.get('open', {})
    pick_targets = parse_pick_targets(paths.pick_targets)
    gripper_close = float(pick_targets.get('gripper_close', -1))
    gripper_open = float(pick_targets.get('gripper_open', -1))

    if open_state.get(GRIPPER_CONTROLLER_JOINT) != gripper_open:
        result.error(
            f'SRDF gripper open ({open_state.get(GRIPPER_CONTROLLER_JOINT)}) '
            f'!= pick_targets gripper_open ({gripper_open})',
            'pick_targets.yaml',
        )
    if close_state.get(GRIPPER_CONTROLLER_JOINT) != gripper_close:
        result.error(
            f'SRDF gripper close ({close_state.get(GRIPPER_CONTROLLER_JOINT)}) '
            f'!= pick_targets gripper_close ({gripper_close})',
            'pick_targets.yaml',
        )


def validate_dual_urdf_drift(
    gazebo_root: ET.Element,
    moveit_root: ET.Element,
    result: ValidationResult,
) -> None:
    gazebo_joints = urdf_joint_names(gazebo_root)
    moveit_joints = urdf_joint_names(moveit_root)
    if gazebo_joints != moveit_joints:
        only_gazebo = sorted(gazebo_joints - moveit_joints)
        only_moveit = sorted(moveit_joints - gazebo_joints)
        if only_gazebo or only_moveit:
            result.error(
                'Gazebo and MoveIt URDF joint sets differ: '
                f'only in Gazebo={only_gazebo}, only in MoveIt={only_moveit}',
                'dual-urdf',
            )

    gazebo_links = urdf_link_names(gazebo_root)
    moveit_links = urdf_link_names(moveit_root)
    srdf_links = {MANIPULATOR_BASE_LINK, MANIPULATOR_TIP_LINK, 'tool0', 'world'}
    for link_name in srdf_links:
        if link_name not in gazebo_links:
            result.error(f'SRDF-relevant link {link_name} missing from Gazebo URDF', 'dual-urdf')
        if link_name not in moveit_links:
            result.error(f'SRDF-relevant link {link_name} missing from MoveIt URDF', 'dual-urdf')


def validate_antipatterns(
    gazebo_root: ET.Element,
    moveit_root: ET.Element,
    paths: ConfigPaths,
    gazebo_xacro_stderr: str,
    result: ValidationResult,
) -> None:
    moveit_blocks = ros2_control_blocks(moveit_root)
    if len(moveit_blocks) > 1:
        result.error(
            f'MoveIt demo URDF has {len(moveit_blocks)} ros2_control blocks (expected 1); '
            'duplicate mock hardware from Robotiq macro + ur5e.ros2_control.xacro',
            'anti-pattern',
        )

    gazebo_xml = ET.tostring(gazebo_root, encoding='unicode')
    if 'PositionJointInterface' in gazebo_xml and 'gz_ros2_control' in gazebo_xml:
        result.warn(
            'Gazebo URDF contains legacy PositionJointInterface transmission alongside gz_ros2_control',
            'anti-pattern',
        )

    if paths.moveit_package_xml.is_file():
        pkg_xml = paths.moveit_package_xml.read_text(encoding='utf-8')
        for dep in ('ur_description', 'robotiq_description'):
            if f'<exec_depend>{dep}</exec_depend>' not in pkg_xml:
                result.warn(
                    f'ur5e_moveit_config/package.xml missing exec_depend on {dep}',
                    'package.xml',
                )

    check_xacro_stderr(gazebo_xacro_stderr, 'Gazebo URDF (launch args)', result)


def validate_moveit_config_load(result: ValidationResult) -> None:
    try:
        from moveit_configs_utils import MoveItConfigsBuilder
    except ImportError:
        result.warn(
            'moveit_configs_utils not available; skipping MoveItConfigsBuilder load check',
            'moveit',
        )
        return

    try:
        config = MoveItConfigsBuilder('ur5e', package_name='ur5e_moveit_config').to_moveit_configs()
        params = config.to_dict()
    except Exception as exc:
        result.error(f'MoveItConfigsBuilder failed: {exc}', 'moveit')
        return

    for key in ('robot_description', 'robot_description_semantic'):
        value = params.get(key)
        if not value or not str(value).strip():
            result.error(f'MoveIt config missing non-empty {key}', 'moveit')

    if 'moveit_controller_manager' not in params and 'moveit_simple_controller_manager' not in str(params):
        controller_keys = [key for key in params if 'controller' in key.lower()]
        if not controller_keys:
            result.warn('MoveIt config may be missing controller manager parameters', 'moveit')


def validate_all(paths: ConfigPaths | None = None) -> ValidationResult:
    """Run all static configuration checks."""
    result = ValidationResult()
    paths = paths or resolve_config_paths()

    for path_attr in (
        'gazebo_xacro',
        'moveit_xacro',
        'srdf',
        'moveit_controllers',
        'ros2_controllers',
        'gazebo_controllers',
        'pick_targets',
    ):
        path = getattr(paths, path_attr)
        if not path.is_file():
            result.error(f'Missing config file: {path}', path_attr)
    if not result.ok:
        return result

    gazebo_args = ['ur_type:=ur5e', 'name:=ur5e', 'prefix:=""']
    try:
        gazebo_xml, gazebo_stderr = expand_xacro(paths.gazebo_xacro, gazebo_args)
        moveit_xml, moveit_stderr = expand_xacro(paths.moveit_xacro)
    except RuntimeError as exc:
        result.error(str(exc), 'xacro')
        return result

    check_xacro_stderr(moveit_stderr, 'MoveIt URDF', result)

    gazebo_root = validate_urdf_structure(gazebo_xml, 'Gazebo URDF', result)
    moveit_root = validate_urdf_structure(moveit_xml, 'MoveIt URDF', result)

    if result.ok or gazebo_root is not None:
        validate_mount_transform(gazebo_root, 'Gazebo URDF', result)
        validate_mount_transform(moveit_root, 'MoveIt URDF', result)
        validate_joint_consistency(gazebo_root, moveit_root, paths, result)
        validate_dual_urdf_drift(gazebo_root, moveit_root, result)
        validate_antipatterns(gazebo_root, moveit_root, paths, gazebo_stderr, result)

    validate_moveit_config_load(result)
    return result


def format_report(result: ValidationResult) -> str:
    if not result.issues:
        return 'All configuration checks passed.'
    lines = []
    for issue in result.issues:
        prefix = 'ERROR' if issue.severity == 'error' else 'WARN'
        context = f' [{issue.context}]' if issue.context else ''
        lines.append(f'{prefix}{context}: {issue.message}')
    summary = 'FAILED' if not result.ok else 'PASSED WITH WARNINGS'
    lines.append(f'\nResult: {summary} ({len(result.issues)} issue(s))')
    return '\n'.join(lines)


def main() -> int:
    """Console entry point."""
    import sys

    result = validate_all()
    print(format_report(result))
    return 0 if result.ok else 1


if __name__ == '__main__':
    import sys

    sys.exit(main())
