"""Gazebo Sim bringup with event-driven controller and MoveIt sequencing."""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_bringup = get_package_share_directory('ur5e_bringup')
    pkg_desc = get_package_share_directory('ur5e_description')
    robotiq_share = get_package_share_directory('robotiq_description')
    robotiq_parent = os.path.dirname(robotiq_share)

    launch_moveit = LaunchConfiguration('launch_moveit')
    launch_bridge = LaunchConfiguration('launch_bridge')
    headless = LaunchConfiguration('headless')
    home_at_startup = LaunchConfiguration('home_at_startup')

    world = os.path.join(pkg_bringup, 'worlds', 'camera_world.sdf')
    urdf = os.path.join(pkg_desc, 'urdf', 'ur5e.urdf.xacro')
    pick_params = os.path.join(pkg_bringup, 'config', 'pick_targets.yaml')
    controller_params = os.path.join(pkg_desc, 'config', 'ros2_controllers.yaml')

    moveit_config = (
        MoveItConfigsBuilder('ur5e', package_name='ur5e_moveit_config')
        .to_moveit_configs()
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf, ' use_mock_hardware:=false']),
        value_type=str,
    )

    gz_prefix = get_package_prefix('gz_ros2_control')
    gz_plugin = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.path.join(gz_prefix, 'lib'),
    )
    gz_res = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [os.environ.get('GZ_SIM_RESOURCE_PATH', ''), ':', robotiq_parent],
    )

    gazebo_gui = ExecuteProcess(
        condition=UnlessCondition(headless),
        cmd=['gz', 'sim', '-r', world, '--render-engine', 'ogre2'],
        output='screen',
    )
    gazebo_headless = ExecuteProcess(
        condition=IfCondition(headless),
        cmd=['gz', 'sim', '-r', '-s', world],
        output='screen',
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'ur5e'],
        output='screen',
    )

    bridge = Node(
        condition=IfCondition(launch_bridge),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/top_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            'arm_controller',
            'gripper_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
            '--switch-timeout', '120',
            '--param-file', controller_params,
        ],
        output='screen',
    )

    go_home = Node(
        condition=IfCondition(home_at_startup),
        package='ur5e_pick_and_place',
        executable='go_home',
        parameters=[pick_params, {'use_sim_time': True}],
        output='screen',
    )

    moveit_params = moveit_config.to_dict()
    moveit_params['robot_description'] = robot_description

    move_group = Node(
        condition=IfCondition(launch_moveit),
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            moveit_params,
            pick_params,
            {
                'use_sim_time': True,
                'planning_scene_monitor_options': {
                    'joint_state_topic': '/joint_states',
                },
            },
        ],
        output='screen',
    )

    # Minimal delay for Gazebo world load, then RSP + bridge.
    rsp_and_bridge = TimerAction(
        period=3.0,
        actions=[rsp, bridge],
    )

    spawn_after_rsp = TimerAction(
        period=4.0,
        actions=[spawn_robot],
    )

    spawn_exit_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[controller_spawner],
        ),
    )

    controllers_exit_home = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_spawner,
            on_exit=[go_home],
        ),
        condition=IfCondition(home_at_startup),
    )

    home_exit_move_group = RegisterEventHandler(
        OnProcessExit(
            target_action=go_home,
            on_exit=[move_group],
        ),
        condition=IfCondition(home_at_startup),
    )

    controllers_exit_move_group = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_spawner,
            on_exit=[move_group],
        ),
        condition=UnlessCondition(home_at_startup),
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument('launch_moveit', default_value='false'),
        DeclareLaunchArgument('launch_bridge', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('home_at_startup', default_value='true'),
        gz_plugin,
        gz_res,
        gazebo_gui,
        gazebo_headless,
        rsp_and_bridge,
        spawn_after_rsp,
        spawn_exit_controllers,
        controllers_exit_home,
        home_exit_move_group,
        controllers_exit_move_group,
    ])
