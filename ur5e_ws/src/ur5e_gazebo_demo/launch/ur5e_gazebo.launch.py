import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_gz = get_package_share_directory('ur5e_gazebo_demo')
    robotiq_share = get_package_share_directory('robotiq_description')
    robotiq_parent = os.path.dirname(robotiq_share)

    launch_moveit = LaunchConfiguration('launch_moveit')
    launch_bridge = LaunchConfiguration('launch_bridge')
    headless = LaunchConfiguration('headless')
    home_at_startup = LaunchConfiguration('home_at_startup')

    world = os.path.join(pkg_gz, 'worlds', 'camera_world.sdf')
    urdf = os.path.join(pkg_gz, 'urdf', 'ur5e.urdf.xacro')
    pick_params = os.path.join(pkg_gz, 'config', 'pick_targets.yaml')

    moveit_config = (
        MoveItConfigsBuilder('ur5e', package_name='ur5e_moveit_config')
        .to_moveit_configs()
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf]),
        value_type=str,
    )

    gz_plugin = SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/opt/ros/jazzy/lib')
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

    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', 'robot_description', '-name', 'ur5e'],
                output='screen',
            )
        ],
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

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    gripper_ctrl = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    clock_bridge = TimerAction(
        period=1.0,
        actions=[bridge],
    )

    sim_stack = TimerAction(
        period=5.0,
        actions=[rsp, jsb, arm_ctrl, gripper_ctrl],
    )

    moveit_params = moveit_config.to_dict()
    moveit_params['robot_description'] = robot_description

    move_group = TimerAction(
        period=12.0,
        condition=IfCondition(launch_moveit),
        actions=[
            Node(
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
        ],
    )

    go_home = TimerAction(
        period=10.0,
        condition=IfCondition(home_at_startup),
        actions=[
            Node(
                package='ur5e_gazebo_demo',
                executable='go_home',
                parameters=[pick_params, {'use_sim_time': True}],
                output='screen',
            )
        ],
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
        clock_bridge,
        spawn_robot,
        sim_stack,
        go_home,
        move_group,
    ])
