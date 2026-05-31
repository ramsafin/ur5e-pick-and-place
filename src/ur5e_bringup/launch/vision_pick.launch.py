from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    pkg_bringup = get_package_share_directory('ur5e_bringup')
    pick_params = os.path.join(pkg_bringup, 'config', 'pick_targets.yaml')
    gz_moveit = os.path.join(pkg_bringup, 'launch', 'gz_moveit.launch.py')

    moveit_config = (
        MoveItConfigsBuilder('ur5e', package_name='ur5e_moveit_config')
        .to_moveit_configs()
    )

    vision_pick = Node(
        package='ur5e_pick_and_place',
        executable='vision_pick_moveit',
        parameters=[moveit_config.to_dict(), pick_params, {'use_sim_time': True}],
        output='screen',
    )

    # Start vision pick after startup homing (go_home exits when complete).
    # Allow event-driven gz_moveit stack (spawn -> controllers -> go_home -> move_group).
    vision_after_stack = TimerAction(
        period=35.0,
        actions=[vision_pick],
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_moveit),
        ),
        vision_after_stack,
    ])
