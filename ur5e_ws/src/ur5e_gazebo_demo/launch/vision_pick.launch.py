from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('ur5e_gazebo_demo')
    pick_params = os.path.join(pkg, 'config', 'pick_targets.yaml')
    sim_launch = os.path.join(pkg, 'launch', 'pick_moveit.launch.py')

    moveit_config = (
        MoveItConfigsBuilder('ur5e', package_name='ur5e_moveit_config')
        .to_moveit_configs()
    )

    vision_pick = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ur5e_gazebo_demo',
                executable='vision_pick_moveit',
                parameters=[moveit_config.to_dict(), pick_params, {'use_sim_time': True}],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch)),
        vision_pick,
    ])
