from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('ur5e_bringup')
    pick_params = os.path.join(pkg, 'config', 'pick_targets.yaml')
    return LaunchDescription([
        Node(
            package='ur5e_pick_and_place',
            executable='pick_and_place_moveit',
            parameters=[pick_params, {'use_sim_time': True}],
            output='screen',
        ),
    ])
