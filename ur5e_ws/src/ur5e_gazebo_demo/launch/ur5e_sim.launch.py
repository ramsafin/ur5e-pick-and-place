from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('ur5e_gazebo_demo')
    gazebo_launch = os.path.join(pkg, 'launch', 'ur5e_gazebo.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('launch_bridge', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'launch_moveit': 'false',
                'launch_bridge': LaunchConfiguration('launch_bridge'),
            }.items(),
        ),
    ])
