from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup = os.path.join(
        get_package_share_directory('ur5e_bringup'),
        'launch',
        'gz_bringup.launch.py',
    )
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('home_at_startup', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={
                'launch_moveit': 'true',
                'headless': LaunchConfiguration('headless'),
                'home_at_startup': LaunchConfiguration('home_at_startup'),
            }.items(),
        ),
    ])
