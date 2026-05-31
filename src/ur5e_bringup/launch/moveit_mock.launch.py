from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """RViz MoveIt demo with mock hardware (generate_demo_launch spawns controllers)."""
    demo = os.path.join(
        get_package_share_directory('ur5e_moveit_config'),
        'launch',
        'demo.launch.py',
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(demo)),
    ])
