from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('ur5e_gazebo_demo')
    pick_params = os.path.join(pkg, 'config', 'pick_targets.yaml')

    moveit_config = (
        MoveItConfigsBuilder('ur5e', package_name='ur5e_moveit_config')
        .to_moveit_configs()
    )

    pick_node = Node(
        package='ur5e_gazebo_demo',
        executable='pick_and_place_moveit',
        parameters=[moveit_config.to_dict(), pick_params, {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        pick_node,
    ])
