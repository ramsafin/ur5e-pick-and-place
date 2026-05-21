from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder('ur5e', package_name='ur5e_moveit_config')
        .to_moveit_configs()
    )

    pick_node = Node(
        package='ur5e_gazebo_demo',
        executable='pick_and_place_moveit',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
        ],
        output='screen',
    )

    return LaunchDescription([pick_node])
