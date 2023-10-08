import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    nav2_yaml_bot1 = os.path.join(get_package_share_directory(
        'multi_robot_arm'), 'config', 'bot1_amcl_config.yaml')
    nav2_yaml_bot2 = os.path.join(get_package_share_directory(
        'multi_robot_arm'), 'config', 'bot2_amcl_config.yaml')

    map_file = '/home/aliihsan/robot_ws/finishedmap.yaml'

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'topic_name':"map"},
                        {'frame_id':"map"},
                        {'yaml_filename': map_file}]
        ),

        Node(
            namespace = 'bot1',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml_bot1]
        ),

        Node(
            namespace = 'bot2',
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml_bot2]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'bond_timeout':0.0},
                        {'node_names': ['map_server', 'bot1/amcl','bot2/amcl']}]
        )
    ])
