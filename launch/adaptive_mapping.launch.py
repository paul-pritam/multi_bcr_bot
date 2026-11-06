import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bcr = get_package_share_directory('bcr_bot')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    scan_topic = LaunchConfiguration('scan_topic', default='/bcr_bot_1/scan')
    global_frame = LaunchConfiguration('global_frame', default='map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_scan_topic = DeclareLaunchArgument(
        'scan_topic',
        default_value='/bcr_bot_1/scan',
        description='Laser scan topic'
    )

    declare_global_frame = DeclareLaunchArgument(
        'global_frame',
        default_value='map',
        description='Global frame id'
    )

    adaptive_mapper_node = Node(
        package='adaptive_costmap_2d',
        executable='adaptive_mapping_node',
        name='adaptive_mapping_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_topic': scan_topic,
            'global_frame': global_frame,
            'resolution': 0.05,
            'publish_rate': 2.0
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_scan_topic)
    ld.add_action(declare_global_frame)
    ld.add_action(adaptive_mapper_node)

    return ld
