#!/usr/bin/env python3
"""
Mega-Warehouse (120m x 80m) Autonomous Mapping & SLAM Demonstration.
Launches Gazebo Harmonic with mega_warehouse.sdf, spawns bcr_bot_1,
runs robot_localization EKF with correct namespaced frames, autostarts
SLAM Toolbox with AdaptiveSparseMapGenerator, runs LOD wireframes,
and launches RViz2 with correct RobotModel TF prefix.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node, PushRosNamespace
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg_bcr = get_package_share_directory('bcr_bot')
    gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_bcr, 'worlds', 'mega_warehouse.sdf')
    rviz_config = os.path.join(pkg_bcr, 'rviz', 'mega_warehouse.rviz')
    ekf_config = os.path.join(pkg_bcr, 'config', 'ekf.yaml')
    slam_params = os.path.join(pkg_bcr, 'config', 'mapper_params_online_async.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Environment variables for Gazebo model & world resolution
    set_resource_paths = [
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(pkg_bcr, 'worlds')
        ),
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(pkg_bcr, 'models')
        ),
    ]

    # 1. Start Gazebo Simulation
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': PythonExpression(["'", world_path, " -r'"])
        }.items()
    )

    # 2. Global Clock Bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 3. BCR Bot 1 Configuration & Spawn
    robot_name = 'bcr_bot_1'
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ', os.path.join(pkg_bcr, 'urdf', 'bcr_bot.xacro'),
                ' robot_namespace:=', robot_name,
                ' wheel_odom_topic:=/', robot_name, '/odom',
                ' two_d_lidar_enabled:=true',
                ' camera_enabled:=false',
                ' stereo_camera_enabled:=false',
                ' sim_gz:=true'
            ]),
            'use_sim_time': use_sim_time,
            'frame_prefix': f'{robot_name}/'
        }],
        remappings=[('/joint_states', f'/{robot_name}/joint_states')]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', f'/{robot_name}/robot_description',
            '-name', robot_name,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.28',
            '-Y', '0.0',
        ],
        output='screen'
    )

    robot_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            f'/{robot_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            f'/{robot_name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{robot_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/{robot_name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            f'/world/mega_warehouse/model/{robot_name}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            (f'/world/mega_warehouse/model/{robot_name}/joint_state', f'/{robot_name}/joint_states'),
            (f'/{robot_name}/odom', f'/{robot_name}/wheel_odom'),
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # EKF Node: Crucial frame namespacing to connect bcr_bot_1/odom -> bcr_bot_1/base_footprint
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {
                'use_sim_time': use_sim_time,
                'odom_frame': f'{robot_name}/odom',
                'base_link_frame': f'{robot_name}/base_footprint',
                'world_frame': f'{robot_name}/odom',
                'map_frame': 'map',
            }
        ]
    )

    robot_group = GroupAction([
        PushRosNamespace(robot_name),
        rsp_node,
        spawn_robot,
        robot_bridge,
        ekf_node
    ])

    # 4. SLAM Toolbox Node with Modular MapGenerator and Autostart
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[
            slam_params,
            {
                'use_sim_time': use_sim_time,
                'odom_frame': f'{robot_name}/odom',
                'map_frame': 'map',
                'base_frame': f'{robot_name}/base_footprint',
                'scan_topic': f'/{robot_name}/scan',
                'map_generator_plugin': 'map_generator_plugins::AdaptiveSparseMapGenerator',
                'map_update_interval': 2.0,
                'resolution': 0.05
            }
        ]
    )

    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] Activating SLAM Toolbox with AdaptiveSparseMapGenerator.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )

    # 5. Adaptive Block-Sparse Mapping Node (Publishes LOD wireframes)
    adaptive_mapper = Node(
        package='adaptive_costmap_2d',
        executable='adaptive_mapping_node',
        name='adaptive_mapping_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_topic': f'/{robot_name}/scan',
            'global_frame': 'map',
            'resolution': 0.05,
            'publish_rate': 2.0,
            'enable_multires_lod': True,
            'max_raytrace_clear_range': 8.0,
            'wireframe_thickness': 0.02
        }]
    )

    # 6. RViz2 Visualizer
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        *set_resource_paths,
        start_gazebo,
        clock_bridge,
        robot_group,
        slam_toolbox_node,
        configure_slam,
        activate_slam,
        adaptive_mapper,
        rviz_node
    ])
