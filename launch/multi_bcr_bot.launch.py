#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    AppendEnvironmentVariable,
    OpaqueFunction,
    TimerAction 
)
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bcr_bot_path = get_package_share_directory('bcr_bot')
    gz_sim_share = get_package_share_directory('ros_gz_sim')
    world_file = LaunchConfiguration('world_file', default=os.path.join(bcr_bot_path, 'worlds', 'small_warehouse.sdf'))
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    robots_config = LaunchConfiguration('robots_config', default=os.path.join(bcr_bot_path, 'config', 'robots.yaml'))

    # ---- 1. Gazebo (only once) ----
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PythonExpression(["'", world_file, " -r'"])}.items()
    )

    # ---- 2. Environment variables for Gazebo resources ----
    set_resource_paths =[
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=os.path.join(bcr_bot_path, 'worlds')),
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=os.path.join(bcr_bot_path, 'models')),
    ]

    def launch_setup(context, *args, **kwargs):
        # Resolve configurations
        config_file = robots_config.perform(context)
        bcr_path = bcr_bot_path
        
        # Load robots
        with open(config_file, 'r') as f:
            robots_data = yaml.safe_load(f)
        
        robots = robots_data.get('robots', [])
        
        robot_groups = []
        for robot in robots:
            robot_name = robot['name']
            
            # Robot state publisher (namespaced)
            rsp_node = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': Command([
                        'xacro ', os.path.join(bcr_path, 'urdf', 'bcr_bot.xacro'),
                        ' robot_namespace:=', robot_name,
                        ' wheel_odom_topic:=/', robot_name, '/odom',
                        ' camera_enabled:=', LaunchConfiguration('camera_enabled', default='true'),
                        ' stereo_camera_enabled:=', LaunchConfiguration('stereo_camera_enabled', default='false'),
                        ' two_d_lidar_enabled:=', LaunchConfiguration('two_d_lidar_enabled', default='true'),
                        ' sim_gz:=true'
                    ]),
                    'use_sim_time': use_sim_time,
                    'frame_prefix': f'{robot_name}/'
                }],
                remappings=[('/joint_states', f'/{robot_name}/joint_states')]
            )

            # Spawn robot in Gazebo
            spawn_node = Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', robot_name,
                    '-x', str(robot.get('x', 0.0)),
                    '-y', str(robot.get('y', 0.0)),
                    '-z', '0.28',
                    '-Y', str(robot.get('yaw', 0.0)),
                ],
                output='screen'
            )

            # Gazebo <-> ROS bridge
            bridge_node = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    f'/{robot_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    f'/{robot_name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    f'/{robot_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    f'/{robot_name}/kinect_camera@sensor_msgs/msg/Image[gz.msgs.Image',
                    f'/{robot_name}/kinect_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                    f'/{robot_name}/kinect_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                    f'/{robot_name}/kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    f'/{robot_name}/stereo_camera/left/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    f'/{robot_name}/stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    f'/{robot_name}/stereo_camera/right/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    f'/{robot_name}/stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    f'/{robot_name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    f'/world/default/model/{robot_name}/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
                ],
                remappings=[
                    (f'/world/default/model/{robot_name}/joint_state', f'/{robot_name}/joint_states'),
                    (f'/{robot_name}/odom', f'/{robot_name}/wheel_odom'),

                ],
                parameters=[{'use_sim_time': use_sim_time}]
            )

            # Static TF for kinect camera
            static_tf = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                    '--x', '0.0', '--y', '0.0', '--z', '0.0',
                    '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                    '--frame-id', f'{robot_name}/kinect_camera',
                    '--child-frame-id', f'{robot_name}/base_footprint/kinect_camera'
                ],
                parameters=[{'use_sim_time': use_sim_time}]
            )

            # Static TF from world to odom
            '''world_to_odom_tf = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                    '--x', str(robot.get('x', 0.0)),
                    '--y', str(robot.get('y', 0.0)),
                    '--z', '0.0',
                    '--yaw', str(robot.get('yaw', 0.0)),
                    '--pitch', '0.0', '--roll', '0.0',
                    '--frame-id', 'world',
                    '--child-frame-id', f'{robot_name}/odom'
                ],
                parameters=[{'use_sim_time': use_sim_time}]
            )'''

            robot_group = GroupAction([
                PushRosNamespace(robot_name),
                rsp_node,
                spawn_node,
                bridge_node,
                static_tf,
                #world_to_odom_tf,
            ])
            robot_groups.append(robot_group)

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(bcr_path, 'rviz', 'multi.rviz')],
            parameters=[{'use_sim_time': use_sim_time}]
        )
        localization_launch = TimerAction(
            period=7.0,   # Wait 7 seconds before launching Nav2/AMCL/Map
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bcr_bot_path, 'launch', 'localization.launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'robots_config': robots_config
                    }.items()
                )
            ]
        )

        global_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                #'/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )

        '''foxglove_bridge = Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,
                'send_buffer_limit': 10000000,
                'use_sim_time': use_sim_time
            }]
        )'''

        return robot_groups + [global_bridge, rviz_node, localization_launch]

    return LaunchDescription([
        DeclareLaunchArgument('world_file', default_value=world_file),
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),
        DeclareLaunchArgument('camera_enabled', default_value='true'),
        DeclareLaunchArgument('stereo_camera_enabled', default_value='false'),
        DeclareLaunchArgument('two_d_lidar_enabled', default_value='true'),
        DeclareLaunchArgument('robots_config', default_value=robots_config),
        *set_resource_paths,
        start_gazebo,
        OpaqueFunction(function=launch_setup)
    ])
