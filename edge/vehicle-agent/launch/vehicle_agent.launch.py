#!/usr/bin/env python3
"""
AtlasMesh Vehicle Agent Launch File
Launches the complete vehicle agent stack with all components
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('atlasmesh_vehicle_agent')
    
    # Launch arguments
    vehicle_id_arg = DeclareLaunchArgument(
        'vehicle_id',
        default_value='vehicle_001',
        description='Unique vehicle identifier'
    )
    
    vehicle_type_arg = DeclareLaunchArgument(
        'vehicle_type',
        default_value='ugv_themis',
        description='Vehicle type for profile selection'
    )
    
    profile_path_arg = DeclareLaunchArgument(
        'profile_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('atlasmesh_vehicle_agent'),
            'config', 'vehicle_profiles', 'ugv_themis.yaml'
        ]),
        description='Path to vehicle profile YAML file'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='50.0',
        description='Control loop frequency in Hz'
    )
    
    telemetry_frequency_arg = DeclareLaunchArgument(
        'telemetry_frequency',
        default_value='10.0',
        description='Telemetry publishing frequency in Hz'
    )
    
    cloud_endpoint_arg = DeclareLaunchArgument(
        'cloud_endpoint',
        default_value='https://api.atlasmesh.com',
        description='Cloud API endpoint URL'
    )
    
    mqtt_broker_arg = DeclareLaunchArgument(
        'mqtt_broker',
        default_value='mqtt.atlasmesh.com',
        description='MQTT broker hostname'
    )
    
    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='false',
        description='Use simulation mode (Gazebo/CARLA)'
    )
    
    enable_safety_monitor_arg = DeclareLaunchArgument(
        'enable_safety_monitor',
        default_value='true',
        description='Enable safety monitoring systems'
    )
    
    enable_ota_updates_arg = DeclareLaunchArgument(
        'enable_ota_updates',
        default_value='true',
        description='Enable OTA update management'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='ROS2 log level (DEBUG, INFO, WARN, ERROR, FATAL)'
    )
    
    # Set global parameters
    set_log_level = SetParameter(name='log_level', value=LaunchConfiguration('log_level'))
    
    # Main vehicle agent node
    vehicle_agent_node = Node(
        package='atlasmesh_vehicle_agent',
        executable='vehicle_agent_node',
        name='vehicle_agent',
        namespace=LaunchConfiguration('vehicle_id'),
        parameters=[
            {
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                'vehicle_type': LaunchConfiguration('vehicle_type'),
                'profile_path': LaunchConfiguration('profile_path'),
                'control_frequency': LaunchConfiguration('control_frequency'),
                'telemetry_frequency': LaunchConfiguration('telemetry_frequency'),
                'cloud_endpoint': LaunchConfiguration('cloud_endpoint'),
                'mqtt_broker': LaunchConfiguration('mqtt_broker'),
                'mqtt_port': 8883,
                'use_tls': True,
                'use_simulation': LaunchConfiguration('use_simulation'),
            }
        ],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=5.0
    )
    
    # Safety monitor node (critical - always enabled)
    safety_monitor_node = Node(
        package='atlasmesh_vehicle_agent',
        executable='safety_monitor_node',
        name='safety_monitor',
        namespace=LaunchConfiguration('vehicle_id'),
        parameters=[
            {
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                'profile_path': LaunchConfiguration('profile_path'),
                'emergency_stop_timeout': 5.0,
                'watchdog_timeout': 2.0,
                'max_control_latency': 0.1,
            }
        ],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration('enable_safety_monitor'))
    )
    
    # Health monitor node
    health_monitor_node = Node(
        package='atlasmesh_vehicle_agent',
        executable='health_monitor_node',
        name='health_monitor',
        namespace=LaunchConfiguration('vehicle_id'),
        parameters=[
            {
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                'profile_path': LaunchConfiguration('profile_path'),
                'health_check_frequency': 1.0,
                'diagnostics_timeout': 10.0,
            }
        ],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=5.0
    )
    
    # Cloud bridge node
    cloud_bridge_node = Node(
        package='atlasmesh_vehicle_agent',
        executable='cloud_bridge_node',
        name='cloud_bridge',
        namespace=LaunchConfiguration('vehicle_id'),
        parameters=[
            {
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                'cloud_endpoint': LaunchConfiguration('cloud_endpoint'),
                'mqtt_broker': LaunchConfiguration('mqtt_broker'),
                'mqtt_port': 8883,
                'use_tls': True,
                'store_and_forward_enabled': True,
                'max_queue_size': 10000,
                'connection_retry_interval': 30.0,
            }
        ],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=10.0
    )
    
    # OTA manager node
    ota_manager_node = Node(
        package='atlasmesh_vehicle_agent',
        executable='ota_manager_node',
        name='ota_manager',
        namespace=LaunchConfiguration('vehicle_id'),
        parameters=[
            {
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                'update_server_url': 'https://updates.atlasmesh.com',
                'auto_update_enabled': False,  # Manual approval required
                'update_window_start_hour': 2,  # 2 AM
                'update_window_end_hour': 6,    # 6 AM
                'check_interval_hours': 1,
            }
        ],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=30.0,
        condition=IfCondition(LaunchConfiguration('enable_ota_updates'))
    )
    
    # Sensor drivers group (conditional based on simulation)
    sensor_drivers_group = GroupAction([
        # LiDAR driver
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver',
            namespace=LaunchConfiguration('vehicle_id'),
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('atlasmesh_vehicle_agent'),
                    'config', 'sensors', 'velodyne_vlp32c.yaml'
                ])
            ],
            condition=UnlessCondition(LaunchConfiguration('use_simulation'))
        ),
        
        # Camera drivers
        Node(
            package='flir_camera_driver',
            executable='camera_driver_node',
            name='front_camera_driver',
            namespace=LaunchConfiguration('vehicle_id'),
            parameters=[
                {
                    'camera_name': 'front_camera',
                    'serial_number': '20491234',  # Would be vehicle-specific
                    'frame_rate': 30.0,
                    'auto_exposure': True,
                }
            ],
            condition=UnlessCondition(LaunchConfiguration('use_simulation'))
        ),
        
        # GNSS driver
        Node(
            package='novatel_gps_driver',
            executable='novatel_gps_node',
            name='gnss_driver',
            namespace=LaunchConfiguration('vehicle_id'),
            parameters=[
                {
                    'connection_type': 'serial',
                    'device': '/dev/ttyUSB0',
                    'baud': 115200,
                    'frame_id': 'gnss',
                }
            ],
            condition=UnlessCondition(LaunchConfiguration('use_simulation'))
        ),
        
        # IMU driver
        Node(
            package='xsens_mti_driver',
            executable='xsens_mti_node',
            name='imu_driver',
            namespace=LaunchConfiguration('vehicle_id'),
            parameters=[
                {
                    'device': '/dev/ttyUSB1',
                    'baudrate': 921600,
                    'frame_id': 'imu',
                    'pub_imu': True,
                    'pub_mag': True,
                }
            ],
            condition=UnlessCondition(LaunchConfiguration('use_simulation'))
        )
    ])
    
    # Transform publishers for sensor frames
    static_transforms_group = GroupAction([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_transform',
            namespace=LaunchConfiguration('vehicle_id'),
            arguments=['2.0', '0.0', '2.1', '0.0', '0.0', '0.0', 'base_link', 'velodyne']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_front_camera_transform',
            namespace=LaunchConfiguration('vehicle_id'),
            arguments=['2.1', '0.0', '1.8', '0.0', '0.0', '0.0', 'base_link', 'front_camera']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_gnss_transform',
            namespace=LaunchConfiguration('vehicle_id'),
            arguments=['0.0', '0.0', '2.2', '0.0', '0.0', '0.0', 'base_link', 'gnss']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_transform',
            namespace=LaunchConfiguration('vehicle_id'),
            arguments=['0.0', '0.0', '1.0', '0.0', '0.0', '0.0', 'base_link', 'imu']
        )
    ])
    
    # Robot state publisher for vehicle model
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=LaunchConfiguration('vehicle_id'),
        parameters=[
            {
                'robot_description': PathJoinSubstitution([
                    FindPackageShare('atlasmesh_vehicle_agent'),
                    'urdf', 'ugv_themis.urdf'
                ])
            }
        ]
    )
    
    # Diagnostics aggregator
    diagnostics_aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostics_aggregator',
        namespace=LaunchConfiguration('vehicle_id'),
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('atlasmesh_vehicle_agent'),
                'config', 'diagnostics', 'diagnostics.yaml'
            ])
        ]
    )
    
    # RViz for visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=LaunchConfiguration('vehicle_id'),
        arguments=[
            '-d', PathJoinSubstitution([
                FindPackageShare('atlasmesh_vehicle_agent'),
                'rviz', 'vehicle_agent.rviz'
            ])
        ],
        condition=IfCondition(LaunchConfiguration('use_simulation'))
    )
    
    return LaunchDescription([
        # Launch arguments
        vehicle_id_arg,
        vehicle_type_arg,
        profile_path_arg,
        control_frequency_arg,
        telemetry_frequency_arg,
        cloud_endpoint_arg,
        mqtt_broker_arg,
        use_simulation_arg,
        enable_safety_monitor_arg,
        enable_ota_updates_arg,
        log_level_arg,
        
        # Global parameters
        set_log_level,
        
        # Core nodes
        vehicle_agent_node,
        safety_monitor_node,
        health_monitor_node,
        cloud_bridge_node,
        ota_manager_node,
        
        # Sensor drivers (hardware only)
        sensor_drivers_group,
        
        # Transform publishers
        static_transforms_group,
        
        # Support nodes
        robot_state_publisher,
        diagnostics_aggregator,
        
        # Visualization (simulation only)
        rviz_node,
    ])
