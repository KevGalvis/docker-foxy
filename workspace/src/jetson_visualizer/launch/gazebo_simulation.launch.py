#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default='empty.world')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'verbose': 'true',
            'world': world_file,
        }.items(),
    )
    
    # Bridge nodes to connect with Jetson topics
    # These nodes will re-publish topics from the Jetson to match expected formats in Gazebo
    
    # TF static publisher for gazebo frames
    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sim_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Create a bridge node to republish sensor data into gazebo compatible formats if needed
    sensor_bridge = Node(
        package='jetson_visualizer',  # You would need to implement this
        executable='sensor_bridge',
        name='sensor_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'world_file',
            default_value='empty.world',
            description='Gazebo world file to load'
        ),
        
        gazebo,
        tf_publisher,
        # Comment out sensor_bridge until implemented
        # sensor_bridge,
    ])