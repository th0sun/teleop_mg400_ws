#!/usr/bin/env python3
"""
Launch file สำหรับ teleop_mg400_ws
เปิดทั้ง Unity bridge, Teleop logic node พร้อมกัน
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # === Arguments ===
        DeclareLaunchArgument('ros_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument('ros_port', default_value='10000'),

        # === Unity Bridge (ROS-TCP-Endpoint) ===
        Node(
            package='ros_tcp_endpoint',
            executable='default_server_endpoint',
            name='unity_bridge',
            parameters=[{
                'ROS_IP': LaunchConfiguration('ros_ip'),
                'ROS_TCP_PORT': LaunchConfiguration('ros_port'),
            }],
            output='screen',
        ),

        # === Teleop Logic Node ===
        Node(
            package='teleop_logic',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
        ),
    ])
