#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    s1_channel_type = LaunchConfiguration('s1_channel_type', default='serial')
    s1_serial_port = LaunchConfiguration('s1_serial_port', default='/dev/ttyUSB0')
    s1_serial_baudrate = LaunchConfiguration('s1_serial_baudrate', default='256000') 
    s1_frame_id = LaunchConfiguration('s1_frame_id', default='rp_s1_lidar')
    s1_inverted = LaunchConfiguration('s1_inverted', default='true')
    s1_topic_name = LaunchConfiguration('s1_topic_name', default='rp_s1_lidar/scan')
    s1_angle_compensate = LaunchConfiguration('s1_angle_compensate', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            's1_channel_type',
            default_value=s1_channel_type,
            description='Specifying channel type of lidar'),
        
        DeclareLaunchArgument(
            's1_serial_port',
            default_value=s1_serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            's1_serial_baudrate',
            default_value=s1_serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            's1_frame_id',
            default_value=s1_frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            's1_inverted',
            default_value=s1_inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            's1_angle_compensate',
            default_value=s1_angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            's1_topic_name',
            default_value=s1_topic_name,
            description='Specifying topic_name the node should publish to.'),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_s1_node',
            parameters=[{'channel_type':s1_channel_type,
                         'serial_port': s1_serial_port,
                         'serial_baudrate': s1_serial_baudrate,
                         'frame_id': s1_frame_id,
                         'inverted': s1_inverted,
                         'angle_compensate': s1_angle_compensate,
                         'topic_name': s1_topic_name }],
            output='screen'),
    ])

