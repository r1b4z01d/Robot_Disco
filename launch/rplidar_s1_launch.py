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
    # By-id path, not /dev/ttyUSB0: the S1's CP2102 adapter re-enumerates as ttyUSB1 after a USB drop
    # (seen twice in one boot on 2026-10-27), and the node then died on every launch.
    s1_serial_port = LaunchConfiguration(
        's1_serial_port',
        default='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
    s1_serial_baudrate = LaunchConfiguration('s1_serial_baudrate', default='256000') 
    s1_frame_id = LaunchConfiguration('s1_frame_id', default='rp_s1_lidar')
    s1_inverted = LaunchConfiguration('s1_inverted', default='true')
    s1_topic_name = LaunchConfiguration('s1_topic_name', default='rp_s1_lidar/scan')
    s1_angle_compensate = LaunchConfiguration('s1_angle_compensate', default='true')
    # Scan mode: left empty the SDK picks DenseBoost (9 kHz); Standard is only 4 kHz on this unit. Tested 2026-11-02:
    # the S1's ranges came back a constant ~0.33 m too long in BOTH modes (box at a taped 0.50 m -> 0.83, 1.00 ->
    # 1.32), so the mode is not the cause; the offset is corrected by the scan_range_offset node below.
    s1_scan_mode = LaunchConfiguration('s1_scan_mode', default='')
    # The driver publishes the uncorrected scan; scan_range_offset.py republishes it with the measured
    # correction on the topic everything else reads (filter chain, map pages, --raw-scans).
    s1_range_offset = LaunchConfiguration('s1_range_offset', default='-0.33')

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

        DeclareLaunchArgument(
            's1_scan_mode',
            default_value=s1_scan_mode,
            description='RPLidar scan mode (empty = the SDK picks DenseBoost; Standard is the 4 kHz fallback)'),

        DeclareLaunchArgument(
            's1_range_offset',
            default_value=s1_range_offset,
            description='metres ADDED to every S1 range (negative shortens); re-measure with a box at a taped distance; 0.0 for a healthy unit'),

        Node(
            package='rplidar_ros',
            # the driver exits on a USB hiccup (the S1's CP2102 re-enumerates) — come back by itself (2026-10-28)
            respawn=True,
            respawn_delay=3.0,
            executable='rplidar_node',
            name='rplidar_s1_node',
            parameters=[{'channel_type':s1_channel_type,
                         'serial_port': s1_serial_port,
                         'serial_baudrate': s1_serial_baudrate,
                         'frame_id': s1_frame_id,
                         'inverted': s1_inverted,
                         'angle_compensate': s1_angle_compensate,
                         'scan_mode': s1_scan_mode,
                         'topic_name': [s1_topic_name, '_uncorrected'] }],
            output='screen'),

        Node(
            package='robot_disco',
            executable='scan_range_offset.py',
            name='rplidar_s1_range_offset',
            respawn=True,
            respawn_delay=3.0,
            parameters=[{'in_topic': [s1_topic_name, '_uncorrected'],
                         'out_topic': s1_topic_name,
                         'offset': s1_range_offset}],
            output='screen'),
    ])

