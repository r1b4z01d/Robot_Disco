# ur5e_namespaced.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        GroupAction([
            PushRosNamespace('ur5e'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('ur_robot_driver') + '/launch/ur_control.launch.py'
                ]),
                launch_arguments={
                    'launch_rviz': 'false',
                    'ur_type': 'ur5e',
                    'robot_ip': '192.168.11.21',
                    'use_tool_communication': 'false'
                }.items()
            )
        ])
    ])
