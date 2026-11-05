from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            remappings=[
                ('/scan', '/rp_s2e_lidar/scan'),
                ('/scan_filtered', '/rp_s2e_lidar/scan_filtered'),
            ],
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("robot_disco"),
                    "config", "rplidar_s2e_filter.yaml"])],
        )
    ])