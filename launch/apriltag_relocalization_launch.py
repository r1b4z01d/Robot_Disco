"""AprilTag map relocalization: apriltag_ros detector + apriltag_relocalizer.

Requires launch_robot.launch.py (camera + TF) and localization_launch.py (AMCL).

    ros2 launch robot_disco apriltag_relocalization_launch.py
    ros2 service call /apriltag_relocalizer/teach_landmarks std_srvs/srv/Trigger
    ros2 service call /apriltag_relocalizer/relocalize_now std_srvs/srv/Trigger
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share = get_package_share_directory('robot_disco')

    # The RealSense is launched with namespace AND name "camera", hence the
    # doubled path. apriltag_ros's own launch defaults to /camera/color and
    # would silently see nothing.
    camera = LaunchConfiguration('camera')
    detector_config = LaunchConfiguration('detector_config')
    landmarks_file = LaunchConfiguration('landmarks_file')
    auto_correct = LaunchConfiguration('auto_correct')

    detector = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                name='apriltag',
                # The container's namespace does not propagate to the component;
                # without this the detections topic would be /apriltag_detections.
                namespace='apriltag',
                package='apriltag_ros',
                plugin='AprilTagNode',
                parameters=[detector_config],
                # Relative names: the node subscribes to "image"/"camera_info"
                # under its namespace, so absolute "/image" rules would not match.
                remappings=[
                    ('image', [camera, '/image_raw']),
                    ('camera_info', [camera, '/camera_info']),
                ],
            ),
        ],
    )

    relocalizer = Node(
        package='robot_disco',
        executable='apriltag_relocalizer.py',
        name='apriltag_relocalizer',
        output='screen',
        parameters=[{
            'detections_topic': '/apriltag/apriltag_detections',
            'landmarks_file': landmarks_file,
            'auto_correct': auto_correct,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='/camera/camera/color',
                              description='Camera topic prefix (image_raw + camera_info)'),
        DeclareLaunchArgument('detector_config',
                              default_value=os.path.join(share, 'config', 'apriltag_landmarks.yaml')),
        DeclareLaunchArgument('landmarks_file',
                              default_value=os.path.join(share, 'config', 'apriltag_landmark_poses.yaml'),
                              description='Taught map-frame tag poses; written by teach_landmarks. '
                                          'With --symlink-install this is the file in src/.'),
        DeclareLaunchArgument('auto_correct', default_value='true',
                              description='false = only correct AMCL via the relocalize_now service'),
        detector,
        relocalizer,
    ])
