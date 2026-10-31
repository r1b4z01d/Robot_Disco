import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():
    package_name='robot_disco' 
    
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )])
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            # Output goes through the collision monitor, which publishes
            # /diff_cont/cmd_vel_unstamped (see config/collision_monitor_params.yaml).
            remappings=[('/cmd_vel_out','/cmd_vel_raw')],
            arguments=['--ros-args','--log-level','WARN']
        )

    velodyne = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("robot_disco"),'launch','velodyne.launch.py'
                )])
    )

    rp_s2e_lidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("robot_disco"),'launch','rplidar_s2e_launch.py'
                )])
    )
    
    rp_s2e_lidar_filter = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("robot_disco"),'launch','rplidar_s2e_filter_launch.py'
                )])
    )
    
    rp_s1_lidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("robot_disco"),'launch','rplidar_s1_launch.py'
                )])
    )  

    rp_s1_lidar_filter = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("robot_disco"),'launch','rplidar_s1_filter_launch.py'
                )])
    )
    realsense_d435 = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("robot_disco"),'launch','realsense_launch.py'
                )]),
                 launch_arguments={
                    'pointcloud.enable': 'True',
                    'device_type' : 'd435',
                    # 1280x720 at 15 fps: on USB 3 the D435i defaults to 30 fps (~78 MB/s raw), which
                    # overruns the Foxglove bridge's send buffer over Wi-Fi; 15 fps is what the old
                    # USB-2 link gave and all robot_face needs (2026-10-11).
                    'rgb_camera.color_profile': '1280,720,15',
                }.items()
    )

    # Build the description from the xacro directly — the same command rsp.launch.py runs — instead
    # of `ros2 param get /robot_state_publisher robot_description`. That shell-out was evaluated when
    # the 3 s timer below fired, and whenever robot_state_publisher had not finished its own xacro +
    # DDS discovery by then it failed with "executed command failed", launch dropped this ONE action
    # and brought everything else up: no controller_manager, no diff_cont, no /odom, no odom->base_link
    # TF. AMCL then has no odometry chain, never publishes map->odom or /amcl_pose, and every pose
    # estimate from Foxglove/RViz is silently ignored — the map never moves. Seen on the robot PC
    # 2026-10-08 (RD log: 30+ such failures over its history). Generating it here has no dependency
    # on another node, so it cannot race.
    xacro_file = os.path.join(get_package_share_directory(package_name), 'description', 'robot.urdf.xacro')
    # ParameterValue(value_type=str): without it launch_ros tries to parse the URDF as YAML and
    # on Humble can fail with "Unable to parse the value of parameter robot_description as yaml"
    # (seen on the Jetson robot PC 2026-10-10 with the apt ur_description).
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_ros2_control:=true sim_mode:=false']), value_type=str)

    # The UR5e's joints in the base description (ur_mount.xacro, tf_prefix "ur_") get their angles
    # from a read-only RTDE poll, so the arm moves in Foxglove/TF during a VLA eval or pendant use,
    # when the ROS UR driver is not (and must not be) running. Harmless when the arm is off: it retries.
    ur_joint_states = Node(
        package=package_name,
        executable='ur_joint_state_publisher.py',
        parameters=[{'robot_ip': '192.168.11.21', 'rate_hz': 15.0, 'tf_prefix': 'ur_',
                     'fallback': 'home'}],   # arm off -> home pose joint state (keeps the model whole)
        respawn=True,
        respawn_delay=2.0,
        arguments=['--ros-args', '--log-level', 'WARN'],
    )

    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')
    # respawn: the RoboClaw's USB link drops now and then (cdc_acm "device not accepting address,
    # error -71" on the hub, 2026-10-30) and ros2_control_node dies with it; without respawn the base
    # stays without odom until someone restarts the whole launch. OnProcessStart below fires again on
    # each respawn, so the spawners re-attach the controllers.
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file],
        arguments=['--ros-args','--log-level','WARN'],
        respawn=True,
        respawn_delay=2.0,
    )
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['diff_cont','--ros-args','--log-level','WARN'],
    )
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad",'--ros-args','--log-level','WARN'],
    )
    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        ur_joint_states,
        joystick,
        twist_mux,
        velodyne,
        realsense_d435,
        rp_s1_lidar,
        rp_s2e_lidar,
        rp_s1_lidar_filter,
        rp_s2e_lidar_filter,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])
    
