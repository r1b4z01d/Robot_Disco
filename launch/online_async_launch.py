import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams


def _maps_dir(bringup_dir):
    """The maps folder the route editor writes to. With a symlink install the share dir holds per-file
    symlinks into the source tree, so files written there later (active_map, slam_request.yaml, new maps)
    would be invisible here: follow a known map's symlink to the real folder."""
    share = os.path.join(bringup_dir, 'maps')
    for probe in ('zone_one.yaml', 'keepout_mask.yaml'):
        p = os.path.join(share, probe)
        if os.path.islink(p):
            return os.path.dirname(os.path.realpath(p))
    return share


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_params_file = os.path.join(get_package_share_directory("robot_disco"),
                                       'config', 'mapper_params_online_async.yaml')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # If the provided param file doesn't have slam_toolbox params, we must pass the
    # default_params_file instead. This could happen due to automatic propagation of
    # LaunchArguments. See:
    # https://github.com/ros-planning/navigation2/pull/2243#issuecomment-800479866
    has_node_params = HasNodeParams(source_file=params_file,
                                    node_name='slam_toolbox')

    actual_params_file = PythonExpression(['"', params_file, '" if ', has_node_params,
                                           ' else "', default_params_file, '"'])

    log_param_change = LogInfo(msg=['provided params_file ',  params_file,
                                    ' does not contain slam_toolbox parameters. Using default: ',
                                    default_params_file],
                               condition=UnlessCondition(has_node_params))

    # maps/slam_request.yaml (written by the manager's Maps tab through route_editor.py) says whether this
    # run maps from scratch or continues an existing map's posegraph; without it: a fresh map.
    maps_dir = _maps_dir(get_package_share_directory("robot_disco"))
    slam_overrides = {'mode': 'mapping', 'map_file_name': '', 'map_start_at_dock': False}
    try:
        import yaml
        req = yaml.safe_load(open(os.path.join(maps_dir, 'slam_request.yaml'))) or {}
        if req.get('mode') == 'continue' and req.get('name') and os.path.exists(os.path.join(maps_dir, req['name'] + '.posegraph')):
            slam_overrides = {'mode': 'mapping', 'map_file_name': os.path.join(maps_dir, req['name']), 'map_start_at_dock': True}
    except Exception:  # noqa: BLE001 — no request, or a bad one: fresh map
        pass

    start_async_slam_toolbox_node = Node(
        parameters=[
          actual_params_file,
          {'use_sim_time': use_sim_time},
          slam_overrides
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(log_param_change)
    ld.add_action(start_async_slam_toolbox_node)

    return ld
