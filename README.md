# robot_disco

The mobile base of Robot Disco: a differential-drive robot on RoboClaw motor controllers with two RPLiDARs (S1 front, S2E rear), a Velodyne, a RealSense D435 and a UR5e arm with the AmazingHand on top. This package holds its description, bringup, localization, navigation and the AprilTag relocalizer; it is a submodule of the `rd_ws` workspace (github.com/r1b4z01d/rd_ws), where every launch file below runs as a supervisor unit (`robot_disco*`) and the workspace README documents the whole robot.

| Launch file | What it starts |
|---|---|
| `launch_robot.launch.py` | bringup: `rsp.launch.py` (URDF from `description/robot.urdf.xacro`: base, lidars, D435 pitched 46° up, UR mount + hand), ros2_control with the RoboClaw diff drive (`config/my_controllers.yaml`), twist_mux, the joystick, the D435, both RPLiDARs with their box filters, the Velodyne, `scripts/ur_joint_state_publisher.py` |
| `localization_launch.py` | AMCL on `maps/zone_one.yaml` (`map:=` overrides) |
| `online_async_launch.py` | slam_toolbox (`config/mapper_params_online_async.yaml`), continues `maps/zone_one.posegraph`; never together with AMCL |
| `navigation_launch.py` | Nav2 (`config/nav2_params.yaml`, `behavior_trees/`): both costmaps from the filtered RPLiDAR scans, the local one also from the Velodyne |
| `apriltag_relocalization_launch.py` | apriltag_ros on the D435 (`config/apriltag_landmarks.yaml`) + `scripts/apriltag_relocalizer.py`, which corrects AMCL from tags with taught map poses (`config/apriltag_landmark_poses.yaml`, written by its `~/teach_landmarks` service) |
| `ur5e.launch.py` | the ROS UR driver, namespaced `/ur5e` (only for ROS-side arm work; the VLA tools and the face's wave drive the arm over RTDE) |
| `foxglove_bridge_launch.xml` | Foxglove WebSocket bridge, whitelisted topics (manual-start; the route editor's /3d page is the daily view) |
| `joystick.launch.py`, `rplidar_s1_launch.py`, `rplidar_s2e_launch.py`, `rplidar_*_filter_launch.py`, `velodyne.launch.py`, `realsense_launch.py`, `rsp.launch.py` | the pieces of the bringup, usable alone |

**Joystick** (`config/joystick.yaml`): an 8BitDo Ultimate 2C on its 2.4 GHz receiver, matched by `device_name` (the rebuilt xpad from `rd_ws/provision/xpad` names it "8BitDo Ultimate 2C Wireless Controller"). Hold RB (button 5) to drive, LB (4) for turbo; `rd_ws/Scripts/path.py` treats either press as its deadman.

**Collision monitor** (`config/collision_monitor_params.yaml`): the nav2 collision monitor gates every velocity source (`twist_mux` → `/cmd_vel_raw` → monitor → `/diff_cont/cmd_vel_unstamped`), so the robot does not move without it; it is the one autostart unit besides the route editor.

**Maps** (`maps/`): `zone_one` is the active map (`.pgm`/`.yaml` for AMCL, `.posegraph`/`.data` for slam_toolbox, `.xcf` source); re-record every map-frame pose (routes, landmarks, taught tags) after a fresh map.

**Scripts** (`scripts/`): `apriltag_relocalizer.py`; `ur_joint_state_publisher.py` (UR joints as `/joint_states` from a read-only RTDE connection, with a home-pose fallback while the arm is off, so the arm stays in the description during VLA runs); `bake_amazing_hand.py` (hand meshes for the description). The copies of the face node under `scripts/` are leftovers from before `robot_face` became its own package.

## License

[CC BY-NC-SA 4.0](LICENSE). The nav2 / realsense launch files under `launch/` started from the upstream
templates and keep their Apache-2.0 headers.
