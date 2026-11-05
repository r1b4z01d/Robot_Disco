#!/usr/bin/env python3
"""AprilTag map relocalizer.

Watches apriltag_ros detections. For every detected tag that has a taught
map-frame pose, it works out where the robot must be on the map and, when that
disagrees with AMCL's current estimate by more than a threshold, publishes the
corrected pose on /initialpose.

The pose of each landmark tag in the map frame is *taught*, not measured: park
the robot somewhere AMCL is well localized, point the camera at the tag(s), and
call ~/teach_landmarks. The node records map->tag for every visible tag and
writes them to landmarks_file.

Services (std_srvs/Trigger):
  ~/teach_landmarks   record the map pose of every visible landmark tag from
                      the current AMCL pose and save to landmarks_file
  ~/relocalize_now    publish /initialpose from the current detections,
                      ignoring the deviation threshold and the rate limit

Math (T_a_b = pose of b expressed in a, as a 4x4 homogeneous matrix):
  teach:       T_map_tag  = T_map_base . T_base_tag
  relocalize:  T_map_base = T_map_tag  . inv(T_base_tag)
T_base_tag comes from TF, so the camera mount in the URDF and the detector's
camera->tag transform are used as-is.
"""

import math
import os
import tempfile
import time
from collections import deque

import numpy as np
import rclpy
import yaml
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


# --- small homogeneous-transform helpers (avoids a tf_transformations dep) ---

def quat_to_mat(x, y, z, w):
    """Unit quaternion -> 3x3 rotation matrix."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(R):
    """3x3 rotation matrix -> (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s,
                (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s,
                (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
            0.25 * s, (R[1, 0] - R[0, 1]) / s)


def transform_to_mat(tf):
    """geometry_msgs/TransformStamped -> 4x4."""
    t, q = tf.transform.translation, tf.transform.rotation
    M = np.eye(4)
    M[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
    M[:3, 3] = [t.x, t.y, t.z]
    return M


def pose_to_mat(pose):
    """geometry_msgs/Pose -> 4x4."""
    p, q = pose.position, pose.orientation
    M = np.eye(4)
    M[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
    M[:3, 3] = [p.x, p.y, p.z]
    return M


def mat_to_xy_yaw(M):
    return float(M[0, 3]), float(M[1, 3]), math.atan2(M[1, 0], M[0, 0])


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def circular_mean(angles):
    return math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))


class AprilTagRelocalizer(Node):

    def __init__(self):
        super().__init__('apriltag_relocalizer')

        self.declare_parameter('detections_topic', '/apriltag/apriltag_detections')
        self.declare_parameter('landmarks_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('max_range', 3.0)            # m, camera -> tag
        self.declare_parameter('min_detections', 3)         # agreeing detections before acting
        self.declare_parameter('window_sec', 1.0)           # detections older than this are dropped
        self.declare_parameter('spread_xy', 0.15)           # m, max disagreement inside the window
        self.declare_parameter('spread_yaw', 0.10)          # rad
        self.declare_parameter('auto_correct', True)
        self.declare_parameter('position_threshold', 0.30)  # m, deviation from AMCL before correcting
        self.declare_parameter('yaw_threshold', 0.20)       # rad (~12 deg)
        self.declare_parameter('min_interval', 5.0)         # s between automatic corrections
        self.declare_parameter('initial_pose_std_xy', 0.10)  # covariance handed to AMCL
        self.declare_parameter('initial_pose_std_yaw', 0.10)
        self.declare_parameter('tilt_warn', 0.25)           # rad; roll/pitch above this hints at bad extrinsics

        p = self.get_parameter
        self.landmarks_file = p('landmarks_file').value
        self.map_frame = p('map_frame').value
        self.base_frame = p('base_frame').value
        self.max_range = p('max_range').value
        self.min_detections = p('min_detections').value
        self.window_sec = p('window_sec').value
        self.spread_xy = p('spread_xy').value
        self.spread_yaw = p('spread_yaw').value
        self.auto_correct = p('auto_correct').value
        self.position_threshold = p('position_threshold').value
        self.yaw_threshold = p('yaw_threshold').value
        self.min_interval = p('min_interval').value
        self.std_xy = p('initial_pose_std_xy').value
        self.std_yaw = p('initial_pose_std_yaw').value
        self.tilt_warn = p('tilt_warn').value

        self.landmarks = {}          # tag id -> 4x4 T_map_tag
        self.load_landmarks()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.window = deque()        # (wall time, tag id, x, y, yaw)
        self.last_detections = None  # most recent AprilTagDetectionArray
        self.last_correction = 0.0

        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.estimate_pub = self.create_publisher(PoseStamped, '~/estimated_pose', 10)
        self.create_subscription(AprilTagDetectionArray, p('detections_topic').value,
                                 self.on_detections, 10)
        self.create_service(Trigger, '~/teach_landmarks', self.on_teach)
        self.create_service(Trigger, '~/relocalize_now', self.on_relocalize_now)

        self.get_logger().info(
            f'{len(self.landmarks)} landmark(s) loaded from {self.landmarks_file or "<none>"}; '
            f'auto_correct={self.auto_correct}, thresholds {self.position_threshold} m / '
            f'{math.degrees(self.yaw_threshold):.0f} deg')

    # ----- landmarks file -----

    def load_landmarks(self):
        if not self.landmarks_file or not os.path.exists(self.landmarks_file):
            self.get_logger().warn('No landmarks file; teach some tags with ~/teach_landmarks')
            return
        with open(self.landmarks_file) as f:
            data = yaml.safe_load(f) or {}
        for tag_id, lm in (data.get('landmarks') or {}).items():
            pos, q = lm['position'], lm['orientation']
            M = np.eye(4)
            M[:3, :3] = quat_to_mat(q['x'], q['y'], q['z'], q['w'])
            M[:3, 3] = [pos['x'], pos['y'], pos['z']]
            self.landmarks[int(tag_id)] = M

    def save_landmarks(self):
        out = {'landmarks': {}}
        for tag_id, M in sorted(self.landmarks.items()):
            qx, qy, qz, qw = mat_to_quat(M[:3, :3])
            out['landmarks'][int(tag_id)] = {
                'position': {'x': float(M[0, 3]), 'y': float(M[1, 3]), 'z': float(M[2, 3])},
                'orientation': {'x': float(qx), 'y': float(qy), 'z': float(qz), 'w': float(qw)},
            }
        header = ('# Map-frame poses of AprilTag landmarks, written by the\n'
                  '# apriltag_relocalizer ~/teach_landmarks service. Re-teach after\n'
                  '# re-mapping or moving a tag.\n')
        # Write through to the real file: with `colcon build --symlink-install` the install-space
        # path is a symlink into src/, and an os.replace() on the symlink itself swapped it for a
        # plain file, so taught poses never reached src/ and the next build would have dropped
        # them (RD, 2026-11-01).
        target = os.path.realpath(self.landmarks_file)
        d = os.path.dirname(target) or '.'
        fd, tmp = tempfile.mkstemp(dir=d, suffix='.tmp')
        with os.fdopen(fd, 'w') as f:
            f.write(header)
            yaml.safe_dump(out, f, sort_keys=True)
        os.chmod(tmp, 0o644)   # mkstemp creates 0600
        os.replace(tmp, target)

    # ----- TF helpers -----

    def lookup(self, target, source, stamp):
        # No timeout: the node is single-threaded, so waiting here would block the thread that
        # receives /tf and the wait could never succeed.
        return transform_to_mat(self.tf_buffer.lookup_transform(target, source, stamp))

    def current_map_base(self):
        """AMCL's current estimate, or None if map->base is not available."""
        try:
            return self.lookup(self.map_frame, self.base_frame, Time())
        except TransformException:
            return None

    def base_from_camera(self, msg):
        """T_base_camera for the detection message's camera frame, or None.

        The camera sits on a fixed mount, so the chain is static and any stamp works; the
        message stamp is tried first in case the mount ever becomes articulated.
        """
        cam = msg.header.frame_id
        for stamp in (Time.from_msg(msg.header.stamp), Time()):
            try:
                return self.lookup(self.base_frame, cam, stamp)
            except TransformException as e:
                err = e
        self.get_logger().warn(f'TF {self.base_frame} <- {cam}: {err}', throttle_duration_sec=5.0)
        return None

    def visible_tags(self, msg):
        """Yield (tag_id, T_base_tag) for detections within range.

        The tag pose comes from the detection itself (camera optical frame), composed with the
        static base->camera transform. It deliberately does NOT look up the tag_N frames the
        detector broadcasts: those arrive on /tf a moment after the detection message, so a
        lookup at the detection stamp failed with "extrapolation into the future" on every
        frame (RD, 2026-11-01), and waiting for it in this single-threaded node cannot work.
        """
        if not msg.detections:
            return
        T_base_cam = self.base_from_camera(msg)
        if T_base_cam is None:
            return
        for det in msg.detections:
            pos = det.pose.pose.pose.position
            rng = math.sqrt(pos.x ** 2 + pos.y ** 2 + pos.z ** 2)
            if rng > self.max_range:
                continue
            yield det.id, T_base_cam @ pose_to_mat(det.pose.pose.pose)

    # ----- detection handling -----

    def on_detections(self, msg):
        self.last_detections = msg
        now = time.monotonic()
        for tag_id, T_base_tag in self.visible_tags(msg):
            T_map_tag = self.landmarks.get(tag_id)
            if T_map_tag is None:
                self.get_logger().info(f'Tag {tag_id} seen but not taught',
                                       throttle_duration_sec=10.0)
                continue
            T_map_base = T_map_tag @ np.linalg.inv(T_base_tag)
            self.check_tilt(T_map_base, tag_id)
            x, y, yaw = mat_to_xy_yaw(T_map_base)
            self.window.append((now, tag_id, x, y, yaw))

        while self.window and now - self.window[0][0] > self.window_sec:
            self.window.popleft()

        est = self.consensus()
        if est is None:
            return
        x, y, yaw = est
        self.publish_estimate(x, y, yaw)

        if not self.auto_correct or now - self.last_correction < self.min_interval:
            return
        cur = self.current_map_base()
        if cur is None:
            self.get_logger().warn('No map->base TF (AMCL down?); correcting anyway',
                                   throttle_duration_sec=10.0)
            self.correct(x, y, yaw, 'no AMCL pose')
            return
        cx, cy, cyaw = mat_to_xy_yaw(cur)
        dpos = math.hypot(x - cx, y - cy)
        dyaw = abs(wrap(yaw - cyaw))
        if dpos > self.position_threshold or dyaw > self.yaw_threshold:
            self.correct(x, y, yaw, f'off by {dpos:.2f} m / {math.degrees(dyaw):.1f} deg')

    def consensus(self):
        """Mean of the window if it holds enough agreeing estimates, else None."""
        if len(self.window) < self.min_detections:
            return None
        xs = [w[2] for w in self.window]
        ys = [w[3] for w in self.window]
        yaws = [w[4] for w in self.window]
        mx, my, myaw = sum(xs) / len(xs), sum(ys) / len(ys), circular_mean(yaws)
        if max(math.hypot(x - mx, y - my) for x, y in zip(xs, ys)) > self.spread_xy:
            return None
        if max(abs(wrap(a - myaw)) for a in yaws) > self.spread_yaw:
            return None
        return mx, my, myaw

    def check_tilt(self, T, tag_id):
        R = T[:3, :3]
        pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
        roll = math.atan2(R[2, 1], R[2, 2])
        if abs(roll) > self.tilt_warn or abs(pitch) > self.tilt_warn:
            self.get_logger().warn(
                f'Tag {tag_id}: estimated base roll/pitch {math.degrees(roll):.0f}/'
                f'{math.degrees(pitch):.0f} deg. Check the camera mount in the URDF '
                f'or re-teach the tag.', throttle_duration_sec=10.0)

    # ----- publishing -----

    def pose_msg(self, x, y, yaw):
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        m.pose.pose.orientation.z = math.sin(yaw / 2)
        m.pose.pose.orientation.w = math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = self.std_xy ** 2
        cov[35] = self.std_yaw ** 2
        m.pose.covariance = cov
        return m

    def publish_estimate(self, x, y, yaw):
        m = self.pose_msg(x, y, yaw)
        ps = PoseStamped()
        ps.header = m.header
        ps.pose = m.pose.pose
        self.estimate_pub.publish(ps)

    def correct(self, x, y, yaw, why):
        self.initialpose_pub.publish(self.pose_msg(x, y, yaw))
        self.last_correction = time.monotonic()
        tags = sorted({w[1] for w in self.window})
        self.get_logger().info(
            f'Relocalized to ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg) from tag(s) {tags}: {why}')

    # ----- services -----

    def on_teach(self, _req, res):
        msg = self.last_detections
        if msg is None or (self.get_clock().now() - Time.from_msg(msg.header.stamp)) > Duration(seconds=1.0):
            res.message = 'No recent detections'
            return res
        T_map_base = self.current_map_base()
        if T_map_base is None:
            res.message = f'No {self.map_frame}->{self.base_frame} TF; is AMCL running and localized?'
            return res
        taught = []
        for tag_id, T_base_tag in self.visible_tags(msg):
            self.landmarks[tag_id] = T_map_base @ T_base_tag
            taught.append(tag_id)
        if not taught:
            res.message = 'No landmark tags in view within max_range'
            return res
        if not self.landmarks_file:
            res.message = f'Taught {taught} in memory only; landmarks_file is not set'
            res.success = True
            return res
        try:
            self.save_landmarks()
        except OSError as e:
            res.message = f'Taught {taught} but could not write {self.landmarks_file}: {e}'
            return res
        res.success = True
        res.message = f'Taught tag(s) {taught} -> {self.landmarks_file}'
        self.get_logger().info(res.message)
        return res

    def on_relocalize_now(self, _req, res):
        est = self.consensus()
        if est is None:
            res.message = (f'Need {self.min_detections} agreeing detections of a taught tag '
                           f'in the last {self.window_sec}s; have {len(self.window)}')
            return res
        self.correct(*est, 'requested')
        res.success = True
        res.message = 'Published /initialpose'
        return res


def main():
    rclpy.init()
    node = AprilTagRelocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl-C / supervisor SIGTERM
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
