#!/usr/bin/env python3
"""
ROS 2 node wrapper around the original combined_face_hand.py.

It subscribes to a color image topic (default: /camera/camera/color/image_raw),
runs MediaPipe pose + hand tracking, renders the animated eyes window, and
optionally relays joint angles over TCP.
"""

import argparse
import math
import socket
import time
from typing import Optional

import cv2
import mediapipe as mp
import numpy as np
import rclpy
import rclpy.utilities
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

from eye_renderer import draw_eye
from face_utils import box_top_center, pick_face, pose_box_from_landmarks, schedule_blink
from hand_processing import JOINT_OPEN_OFFSETS, format_joint_command, generate_joint_offsets
from window_utils import is_global_key_pressed, set_windows_window_frame_color, supports_global_hotkeys


# ---------- MediaPipe Hands setup ----------
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
mp_pose = mp.solutions.pose


def parse_args(raw_args=None):
    """Parse CLI args that are not ROS remapping arguments."""
    parser = argparse.ArgumentParser(description="Eyes + One-hand tracking on a ROS2 image topic")
    parser.add_argument(
        "--image_topic",
        type=str,
        default="/camera/camera/color/image_raw",
        help="Image topic to subscribe to (sensor_msgs/Image)",
    )
    parser.add_argument("--width", type=int, default=1280, help="Fallback camera width if msg lacks info")
    parser.add_argument("--height", type=int, default=720, help="Fallback camera height if msg lacks info")
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Assumed frame rate when writing video files (topic may not include FPS)",
    )
    parser.add_argument("--mirror", action="store_true", help="Mirror hand visuals (selfie style)")
    parser.add_argument(
        "--hand_in_eyes",
        action="store_true",
        help="Overlay hand landmarks in a panel below the eyes",
    )
    parser.add_argument(
        "--hand_panel_height",
        type=int,
        default=400,
        help="Height in pixels for the hand panel under the eyes",
    )
    parser.add_argument(
        "--hand_panel_scale",
        type=float,
        default=1.0,
        help="Scale factor for in-eyes hand overlay rendering",
    )
    parser.add_argument(
        "--blink_min",
        type=float,
        default=2.5,
        help="Minimum seconds between blinks",
    )
    parser.add_argument(
        "--blink_max",
        type=float,
        default=15.5,
        help="Maximum seconds between blinks",
    )
    parser.add_argument(
        "--blink_duration",
        type=float,
        default=0.18,
        help="Blink duration in seconds",
    )
    parser.add_argument(
        "--gaze_speed",
        type=float,
        default=0.3,
        help="Smoothing factor (0-1) controlling how fast the eyes chase a tracked target",
    )
    parser.add_argument(
        "--gaze_idle_speed",
        type=float,
        default=0.01,
        help="Smoothing factor (0-1) controlling eye speed during idle wandering",
    )
    parser.add_argument(
        "--relay_hand",
        action="store_true",
        help="Stream computed hand joints to the wireless hand over TCP",
    )
    parser.add_argument("--hand_host", type=str, default="192.168.1.194", help="Hand relay TCP host")
    parser.add_argument("--hand_port", type=int, default=8765, help="Hand relay TCP port")
    parser.add_argument("--hand_rate", type=float, default=20.0, help="Maximum hand relay updates per second")
    parser.add_argument(
        "--hand_speed",
        type=int,
        default=1000,
        help="Servo speed value embedded in joint messages",
    )
    parser.add_argument(
        "--save_raw_video",
        type=str,
        nargs="?",
        const="raw_camera.mp4",
        default=None,
        help="Save the raw camera feed to a video file (default raw_camera.mp4 if no path is provided)",
    )
    parser.add_argument(
        "--save_video",
        type=str,
        nargs="?",
        const="combined_output.mp4",
        default=None,
        help="Save the eyes window output to a video file (default combined_output.mp4 if no path is provided)",
    )
    return parser.parse_args(raw_args)


class FaceHandNode(Node):
    def __init__(self, parsed_args):
        super().__init__("combined_face_hand")
        self.args = parsed_args
        self.bridge = CvBridge()

        # Camera/image state
        self.camera_width = parsed_args.width
        self.camera_height = parsed_args.height
        self.capture_fps = parsed_args.fps if parsed_args.fps > 0 else 30.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, parsed_args.image_topic, self.image_callback, qos)

        # Hand relay state
        self.hand_stream_enabled = parsed_args.relay_hand
        self.hand_stream_hotkey = "s"
        self.global_hotkey_supported = supports_global_hotkeys()
        self.hand_stream_toggle_active = False
        self.hotkey_prev_state = False
        self.hand_target_speed = int(min(max(parsed_args.hand_speed, 50), 2000))
        self.hand_socket: Optional[socket.socket] = None
        self.last_hand_send = 0.0
        self.hand_send_interval = 1.0 / max(parsed_args.hand_rate, 1e-5)
        self.hand_idle_timeout = 0.75
        self.hand_last_detection_time = time.monotonic()
        self.hand_open_sent = False

        if self.hand_stream_enabled:
            if self.global_hotkey_supported:
                self.get_logger().info(
                    f"Hand relay armed. Press '{self.hand_stream_hotkey}' to toggle streaming "
                    "(works even when the window is unfocused)."
                )
            else:
                self.hand_stream_toggle_active = True
                self.get_logger().info(
                    "Hand relay armed but global hotkeys unavailable; streaming continuously."
                )

        # Eyes canvas params
        self.eye_canvas_width = 1080
        self.eye_canvas_height = 1900
        self.window_name = "Robot Disco"
        self.eye_radius = 90
        self.eye_white_radius = 220
        self.eye_top_margin = 80
        self.eye_side_margin = 80
        self.eyelid_color = (30, 30, 30)
        self.draw_eyelashes = True
        self.eyelash_color = (50, 50, 55)
        self.eyelash_count = 7
        self.eyelash_length_top = 55
        self.eyelash_length_side = 20
        self.eyelash_thickness = 4

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow(self.window_name, self.eye_canvas_width, self.eye_canvas_height)
        set_windows_window_frame_color(self.window_name, (0, 0, 0))

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.raw_video_writer = (
            cv2.VideoWriter(
                parsed_args.save_raw_video, fourcc, self.capture_fps, (self.camera_width, self.camera_height)
            )
            if parsed_args.save_raw_video
            else None
        )
        if self.raw_video_writer is not None and not self.raw_video_writer.isOpened():
            self.get_logger().warning(f"Could not open raw video writer at {parsed_args.save_raw_video}")
            self.raw_video_writer = None

        self.video_writer = (
            cv2.VideoWriter(
                parsed_args.save_video,
                fourcc,
                self.capture_fps,
                (self.eye_canvas_width, self.eye_canvas_height),
            )
            if parsed_args.save_video
            else None
        )
        if self.video_writer is not None and not self.video_writer.isOpened():
            self.get_logger().warning(f"Could not open video writer at {parsed_args.save_video}")
            self.video_writer = None

        # Face tracking state
        self.tracked_face = None
        self.last_seen_time = time.time()
        self.face_stick_seconds = 1.0
        self.idle_scan_delay_seconds = 4.0
        self.idle_scan_period_seconds = 8.0
        self.idle_scan_horizontal_fraction = 0.25
        self.idle_scan_vertical_fraction = 0.12
        self.smoothed_center = None

        # Blink state
        self.blink_min_interval_seconds = parsed_args.blink_min
        self.blink_max_interval_seconds = parsed_args.blink_max
        self.blink_duration_seconds = parsed_args.blink_duration
        self.blink_start_time = None
        self.next_blink_time = schedule_blink(
            self.blink_min_interval_seconds, self.blink_max_interval_seconds
        )

        # Mediapipe detectors
        self.pose_detector = mp_pose.Pose(
            model_complexity=0,
            enable_segmentation=False,
            smooth_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.hands_detector = mp_hands.Hands(
            max_num_hands=1,
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

    # ---------- Utility ----------
    def clamp_speed(self, val):
        return min(max(val, 0.01), 1.0)

    # ---------- Image callback ----------
    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().warning(f"CvBridge failed: {exc}")
            return

        self.camera_height, self.camera_width = frame.shape[:2]
        if self.raw_video_writer is not None:
            self.raw_video_writer.write(frame)

        current_time = time.time()
        current_mono = time.monotonic()

        # ---- Person detection for eyes ----
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb_frame.flags.writeable = False
        pose_results = self.pose_detector.process(rgb_frame)
        rgb_frame.flags.writeable = True
        faces = []
        if pose_results and pose_results.pose_landmarks:
            pose_box = pose_box_from_landmarks(
                pose_results.pose_landmarks, self.camera_width, self.camera_height
            )
            if pose_box is not None:
                faces.append(pose_box)

        selected_face = pick_face(faces, self.tracked_face)
        if selected_face is not None:
            self.tracked_face = selected_face
            self.last_seen_time = current_time
        elif current_time - self.last_seen_time > self.face_stick_seconds:
            self.tracked_face = None

        time_since_face = current_time - self.last_seen_time
        if self.tracked_face is not None:
            center = box_top_center(self.tracked_face)
        elif time_since_face > self.idle_scan_delay_seconds:
            scan_elapsed = time_since_face - self.idle_scan_delay_seconds
            sweep_angle = (2 * math.pi / self.idle_scan_period_seconds) * scan_elapsed
            center = (
                self.camera_width / 2.0
                + math.sin(sweep_angle) * self.camera_width * self.idle_scan_horizontal_fraction,
                self.camera_height / 2.0
                + math.cos(sweep_angle * 0.7) * self.camera_height * self.idle_scan_vertical_fraction,
            )
        else:
            center = (self.camera_width / 2.0, self.camera_height / 2.0)

        if self.smoothed_center is None:
            self.smoothed_center = center
        else:
            smoothing_factor_active = self.clamp_speed(self.args.gaze_speed)
            smoothing_factor_idle = self.clamp_speed(self.args.gaze_idle_speed)
            current_smoothing = smoothing_factor_active if self.tracked_face is not None else smoothing_factor_idle
            self.smoothed_center = (
                self.smoothed_center[0] + (center[0] - self.smoothed_center[0]) * current_smoothing,
                self.smoothed_center[1] + (center[1] - self.smoothed_center[1]) * current_smoothing,
            )

        if self.blink_start_time is None and current_mono >= self.next_blink_time:
            self.blink_start_time = current_mono

        if self.blink_start_time is not None:
            elapsed = current_mono - self.blink_start_time
            half = self.blink_duration_seconds / 2.0
            if elapsed >= self.blink_duration_seconds:
                self.blink_start_time = None
                self.next_blink_time = schedule_blink(
                    self.blink_min_interval_seconds, self.blink_max_interval_seconds
                )
                blink_amount = 0.0
            else:
                if elapsed <= half:
                    blink_amount = min(1.0, elapsed / half)
                else:
                    blink_amount = max(0.0, 1.0 - ((elapsed - half) / half))
        else:
            blink_amount = 0.0

        # Gaze mapping normalized by camera dims
        norm_x = (self.smoothed_center[0] - self.camera_width / 2.0) / (self.camera_width / 2.0)
        norm_y = (self.smoothed_center[1] - self.camera_height / 2.0) / (self.camera_height / 2.0)
        norm_x = max(min(norm_x, 1.0), -1.0)
        norm_y = max(min(norm_y, 1.0), -1.0)

        # Scale by usable pupil travel within the sclera
        max_travel = max(self.eye_white_radius - self.eye_radius, 0)
        gaze_gain_x = 0.7
        gaze_gain_y = 0.6
        offsetX = -norm_x * max_travel * gaze_gain_x  # mirror horizontally
        offsetY = norm_y * max_travel * gaze_gain_y

        # Determine if we need hands processing (for in-eyes overlay or hotkey-triggered streaming)
        if self.hand_stream_enabled and self.global_hotkey_supported:
            hotkey_pressed = is_global_key_pressed(self.hand_stream_hotkey)
            if hotkey_pressed and not self.hotkey_prev_state:
                self.hand_stream_toggle_active = not self.hand_stream_toggle_active
                state_str = "enabled" if self.hand_stream_toggle_active else "paused"
                self.get_logger().info(f"Hand relay {state_str} via '{self.hand_stream_hotkey}' toggle.")
            self.hotkey_prev_state = hotkey_pressed
        elif self.hand_stream_enabled and not self.global_hotkey_supported:
            self.hand_stream_toggle_active = True

        hand_stream_active = self.hand_stream_enabled and self.hand_stream_toggle_active
        need_hands = self.args.hand_in_eyes or hand_stream_active
        results = None
        if need_hands:
            rgb_frame.flags.writeable = False
            results = self.hands_detector.process(rgb_frame)
            rgb_frame.flags.writeable = True

        if hand_stream_active:
            hand_payload = None
            if results and results.multi_hand_landmarks:
                joint_offsets = generate_joint_offsets(results.multi_hand_landmarks[0])
                hand_payload = format_joint_command(joint_offsets, self.hand_target_speed)
                self.hand_last_detection_time = current_mono
                self.hand_open_sent = False
            elif (current_mono - self.hand_last_detection_time) > self.hand_idle_timeout and not self.hand_open_sent:
                hand_payload = format_joint_command(JOINT_OPEN_OFFSETS, self.hand_target_speed)
                self.hand_open_sent = True

            if hand_payload is not None and (current_mono - self.last_hand_send) >= self.hand_send_interval:
                if self.hand_socket is None:
                    try:
                        self.hand_socket = socket.create_connection(
                            (self.args.hand_host, self.args.hand_port), timeout=0.5
                        )
                        self.hand_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    except OSError:
                        self.hand_socket = None
                if self.hand_socket is not None:
                    try:
                        self.hand_socket.sendall(hand_payload.encode("ascii"))
                        self.last_hand_send = current_mono
                    except OSError:
                        self.hand_socket.close()
                        self.hand_socket = None

        # Prepare eyes canvas
        eye_canvas = np.zeros((self.eye_canvas_height, self.eye_canvas_width, 3), dtype=np.uint8)
        eye_vertical_center = self.eye_top_margin + self.eye_white_radius
        left_eye_center = (
            self.eye_side_margin + self.eye_white_radius,
            eye_vertical_center,
        )
        right_eye_center = (
            self.eye_canvas_width - self.eye_side_margin - self.eye_white_radius,
            eye_vertical_center,
        )
        right_center_with_offset = (
            int(right_eye_center[0] + offsetX),
            int(right_eye_center[1] + offsetY),
        )
        left_center_with_offset = (
            int(left_eye_center[0] + offsetX),
            int(left_eye_center[1] + offsetY),
        )
        draw_eye(
            eye_canvas,
            right_eye_center,
            self.eye_white_radius,
            self.eye_radius,
            (
                right_center_with_offset[0] - right_eye_center[0],
                right_center_with_offset[1] - right_eye_center[1],
            ),
            blink_amount,
            self.eyelid_color,
            self.draw_eyelashes,
            self.eyelash_color,
            self.eyelash_count,
            self.eyelash_length_top,
            self.eyelash_length_side,
            self.eyelash_thickness,
        )
        draw_eye(
            eye_canvas,
            left_eye_center,
            self.eye_white_radius,
            self.eye_radius,
            (
                left_center_with_offset[0] - left_eye_center[0],
                left_center_with_offset[1] - left_eye_center[1],
            ),
            blink_amount,
            self.eyelid_color,
            self.draw_eyelashes,
            self.eyelash_color,
            self.eyelash_count,
            self.eyelash_length_top,
            self.eyelash_length_side,
            self.eyelash_thickness,
        )

        if self.args.hand_in_eyes and results is not None and results.multi_hand_landmarks:
            panel_h = max(600, min(self.args.hand_panel_height, self.eye_canvas_height))
            y0 = self.eye_canvas_height - panel_h
            x0 = 0
            hand_panel = eye_canvas[y0 : self.eye_canvas_height, x0 : self.eye_canvas_width]
            panel_img = np.zeros_like(hand_panel)
            mp_drawing.draw_landmarks(
                panel_img,
                results.multi_hand_landmarks[0],
                mp_hands.HAND_CONNECTIONS,
                mp_drawing_styles.get_default_hand_landmarks_style(),
                mp_drawing_styles.get_default_hand_connections_style(),
            )
            if self.args.mirror:
                panel_img = cv2.flip(panel_img, 1)
            panel_scale = max(self.args.hand_panel_scale, 1.0)
            if panel_scale != 1.0:
                scaled_img = cv2.resize(
                    panel_img,
                    None,
                    fx=panel_scale,
                    fy=panel_scale,
                    interpolation=cv2.INTER_LINEAR,
                )
                target_h, target_w = hand_panel.shape[:2]
                if scaled_img.shape[0] >= target_h and scaled_img.shape[1] >= target_w:
                    start_y = (scaled_img.shape[0] - target_h) // 2
                    start_x = (scaled_img.shape[1] - target_w) // 2
                    panel_img = scaled_img[start_y : start_y + target_h, start_x : start_x + target_w]
                else:
                    panel_img = cv2.resize(scaled_img, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
            hand_panel[:] = panel_img

        if self.video_writer is not None:
            self.video_writer.write(eye_canvas)

        cv2.imshow(self.window_name, eye_canvas)
        key_code = cv2.waitKey(1) & 0xFF
        if key_code == ord("q"):
            self.get_logger().info("Quit requested via keypress; shutting down.")
            rclpy.shutdown()

    # ---------- Cleanup ----------
    def destroy_node(self):
        if self.hand_socket is not None:
            self.hand_socket.close()
        if self.raw_video_writer is not None:
            self.raw_video_writer.release()
        if self.video_writer is not None:
            self.video_writer.release()
        cv2.destroyAllWindows()
        self.pose_detector.close()
        self.hands_detector.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    parsed_args = parse_args(rclpy.utilities.remove_ros_args(args))
    node = FaceHandNode(parsed_args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
