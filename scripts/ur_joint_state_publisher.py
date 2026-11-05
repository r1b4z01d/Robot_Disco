#!/usr/bin/env python3
"""Publish the UR5e's joint angles as /joint_states so robot_state_publisher moves the arm in the
base robot description (ur_mount.xacro, tf_prefix "ur_") — for Foxglove's 3D panel and TF.

Read-only: it opens an RTDE *receive* connection (ur_rtde) next to whatever is driving the arm
(eval_hw / teleop over RTDE, or the pendant), so it works during a VLA eval, when the ROS UR driver
is not running and must not be. The arm being off or unreachable is not an error: it retries.

While the arm is off or unreachable it still publishes a FALLBACK joint state (param `fallback`:
"home" = the captured home pose from `home_pose_file`, "last" = the last real reading, "zeros",
"none" = publish nothing as before), so the robot description stays whole in Foxglove / the map page
instead of the arm's links dropping out of TF whenever the UR is powered down (2026-10-28).

Params: robot_ip (192.168.11.21), rate_hz (15), tf_prefix ("ur_"), topic ("/joint_states"),
fallback ("home"), home_pose_file (~/rd_ws/VLA/outputs/home_pose.json).
Needs ur_rtde in the SYSTEM python (pip3 install --user ur_rtde; builds from source on arm64).
"""
import json
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

UR_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


class URJointStatePublisher(Node):
    def __init__(self):
        super().__init__("ur_joint_state_publisher")
        self.declare_parameter("robot_ip", "192.168.11.21")
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("tf_prefix", "ur_")
        self.declare_parameter("topic", "/joint_states")
        self.declare_parameter("fallback", "home")
        self.declare_parameter("home_pose_file", os.path.expanduser("~/rd_ws/VLA/outputs/home_pose.json"))
        self.fallback = str(self.get_parameter("fallback").value)
        self.home_pose_file = str(self.get_parameter("home_pose_file").value)
        self._home_q = self._load_home()
        self._last_q = None
        self._live = False
        self.ip = self.get_parameter("robot_ip").value
        self.rate = float(self.get_parameter("rate_hz").value)
        prefix = self.get_parameter("tf_prefix").value
        self.names = [prefix + j for j in UR_JOINTS]
        self.pub = self.create_publisher(JointState, self.get_parameter("topic").value, 10)
        self._rtde = None
        self._next_try = 0.0
        self._warned_import = False
        self.create_timer(1.0 / self.rate, self._tick)
        self.get_logger().info(f"UR joint states from {self.ip} at {self.rate:.0f} Hz as {self.names[0]}..")

    def _load_home(self):
        try:
            q = json.load(open(self.home_pose_file)).get("q")
            if q and len(q) == 6:
                return [float(v) for v in q]
            self.get_logger().warn(f"{self.home_pose_file} has no 6-joint 'q'; fallback uses zeros")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"no home pose ({self.home_pose_file}: {e}); fallback uses zeros")
        return None

    def _fallback_q(self):
        if self.fallback == "none":
            return None
        if self.fallback == "last" and self._last_q is not None:
            return self._last_q
        if self.fallback in ("home", "last") and self._home_q is not None:
            return self._home_q
        return [0.0] * 6

    def _publish(self, q, qd=None) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.names
        msg.position = [float(v) for v in q]
        msg.velocity = [float(v) for v in qd] if qd and len(qd) == 6 else []
        self.pub.publish(msg)

    def _connect(self) -> bool:
        now = time.time()
        if now < self._next_try:
            return False
        # ur_rtde prints "connect: No route to host" itself on every attempt; back off to 30 s while
        # the arm stays off so the base log is not spammed (the fallback joint state keeps publishing)
        self._fails = getattr(self, "_fails", 0)
        self._next_try = now + (30.0 if self._fails >= 3 else 5.0)
        try:
            from rtde_receive import RTDEReceiveInterface
        except ImportError:
            if not self._warned_import:
                self.get_logger().error("ur_rtde is not installed for this python (pip3 install --user ur_rtde); idling")
                self._warned_import = True
            self._next_try = now + 60.0
            return False
        try:
            # low frequency + only the two fields we need: minimal load on the controller next to the
            # client that is actually driving the arm
            self._rtde = RTDEReceiveInterface(self.ip, self.rate, ["actual_q", "actual_qd"])
            self.get_logger().info(f"connected to {self.ip} (RTDE receive)")
            self._fails = 0
            return True
        except Exception as e:  # noqa: BLE001 — arm off, in local mode, or RTDE busy: retry later
            self._fails += 1
            self.get_logger().warn(f"RTDE connect to {self.ip} failed: {e}; retrying", throttle_duration_sec=120.0)
            self._rtde = None
            return False

    def _tick(self):
        if self._rtde is None and not self._connect():
            q = self._fallback_q()
            if q is not None:
                if self._live or not hasattr(self, "_said_fallback"):
                    self._said_fallback = True
                    src = "home pose" if (self.fallback in ("home", "last") and q is self._home_q) else self.fallback
                    self.get_logger().info(f"UR unreachable — publishing the {src} joint state so the arm model stays whole")
                self._live = False
                self._publish(q)
            return
        try:
            if not self._rtde.isConnected():
                raise RuntimeError("RTDE disconnected")
            q = self._rtde.getActualQ()
            qd = self._rtde.getActualQd()
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"RTDE read failed ({e}); reconnecting", throttle_duration_sec=30.0)
            try:
                self._rtde.disconnect()
            except Exception:  # noqa: BLE001
                pass
            self._rtde = None
            return
        if not q or len(q) != 6:
            return
        if not self._live:
            self._live = True
            self.get_logger().info("UR joint states live from RTDE")
        self._last_q = [float(v) for v in q]
        self._publish(q, qd)


def main():
    rclpy.init()
    node = URJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._rtde is not None:
            try:
                node._rtde.disconnect()
            except Exception:  # noqa: BLE001
                pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
