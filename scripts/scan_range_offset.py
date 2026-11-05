#!/usr/bin/env python3
"""Add a constant to every range of a LaserScan and republish it.

Why this exists (2026-11-02): the front RPLidar S1 on RD started returning every range a constant ~0.33 m
too LONG (box at a taped 0.50 m -> 0.83 m, 1.00 m -> 1.32 m; same in Standard and DenseBoost mode, same
after a power cycle of the unit). The error is additive, not a scale, so it is corrected here, between
the driver (remapped to .../scan_uncorrected) and the filter chain / everything else that reads
/rp_s1_lidar/scan. Re-measure with a box at a taped distance after any change to the unit and set
`offset` (negative shortens) in rplidar_s1_launch.py; set it to 0.0 for a healthy unit.

Ranges that would fall below range_min after the correction are clamped to range_min (something IS that
close) rather than dropped; invalid ranges (inf / nan) pass through untouched.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import numpy as np


class ScanRangeOffset(Node):
    def __init__(self):
        super().__init__("scan_range_offset")
        self.declare_parameter("in_topic", "scan_uncorrected")
        self.declare_parameter("out_topic", "scan")
        self.declare_parameter("offset", 0.0)
        self.offset = float(self.get_parameter("offset").value)
        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value
        # subscribe best-effort (matches the driver's reliable publisher); publish RELIABLE like the driver
        # did, so the filter chain (best effort) and any reliable subscriber both still match
        self.pub = self.create_publisher(LaserScan, out_topic, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST))
        self.sub = self.create_subscription(LaserScan, in_topic, self.cb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST))
        self.add_on_set_parameters_callback(self._on_params)
        self.get_logger().info(f"{in_topic} -> {out_topic}, range offset {self.offset:+.3f} m")

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == "offset":
                self.offset = float(p.value); self.get_logger().info(f"range offset now {self.offset:+.3f} m")
        return SetParametersResult(successful=True)

    def cb(self, m: LaserScan):
        if self.offset:
            r = np.asarray(m.ranges, dtype=np.float32)
            ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
            r[ok] = np.maximum(r[ok] + self.offset, m.range_min)
            m.ranges = r.tolist()
        self.pub.publish(m)


def main():
    rclpy.init()
    node = ScanRangeOffset()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
