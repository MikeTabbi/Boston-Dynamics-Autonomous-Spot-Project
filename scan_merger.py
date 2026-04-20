#!/usr/bin/env python3
"""
scan_merger.py — Merges /scan_fl, /scan_fr, /scan_back into a single /scan.

Main fixes in this version:
1. Use each scan's actual timestamp for TF lookup
2. Only merge scans that are close together in time
3. Stamp the merged scan from source scan time, not node "now"
4. Reduce self-body exclusion radius
5. Add better logging for TF/timing problems
"""

import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class ScanMerger(Node):
    def __init__(self):
        super().__init__("scan_merger")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("target_frame", "spot/body")
        self.declare_parameter("publish_topic", "/scan")
        self.declare_parameter("publish_rate_hz", 10.0)

        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.pi / 360.0)  # 0.5 deg

        self.declare_parameter("range_min", 0.3)
        self.declare_parameter("range_max", 10.0)

        # Reduced from 1.2 -> 0.8 so we do not throw away too much useful nearby structure
        self.declare_parameter("body_exclusion_radius", 0.8)

        # Maximum allowed age difference between scans to merge together
        # 0.10s is a good debugging starting point at 10 Hz publishing
        self.declare_parameter("max_scan_skew_sec", 0.40)

        # Ignore scans older than this relative to the newest scan in the bundle
        self.declare_parameter("max_scan_age_sec", 0.40)

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.publish_topic = str(self.get_parameter("publish_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.angle_increment = float(self.get_parameter("angle_increment").value)

        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.body_exclusion_radius = float(self.get_parameter("body_exclusion_radius").value)

        self.max_scan_skew_sec = float(self.get_parameter("max_scan_skew_sec").value)
        self.max_scan_age_sec = float(self.get_parameter("max_scan_age_sec").value)

        self.num_beams = int(round((self.angle_max - self.angle_min) / self.angle_increment))

        # -----------------------------
        # TF
        # -----------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -----------------------------
        # Latest scans by topic
        # -----------------------------
        self.scans: Dict[str, LaserScan] = {}
        self.topics = ["/scan_fl", "/scan_fr"]

        for topic in self.topics:
            self.create_subscription(
                LaserScan,
                topic,
                lambda msg, t=topic: self._scan_cb(msg, t),
                10,
            )

        self.pub = self.create_publisher(LaserScan, self.publish_topic, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self.merge_and_publish)

        self._last_warn_missing_scans_ns = 0
        self._last_warn_skew_ns = 0
        self._last_warn_tf_ns = 0

        self.get_logger().info(
            f"scan_merger ready: target_frame={self.target_frame}, "
            f"publish_topic={self.publish_topic}, "
            f"body_exclusion_radius={self.body_exclusion_radius:.2f} m, "
            f"max_scan_skew_sec={self.max_scan_skew_sec:.3f}"
        )

    def _scan_cb(self, msg: LaserScan, topic: str):
        self.scans[topic] = msg

    @staticmethod
    def _stamp_to_float_sec(msg: LaserScan) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _warn_throttled(self, key: str, text: str, period_sec: float = 2.0):
        now_ns = self.get_clock().now().nanoseconds
        attr = f"_last_warn_{key}_ns"
        last_ns = getattr(self, attr, 0)
        if (now_ns - last_ns) / 1e9 >= period_sec:
            self.get_logger().warn(text)
            setattr(self, attr, now_ns)

    def _choose_scan_bundle(self) -> List[LaserScan]:
        """
        Select a set of scans that are close enough in timestamp to merge together.
        We prefer using all available recent scans, but reject badly skewed bundles.
        """
        available = [self.scans[t] for t in self.topics if t in self.scans]
        if not available:
            return []

        # If not all scans are present yet, still allow partial merging, but warn
        if len(available) < len(self.topics):
            missing = [t for t in self.topics if t not in self.scans]
            self._warn_throttled(
                "missing_scans",
                f"Still waiting on scans from: {missing}. Publishing partial merged scan for now."
            )

        times = [self._stamp_to_float_sec(s) for s in available]
        newest = max(times)

        # Filter out scans that are too old relative to the newest one
        fresh = [
            s for s in available
            if newest - self._stamp_to_float_sec(s) <= self.max_scan_age_sec
        ]
        if not fresh:
            return []

        fresh_times = [self._stamp_to_float_sec(s) for s in fresh]
        skew = max(fresh_times) - min(fresh_times)

        if skew > self.max_scan_skew_sec:
            self._warn_throttled(
                "skew",
                f"Scan bundle rejected due to timestamp skew {skew:.3f}s "
                f"(limit {self.max_scan_skew_sec:.3f}s)."
            )
            return []

        return fresh

    def _lookup_tf_at_scan_time(self, frame: str, stamp) -> Optional[Tuple[float, float, float]]:
        """
        Return (tx, ty, yaw) for transform: target_frame <- frame at scan timestamp.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                frame,
                Time.from_msg(stamp),
            )
        except Exception as e:
            self._warn_throttled(
                "tf",
                f"TF lookup failed for {frame} -> {self.target_frame} at scan time: {e}"
            )
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return tx, ty, yaw

    def _scan_to_body_points(self, scan: LaserScan) -> List[Tuple[float, float]]:
        frame = scan.header.frame_id
        tf_data = self._lookup_tf_at_scan_time(frame, scan.header.stamp)
        if tf_data is None:
            return []

        tx, ty, yaw = tf_data
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        points: List[Tuple[float, float]] = []

        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)

                bx = tx + lx * cos_y - ly * sin_y
                by = ty + lx * sin_y + ly * cos_y
                points.append((bx, by))

            angle += scan.angle_increment

        return points

    def merge_and_publish(self):
        bundle = self._choose_scan_bundle()
        if not bundle:
            return

        ranges = np.full(self.num_beams, float("inf"), dtype=np.float32)

        for scan in bundle:
            for bx, by in self._scan_to_body_points(scan):
                r = math.hypot(bx, by)

                # Reject self-body returns and out-of-range points
                if r < self.body_exclusion_radius or r < self.range_min or r > self.range_max:
                    continue

                a = math.atan2(by, bx)
                idx = int((a - self.angle_min) / self.angle_increment)

                if 0 <= idx < self.num_beams and r < ranges[idx]:
                    ranges[idx] = r

        # Use the newest source scan timestamp, not "now"
        newest_scan = max(bundle, key=self._stamp_to_float_sec)

        msg = LaserScan()
        msg.header.stamp = newest_scan.header.stamp
        msg.header.frame_id = self.target_frame

        msg.angle_min = self.angle_min
        msg.angle_max = float(self.angle_min + self.num_beams * self.angle_increment)
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.publish_rate_hz

        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges.tolist()

        self.pub.publish(msg)


def main():
        rclpy.init()
        node = ScanMerger()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()