#!/usr/bin/env python3
"""
auto_mapper.py -- SLAM-friendly autonomous mapping for Spot.

Motion strategy: drive-pause-wiggle
  1. Drive forward slowly for drive_sec seconds
  2. Stop — let scan settle for pause_sec
  3. Wiggle: rotate +wiggle_deg then back to 0, slowly
     (gives slam_toolbox two viewpoints from the same position)
  4. If obstacle detected: backup -> turn -> repeat
  5. After N cycles without wall contact: larger heading change (sweep_deg)
     to ensure room coverage

This is optimized for SLAM quality, not coverage speed.
Slow movement + stationary scans + small viewpoint changes = good loop closure.

Run with:
  python3 /root/auto_mapper.py --ros-args -p duration_sec:=600.0 ...
"""

import math
import subprocess
import time
from statistics import median

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class AutoMapper(Node):
    def __init__(self):
        super().__init__("auto_mapper")

        # Timing
        self.declare_parameter("duration_sec",   600.0)
        self.declare_parameter("drive_sec",        2.5)  # forward segment length
        self.declare_parameter("pause_sec",        1.0)  # stationary settle time
        self.declare_parameter("backup_sec",       2.0)

        # Speeds
        self.declare_parameter("forward_speed",   0.10)  # slow for clean scans
        self.declare_parameter("backup_speed",    0.10)
        self.declare_parameter("turn_speed",      0.25)

        # Obstacle detection
        self.declare_parameter("obstacle_distance", 0.75)
        self.declare_parameter("forward_cone_deg",  20.0)
        self.declare_parameter("min_obstacle_hits",    5)

        # Wiggle (viewpoint diversity while stationary)
        self.declare_parameter("wiggle_deg",      15.0)  # rotate ±wiggle_deg while paused
        self.declare_parameter("wiggle_speed",    0.20)  # rad/s during wiggle

        # Heading change after N forward cycles without finding wall
        self.declare_parameter("cycles_before_turn",   3)   # turn every N drive cycles
        self.declare_parameter("turn_deg",            90.0)  # heading change amount

        self.declare_parameter("map_save_path", "/root/my_map")

        # Read params
        self.duration      = float(self.get_parameter("duration_sec").value)
        self.drive_sec     = float(self.get_parameter("drive_sec").value)
        self.pause_sec     = float(self.get_parameter("pause_sec").value)
        self.backup_sec    = float(self.get_parameter("backup_sec").value)
        self.fwd_speed     = float(self.get_parameter("forward_speed").value)
        self.backup_speed  = float(self.get_parameter("backup_speed").value)
        self.turn_speed    = float(self.get_parameter("turn_speed").value)
        self.obs_dist      = float(self.get_parameter("obstacle_distance").value)
        self.fwd_cone_deg  = float(self.get_parameter("forward_cone_deg").value)
        self.min_hits      = int(self.get_parameter("min_obstacle_hits").value)
        self.wiggle_deg    = float(self.get_parameter("wiggle_deg").value)
        self.wiggle_speed  = float(self.get_parameter("wiggle_speed").value)
        self.cycles_before_turn = int(self.get_parameter("cycles_before_turn").value)
        self.turn_deg      = float(self.get_parameter("turn_deg").value)
        self.map_path      = str(self.get_parameter("map_save_path").value)

        # Derived
        self.wiggle_sec = math.radians(self.wiggle_deg) / self.wiggle_speed
        self.turn_sec   = math.radians(self.turn_deg)   / self.turn_speed

        # State
        self.cmd_pub = self.create_publisher(Twist, "/spot/cmd_vel", 10)
        self.scan: LaserScan | None = None
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)

        self.start_time    = time.time()
        self.state         = "drive"
        self.state_start   = time.time()
        self._saved        = False
        self._drive_cycles = 0       # counts completed drive segments
        self._wiggle_dir   = 1.0     # +1 = wiggle right first, -1 = left first
        self._turn_dir     = 1.0     # alternates each forced turn

        self.create_timer(0.1, self.tick)

        self.get_logger().info(
            f"AutoMapper ready -- duration={self.duration:.0f}s  "
            f"fwd={self.fwd_speed:.2f}m/s  drive={self.drive_sec:.1f}s  "
            f"pause={self.pause_sec:.1f}s  wiggle=±{self.wiggle_deg:.0f}deg  "
            f"map -> {self.map_path}"
        )

    # ------------------------------------------------------------------ #

    def _scan_cb(self, msg: LaserScan):
        self.scan = msg

    def _cmd(self, vx: float = 0.0, wz: float = 0.0):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)

    def _set_state(self, state: str):
        self.state       = state
        self.state_start = time.time()
        self.get_logger().info(f"State -> {state}")

    def _obstacle_ahead(self) -> bool:
        if self.scan is None:
            return False
        cone  = math.radians(self.fwd_cone_deg)
        hits  = 0
        angle = self.scan.angle_min
        for r in self.scan.ranges:
            if -cone <= angle <= cone and math.isfinite(r) and r < self.obs_dist:
                hits += 1
                if hits >= self.min_hits:
                    return True
            angle += self.scan.angle_increment
        return False

    def _front_median(self) -> float:
        if self.scan is None:
            return math.inf
        cone  = math.radians(self.fwd_cone_deg)
        vals  = []
        angle = self.scan.angle_min
        for r in self.scan.ranges:
            if -cone <= angle <= cone and math.isfinite(r):
                vals.append(r)
            angle += self.scan.angle_increment
        return median(vals) if vals else math.inf

    # ------------------------------------------------------------------ #

    def tick(self):
        elapsed       = time.time() - self.start_time
        state_elapsed = time.time() - self.state_start

        if elapsed >= self.duration and not self._saved:
            self._saved = True
            self._cmd(0.0, 0.0)
            self.get_logger().info(
                f"Mapping complete ({self.duration:.0f}s). Saving map ..."
            )
            self._save_map()
            return

        if self.scan is None:
            self._cmd(0.0, 0.0)
            return

        # ── drive ───────────────────────────────────────────────────────
        if self.state == "drive":
            if self._obstacle_ahead():
                self.get_logger().info(
                    f"Obstacle (front median {self._front_median():.2f}m) -> backup"
                )
                self._set_state("backup")
                return

            if state_elapsed >= self.drive_sec:
                self._drive_cycles += 1
                self._cmd(0.0, 0.0)
                self._set_state("pause")
                return

            self._cmd(vx=self.fwd_speed, wz=0.0)

        # ── pause ── settle before wiggle ────────────────────────────────
        elif self.state == "pause":
            self._cmd(0.0, 0.0)
            if state_elapsed >= self.pause_sec:
                self._set_state("wiggle_out")

        # ── wiggle_out ── rotate +wiggle_deg ─────────────────────────────
        elif self.state == "wiggle_out":
            wz = self.wiggle_speed * self._wiggle_dir
            if state_elapsed < self.wiggle_sec:
                self._cmd(vx=0.0, wz=wz)
            else:
                self._set_state("wiggle_back")

        # ── wiggle_back ── rotate back to original heading ───────────────
        elif self.state == "wiggle_back":
            wz = -self.wiggle_speed * self._wiggle_dir
            if state_elapsed < self.wiggle_sec:
                self._cmd(vx=0.0, wz=wz)
            else:
                self._cmd(0.0, 0.0)
                self._wiggle_dir *= -1.0  # alternate wiggle direction each cycle

                # Every N drive cycles, make a larger heading change
                if self._drive_cycles % self.cycles_before_turn == 0:
                    self._set_state("turn")
                else:
                    self._set_state("drive")

        # ── turn ── larger heading change for coverage ───────────────────
        elif self.state == "turn":
            wz = self.turn_speed * self._turn_dir
            if state_elapsed < self.turn_sec:
                self._cmd(vx=0.0, wz=wz)
            else:
                self._cmd(0.0, 0.0)
                self.get_logger().info("Heading change complete -> drive")
                self._set_state("drive")

        # ── backup ──────────────────────────────────────────────────────
        elif self.state == "backup":
            if state_elapsed < self.backup_sec:
                self._cmd(vx=-self.backup_speed, wz=0.0)
            else:
                self._cmd(0.0, 0.0)
                self._set_state("turn")

        else:
            self.get_logger().warn(f"Unknown state {self.state}, stopping.")
            self._cmd(0.0, 0.0)

    # ------------------------------------------------------------------ #

    def _save_map(self):
        try:
            result = subprocess.run(
                ["ros2", "service", "call",
                 "/slam_toolbox/serialize_map",
                 "slam_toolbox/srv/SerializePoseGraph",
                 f"{{filename: '{self.map_path}'}}"],
                capture_output=True, text=True, timeout=20,
            )
            if result.returncode == 0:
                self.get_logger().info(f"Posegraph serialized: {self.map_path}.posegraph")
            else:
                self.get_logger().warn(f"Posegraph serialize failed: {result.stderr}")
        except Exception as e:
            self.get_logger().warn(f"Posegraph serialize error: {e}")

        try:
            result = subprocess.run(
                ["ros2", "run", "nav2_map_server", "map_saver_cli",
                 "-f", self.map_path],
                capture_output=True, text=True, timeout=20,
            )
            if result.returncode == 0:
                self.get_logger().info(f"Map saved: {self.map_path}.pgm / .yaml")
            else:
                self.get_logger().error(f"map_saver_cli failed: {result.stderr}")
        except Exception as e:
            self.get_logger().error(f"Map save error: {e}")
        finally:
            self._cmd(0.0, 0.0)
            rclpy.shutdown()


def main():
    rclpy.init()
    node = AutoMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._cmd(0.0, 0.0)
        node.get_logger().info("Interrupted.")
    node.destroy_node()


if __name__ == "__main__":
    main()
