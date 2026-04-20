#!/usr/bin/env python3
"""
auto_mapper.py -- Autonomous mapping behavior for Spot.

State machine:
  forward -> detect obstacle -> backup -> turn_to_acquire_wall -> wall_follow -> sweep -> forward

The sweep state turns Spot away from the wall (into the room) after each
wall-follow segment. This sends Spot diagonally across the room, giving
slam_toolbox cross-room scan data and loop-closure opportunities.

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

        self.declare_parameter("duration_sec", 600.0)
        self.declare_parameter("forward_speed", 0.15)
        self.declare_parameter("wall_follow_speed", 0.12)
        self.declare_parameter("backup_speed", 0.10)
        self.declare_parameter("turn_speed", 0.30)
        self.declare_parameter("max_turn_speed", 0.45)
        self.declare_parameter("obstacle_distance", 0.75)
        self.declare_parameter("forward_cone_deg", 20.0)
        self.declare_parameter("min_obstacle_hits", 5)
        self.declare_parameter("wall_follow_sec", 8.0)
        self.declare_parameter("wall_target_dist", 0.65)
        self.declare_parameter("wall_kp", 1.2)
        self.declare_parameter("side_angle_deg", 75.0)
        self.declare_parameter("side_window_deg", 12.0)
        self.declare_parameter("front_window_deg", 20.0)
        self.declare_parameter("backup_sec", 1.2)
        self.declare_parameter("sweep_deg", 75.0)
        self.declare_parameter("wall_detect_dist", 3.0)
        self.declare_parameter("map_save_path", "/root/my_map")

        self.duration       = float(self.get_parameter("duration_sec").value)
        self.fwd_speed      = float(self.get_parameter("forward_speed").value)
        self.wall_speed     = float(self.get_parameter("wall_follow_speed").value)
        self.backup_speed   = float(self.get_parameter("backup_speed").value)
        self.turn_speed     = float(self.get_parameter("turn_speed").value)
        self.max_turn_speed = float(self.get_parameter("max_turn_speed").value)
        self.obs_dist       = float(self.get_parameter("obstacle_distance").value)
        self.forward_cone_deg  = float(self.get_parameter("forward_cone_deg").value)
        self.min_obstacle_hits = int(self.get_parameter("min_obstacle_hits").value)
        self.wall_follow_sec   = float(self.get_parameter("wall_follow_sec").value)
        self.wall_target    = float(self.get_parameter("wall_target_dist").value)
        self.wall_kp        = float(self.get_parameter("wall_kp").value)
        self.side_angle_deg = float(self.get_parameter("side_angle_deg").value)
        self.side_window_deg = float(self.get_parameter("side_window_deg").value)
        self.front_window_deg = float(self.get_parameter("front_window_deg").value)
        self.backup_sec     = float(self.get_parameter("backup_sec").value)
        self.sweep_deg      = float(self.get_parameter("sweep_deg").value)
        self.wall_detect_dist = float(self.get_parameter("wall_detect_dist").value)
        self.map_path       = str(self.get_parameter("map_save_path").value)

        # sweep_sec: time to rotate sweep_deg at turn_speed
        self.sweep_sec = math.radians(self.sweep_deg) / self.turn_speed

        self.cmd_pub = self.create_publisher(Twist, "/spot/cmd_vel", 10)
        self.scan: LaserScan | None = None
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)

        self.start_time  = time.time()
        self.state       = "forward"
        self.state_start = time.time()
        self.follow_side = -1.0   # +1 = left wall, -1 = right wall
        self._saved      = False

        self.create_timer(0.1, self.tick)

        self.get_logger().info(
            f"AutoMapper ready -- duration={self.duration:.0f}s  "
            f"fwd={self.fwd_speed:.2f}m/s  obs_dist={self.obs_dist:.2f}m  "
            f"wall_follow={self.wall_follow_sec:.0f}s  "
            f"sweep={self.sweep_deg:.0f}deg ({self.sweep_sec:.1f}s)  "
            f"map -> {self.map_path}"
        )

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

    def _ranges_in_window_deg(self, center_deg: float, half_width_deg: float):
        if self.scan is None:
            return []
        center = math.radians(center_deg)
        half   = math.radians(half_width_deg)
        vals   = []
        angle  = self.scan.angle_min
        for r in self.scan.ranges:
            if abs(angle - center) <= half and math.isfinite(r):
                vals.append(r)
            angle += self.scan.angle_increment
        return vals

    def _median_range_deg(self, center_deg: float, half_width_deg: float) -> float:
        vals = self._ranges_in_window_deg(center_deg, half_width_deg)
        return median(vals) if vals else math.inf

    def _front_distance(self) -> float:
        return self._median_range_deg(0.0, self.front_window_deg)

    def _side_distance(self, side: float) -> float:
        center = self.side_angle_deg if side > 0 else -self.side_angle_deg
        return self._median_range_deg(center, self.side_window_deg)

    def _obstacle_ahead(self) -> bool:
        if self.scan is None:
            return False
        cone  = math.radians(self.forward_cone_deg)
        hits  = 0
        angle = self.scan.angle_min
        for r in self.scan.ranges:
            if -cone <= angle <= cone and math.isfinite(r) and r < self.obs_dist:
                hits += 1
                if hits >= self.min_obstacle_hits:
                    return True
            angle += self.scan.angle_increment
        return False

    def tick(self):
        elapsed       = time.time() - self.start_time
        state_elapsed = time.time() - self.state_start

        if elapsed >= self.duration and not self._saved:
            self._saved = True
            self._cmd(0.0, 0.0)
            self.get_logger().info(
                f"Mapping complete ({self.duration:.0f}s). Saving map to {self.map_path} ..."
            )
            self._save_map()
            return

        if self.scan is None:
            self._cmd(0.0, 0.0)
            return

        # ── forward ──────────────────────────────────────────
        if self.state == "forward":
            if self._obstacle_ahead():
                self.get_logger().info(
                    f"Obstacle ahead (front median {self._front_distance():.2f}m) -> backup"
                )
                self._set_state("backup")
            else:
                self._cmd(vx=self.fwd_speed, wz=0.0)

        # ── backup ───────────────────────────────────────────
        elif self.state == "backup":
            if state_elapsed < self.backup_sec:
                self._cmd(vx=-self.backup_speed, wz=0.0)
            else:
                self.follow_side *= -1.0
                self._set_state("turn_to_acquire_wall")

        # ── turn_to_acquire_wall ─────────────────────────────
        elif self.state == "turn_to_acquire_wall":
            side_dist  = self._side_distance(self.follow_side)
            front_dist = self._front_distance()

            if math.isfinite(side_dist) and side_dist < self.wall_detect_dist and front_dist > self.obs_dist:
                self.get_logger().info(
                    f"Wall acquired on {'left' if self.follow_side > 0 else 'right'} "
                    f"(side={side_dist:.2f}m, front={front_dist:.2f}m)"
                )
                self._set_state("wall_follow")
            else:
                wz = self.turn_speed if self.follow_side > 0 else -self.turn_speed
                self._cmd(vx=0.0, wz=wz)
                if state_elapsed > 6.0:
                    self.get_logger().warn(
                        f"Could not acquire wall (side={side_dist:.2f}m limit={self.wall_detect_dist:.1f}m "
                        f"front={front_dist:.2f}m) -> forward"
                    )
                    self._set_state("forward")

        # ── wall_follow ──────────────────────────────────────
        elif self.state == "wall_follow":
            front_dist = self._front_distance()
            side_dist  = self._side_distance(self.follow_side)

            if self._obstacle_ahead():
                self.get_logger().info(
                    f"Wall-follow front blocked (front={front_dist:.2f}m) -> backup"
                )
                self._set_state("backup")
                return

            if state_elapsed >= self.wall_follow_sec:
                self.get_logger().info(
                    f"Wall-follow segment done -> sweep {self.sweep_deg:.0f}deg away from wall"
                )
                self._set_state("sweep")
                return

            if not math.isfinite(side_dist):
                wz = 0.18 if self.follow_side > 0 else -0.18
                self._cmd(vx=self.wall_speed * 0.7, wz=wz)
                return

            error = side_dist - self.wall_target
            wz    = self.wall_kp * error * (1.0 if self.follow_side > 0 else -1.0)
            wz    = max(-self.max_turn_speed, min(self.max_turn_speed, wz))
            speed_scale = 1.0 - min(abs(wz) / max(self.max_turn_speed, 1e-6), 0.6)
            self._cmd(vx=self.wall_speed * speed_scale, wz=wz)

        # ── sweep ────────────────────────────────────────────
        # Turn away from the wall (into the room) by sweep_deg.
        # This changes Spot's heading so the next forward segment crosses
        # the room diagonally, giving slam_toolbox new geometry to match.
        elif self.state == "sweep":
            if state_elapsed < self.sweep_sec:
                # Turn AWAY from follow_side (into the room)
                wz = -self.turn_speed if self.follow_side > 0 else self.turn_speed
                self._cmd(vx=0.0, wz=wz)
            else:
                self.get_logger().info("Sweep complete -> forward")
                self._set_state("forward")

        else:
            self.get_logger().warn(f"Unknown state {self.state}, stopping.")
            self._cmd(0.0, 0.0)

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
                ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", self.map_path],
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
        node.get_logger().info("Interrupted -- stopping Spot.")
    node.destroy_node()


if __name__ == "__main__":
    main()
