#!/usr/bin/env python3
# frontier_explore_hybrid.py
# Autonomous frontier exploration for Boston Dynamics Spot.
# Core idea: find the edges between known-free space and unknown space (frontiers),
# pick the best one to drive toward, repeat until the whole room is mapped.

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
from tf2_ros import Buffer, TransformListener


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class HybridFrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer_hybrid')

        # --- Tunable parameters ---
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('costmap_topic', '/global_costmap/costmap_raw')
        self.declare_parameter('robot_frame', 'spot/body')
        self.declare_parameter('global_frame', 'map')

        self.declare_parameter('min_frontier_cluster', 10)
        self.declare_parameter('goal_clearance_cells', 10)

        self.declare_parameter('min_goal_distance', 0.75)
        self.declare_parameter('max_goal_distance', 3.5)

        self.declare_parameter('cost_threshold', 50)
        self.declare_parameter('reject_unknown_costmap', False)

        # Debug / behavior toggles
        self.declare_parameter('enable_initial_spin', False)   # disabled for debugging
        self.declare_parameter('enable_prearming', False)      # disabled for debugging
        self.declare_parameter('pre_arm_distance', 1.5)

        # --- Read parameters ---
        self.map_topic = self.get_parameter('map_topic').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.global_frame = self.get_parameter('global_frame').value

        self.min_cluster = int(self.get_parameter('min_frontier_cluster').value)
        self.clear_cells = int(self.get_parameter('goal_clearance_cells').value)

        self.min_goal_dist = float(self.get_parameter('min_goal_distance').value)
        self.max_goal_dist = float(self.get_parameter('max_goal_distance').value)

        self.cost_thresh = int(self.get_parameter('cost_threshold').value)
        self.reject_unknown_costmap = bool(self.get_parameter('reject_unknown_costmap').value)

        self.enable_initial_spin = bool(self.get_parameter('enable_initial_spin').value)
        self.enable_prearming = bool(self.get_parameter('enable_prearming').value)
        self.pre_arm_dist = float(self.get_parameter('pre_arm_distance').value)

        # --- Latest messages ---
        self.map_msg: OccupancyGrid | None = None
        self.cm_msg: Costmap | None = None

        # --- Subscribers ---
        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, 10)
        self.create_subscription(Costmap, self.costmap_topic, self.on_costmap, 10)

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Action clients ---
        self.ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin_ac = ActionClient(self, Spin, '/spin')

        # --- Costmap clear services ---
        self._clear_global = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap'
        )
        self._clear_local = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap'
        )

        # --- Internal state ---
        self.goal_in_flight = False
        self.blacklist = set()
        self.last_goal = None
        self.next_goal = None
        self._last_goal_angle = None
        self._seen_nonzero_dist = False

        # --- Startup spin flags ---
        # If startup spin is disabled, mark it done immediately.
        self._initial_spin_done = not self.enable_initial_spin
        self._spin_in_flight = False

        # --- Main loop ---
        self.timer = self.create_timer(1.0, self.tick)

        if self.enable_initial_spin:
            self.get_logger().info(
                "HybridFrontierExplorer ready — will spin 360° on startup to seed the map"
            )
        else:
            self.get_logger().info(
                "HybridFrontierExplorer ready — startup spin disabled for debugging"
            )

        if self.enable_prearming:
            self.get_logger().info(
                f"Goal pre-arming enabled (pre_arm_distance={self.pre_arm_dist:.2f} m)"
            )
        else:
            self.get_logger().info("Goal pre-arming disabled for debugging")

    def on_map(self, msg):
        self.map_msg = msg

    def on_costmap(self, msg):
        self.cm_msg = msg

    def robot_xy_yaw(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return x, y, yaw
        except Exception as e:
            self.get_logger().warn(
                f"TF lookup failed {self.global_frame}->{self.robot_frame}: {e}"
            )
            return None

    def _do_initial_spin(self):
        """
        Rotate Spot 360° in place before exploration begins.
        Disabled by default in this debug build because startup rotation was
        amplifying scan/TF timing issues and creating shaky maps.
        """
        if self._spin_in_flight:
            return

        if not self.spin_ac.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for /spin action server...")
            return

        self._spin_in_flight = True
        self.goal_in_flight = True
        self.get_logger().info("Starting initial 360° spin to seed SLAM map...")
        spin_goal = Spin.Goal()
        spin_goal.target_yaw = 2.0 * math.pi
        fut = self.spin_ac.send_goal_async(spin_goal)
        fut.add_done_callback(self._spin_goal_resp)

    def _spin_goal_resp(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn("Spin goal rejected — skipping initial spin.")
            self._initial_spin_done = True
            self._spin_in_flight = False
            self.goal_in_flight = False
            return

        gh.get_result_async().add_done_callback(self._spin_result)

    def _spin_result(self, fut):
        self.get_logger().info("Initial spin complete — starting frontier exploration.")
        self._initial_spin_done = True
        self._spin_in_flight = False
        self.goal_in_flight = False

    def tick(self):
        if self.goal_in_flight:
            return

        if self.map_msg is None:
            self.get_logger().info("Waiting for /map...")
            return

        if self.cm_msg is None:
            self.get_logger().info("Waiting for global costmap...")
            return

        if self.robot_xy_yaw() is None:
            return

        if not self._initial_spin_done:
            self._do_initial_spin()
            return

        pose = self.robot_xy_yaw()
        if pose is None:
            return

        rx, ry, _ = pose
        goal = self.pick_goal(rx, ry)

        if goal is None:
            self.get_logger().info("No usable frontiers (yet).")
            return

        gx, gy = goal
        goal_yaw = math.atan2(gy - ry, gx - rx)
        self.send_goal(gx, gy, goal_yaw)

    def cost_at(self, x, y):
        cm = self.cm_msg
        meta = cm.metadata
        res = meta.resolution
        ox = meta.origin.position.x
        oy = meta.origin.position.y
        w = meta.size_x
        h = meta.size_y

        c = int((x - ox) / res)
        r = int((y - oy) / res)

        if r < 0 or r >= h or c < 0 or c >= w:
            return None

        data = np.array(cm.data, dtype=np.uint8)
        return int(data[r * w + c])

    def pick_goal(self, rx, ry):
        m = self.map_msg
        w, h = m.info.width, m.info.height
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y

        grid = np.array(m.data, dtype=np.int16).reshape((h, w))
        unknown = (grid == -1)
        free = (grid == 0)
        occ = (grid > 50)

        frontier = np.zeros_like(free, dtype=bool)
        frontier[:-1, :] |= free[:-1, :] & unknown[1:, :]
        frontier[1:, :] |= free[1:, :] & unknown[:-1, :]
        frontier[:, :-1] |= free[:, :-1] & unknown[:, 1:]
        frontier[:, 1:] |= free[:, 1:] & unknown[:, :-1]

        idx = np.argwhere(frontier)
        if idx.size == 0:
            return None

        visited = np.zeros(frontier.shape, dtype=bool)

        def neigh(r, c):
            for dr, dc in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w:
                    yield rr, cc

        best = None
        best_score = float('-inf')

        for r, c in idx:
            if visited[r, c]:
                continue

            q = [(r, c)]
            visited[r, c] = True
            cluster = []

            while q:
                rr, cc = q.pop()
                cluster.append((rr, cc))
                for nr, nc in neigh(rr, cc):
                    if frontier[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        q.append((nr, nc))

            if len(cluster) < self.min_cluster:
                continue

            cr = int(sum(p[0] for p in cluster) / len(cluster))
            cc = int(sum(p[1] for p in cluster) / len(cluster))

            goal_rc = None
            for rad in range(self.clear_cells, self.clear_cells + 25):
                for dr in range(-rad, rad + 1):
                    for dc in range(-rad, rad + 1):
                        rr, xx = cr + dr, cc + dc
                        if 0 <= rr < h and 0 <= xx < w and free[rr, xx] and not frontier[rr, xx]:
                            r0, r1 = max(0, rr - 1), min(h, rr + 2)
                            c0, c1 = max(0, xx - 1), min(w, xx + 2)
                            if occ[r0:r1, c0:c1].any():
                                continue
                            goal_rc = (rr, xx)
                            break
                    if goal_rc:
                        break
                if goal_rc:
                    break

            if goal_rc is None:
                continue

            gr, gc = goal_rc
            gx = ox + (gc + 0.5) * res
            gy = oy + (gr + 0.5) * res
            dist = math.hypot(gx - rx, gy - ry)

            if dist < self.min_goal_dist:
                continue

            if dist > self.max_goal_dist:
                continue

            if self.last_goal is not None:
                lgx, lgy = self.last_goal
                if math.hypot(gx - lgx, gy - lgy) < 0.5:
                    continue

            key = (round(gx, 2), round(gy, 2))
            if key in self.blacklist:
                continue

            cval = self.cost_at(gx, gy)
            if cval is None:
                continue
            if self.reject_unknown_costmap and cval == 255:
                continue
            if cval >= self.cost_thresh and cval != 255:
                continue

            score = 0.6 * (dist / self.max_goal_dist) + 0.4 * min(len(cluster) / 200.0, 1.0)
            if score > best_score:
                best_score = score
                best = (gx, gy)

        return best

    def send_goal(self, gx, gy, yaw):
        if not self.ac.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("NavigateToPose server not ready.")
            return

        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.global_frame
        ps.pose.position.x = float(gx)
        ps.pose.position.y = float(gy)
        ps.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(yaw)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        goal.pose = ps

        self.goal_in_flight = True
        self.next_goal = None
        self.last_goal = (gx, gy)
        self._last_goal_angle = yaw
        self._seen_nonzero_dist = False

        self.get_logger().info(f"Sending goal: x={gx:.3f} y={gy:.3f}")
        fut = self.ac.send_goal_async(goal, feedback_callback=self._on_feedback)
        fut.add_done_callback(self._goal_resp)

    def _on_feedback(self, feedback_msg):
        # Disabled by default for debugging so goals do not chain too aggressively.
        if not self.enable_prearming:
            return

        dist_remaining = feedback_msg.feedback.distance_remaining
        if dist_remaining > 0.0:
            self._seen_nonzero_dist = True

        if (
            self._seen_nonzero_dist
            and dist_remaining < self.pre_arm_dist
            and self.next_goal is None
            and self.map_msg is not None
        ):
            pose = self.robot_xy_yaw()
            if pose is None:
                return

            rx, ry, _ = pose
            nxt = self.pick_goal(rx, ry)
            if nxt is not None:
                ngx, ngy = nxt
                nyaw = math.atan2(ngy - ry, ngx - rx)
                self.next_goal = (ngx, ngy, nyaw)
                self.get_logger().info(
                    f"Pre-armed next goal: x={ngx:.3f} y={ngy:.3f} "
                    f"(dist_remaining={dist_remaining:.2f}m)"
                )

    def _goal_resp(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn("Goal rejected by Nav2.")
            self.goal_in_flight = False
            return

        self.get_logger().info("Goal accepted.")
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self._result)

    def _clear_costmaps(self):
        req = ClearEntireCostmap.Request()
        if self._clear_global.service_is_ready():
            self._clear_global.call_async(req)
        if self._clear_local.service_is_ready():
            self._clear_local.call_async(req)
        self.get_logger().info("Costmaps cleared.")

    def _result(self, fut):
        status = fut.result().status
        self.get_logger().info(f"Goal finished status={status}")

        if self.last_goal is not None:
            gx, gy = self.last_goal
            key = (round(gx, 2), round(gy, 2))

            if status in (4, 5, 6):
                self.get_logger().warn(f"Goal {key} failed with status {status}, but not blacklisting in debug mode")
            else:
                self.get_logger().info(f"Goal {key} completed successfully")

        self._clear_costmaps()
        self.goal_in_flight = False

        # Only chain immediately if pre-arming is enabled.
        if self.enable_prearming and self.next_goal is not None:
            ngx, ngy, nyaw = self.next_goal
            self.next_goal = None
            self.send_goal(ngx, ngy, nyaw)


def main():
    rclpy.init()
    node = HybridFrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()