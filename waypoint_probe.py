#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

WAYPOINTS = [
    (2.2,  2.4,   0.0),
    (1.7,  2.4,   0.0),
]

def yaw_to_quat(yaw_deg):
    yaw = math.radians(yaw_deg)
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

class WaypointProbe(Node):
    def __init__(self):
        super().__init__("waypoint_probe")
        self.ac = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.idx = 0
        self.results = []
        self.in_flight = False
        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info(f"WaypointProbe ready -- {len(WAYPOINTS)} waypoints queued.")

    def tick(self):
        if self.in_flight:
            return
        if self.idx >= len(WAYPOINTS):
            ok = sum(1 for r in self.results if r == "ok")
            self.get_logger().info(f"All waypoints done. {ok}/{len(WAYPOINTS)} succeeded. Results: {self.results}")
            self.timer.cancel()
            return
        if not self.ac.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Waiting for /navigate_to_pose server...")
            return
        x, y, yaw_deg = WAYPOINTS[self.idx]
        self.send_goal(x, y, yaw_deg)

    def send_goal(self, x, y, yaw_deg):
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw_deg)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        goal.pose = ps
        self.in_flight = True
        self.get_logger().info(f"[{self.idx+1}/{len(WAYPOINTS)}] Goal -> x={x:.2f} y={y:.2f} yaw={yaw_deg:.0f} deg")
        fut = self.ac.send_goal_async(goal)
        fut.add_done_callback(self._goal_resp)

    def _goal_resp(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn(f"Goal {self.idx+1} rejected by Nav2.")
            self.results.append("rejected")
            self.in_flight = False
            self.idx += 1
            return
        gh.get_result_async().add_done_callback(self._result)

    def _result(self, fut):
        status = fut.result().status
        label = "ok" if status == 4 else f"fail(status={status})"
        self.get_logger().info(f"Goal {self.idx+1} finished: {label}")
        self.results.append(label)
        self.in_flight = False
        self.idx += 1

def main():
    rclpy.init()
    node = WaypointProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
