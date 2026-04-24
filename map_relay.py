#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

class MapRelay(Node):
    def __init__(self):
        super().__init__('map_relay')
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        volatile  = QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE,        reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, '/map', transient)
        self.sub = self.create_subscription(OccupancyGrid, '/rtabmap/grid_prob_map', self.cb, volatile)
        self.get_logger().info('map_relay ready')

    def cb(self, msg):
        self.pub.publish(msg)
        self.get_logger().info(f'map relayed: {msg.info.width}x{msg.info.height}')

def main():
    rclpy.init()
    node = MapRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
