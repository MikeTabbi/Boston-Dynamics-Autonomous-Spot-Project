#!/usr/bin/env python3
"""
tf_keep_alive.py — Keeps spot/frontleft->spot/frontleft_fisheye TF fresh.

The register_node looks up TF at Spot's hardware clock timestamps, which are
ahead of the container clock. Static TF entries stored at container-clock time
appear "in the past" to lookups, causing extrapolation errors after ~seconds.

Fix: track Spot's clock from incoming depth images, then publish the (constant)
camera transform at Spot's current clock time at 10 Hz, plus a 0.5s lookahead
window so the buffer is always populated for the next image.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Image

# spot/frontleft -> spot/frontleft_fisheye (from tf2_echo at driver startup)
TX = 0.07666037999999999
TY = -0.0027737399999999994
TZ = 0.00100019
RX = -0.004046150017636412
RY =  9.483000041334638e-05
RZ = -0.004172610018187629
RW =  0.9999831043587396


class TFKeepAlive(Node):
    def __init__(self):
        super().__init__('tf_keep_alive')
        self.br = TransformBroadcaster(self)
        self.last_spot_ns = None

        self.create_subscription(
            Image, '/spot/depth/frontleft/image', self.img_cb, 1)

        # Publish at 10 Hz
        self.create_timer(0.1, self.publish_tf)
        self.get_logger().info('tf_keep_alive started — waiting for first depth image')

    def img_cb(self, msg):
        self.last_spot_ns = (
            msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        )

    def _make_tf(self, sec, nanosec):
        t = TransformStamped()
        t.header.stamp.sec = sec
        t.header.stamp.nanosec = nanosec
        t.header.frame_id = 'spot/frontleft'
        t.child_frame_id = 'spot/frontleft_fisheye'
        t.transform.translation.x = TX
        t.transform.translation.y = TY
        t.transform.translation.z = TZ
        t.transform.rotation.x = RX
        t.transform.rotation.y = RY
        t.transform.rotation.z = RZ
        t.transform.rotation.w = RW
        return t

    def publish_tf(self):
        if self.last_spot_ns is None:
            return

        # Publish at current Spot time and 0.5s into the future (5 steps)
        # so the register_node always finds a valid entry for upcoming images.
        transforms = []
        for dt_ms in range(0, 1600, 100):
            ns = self.last_spot_ns + dt_ms * 10**6
            sec = ns // 10**9
            nanosec = ns % 10**9
            transforms.append(self._make_tf(int(sec), int(nanosec)))

        self.br.sendTransform(transforms)


def main():
    rclpy.init()
    node = TFKeepAlive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
