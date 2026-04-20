#!/usr/bin/env python3
import copy, numpy as np, rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

class DepthCropRelay(Node):
    def __init__(self):
        super().__init__("depth_crop_relay")
        self.declare_parameter("row_start", 60)
        self.declare_parameter("row_end",  120)
        self.row_start = int(self.get_parameter("row_start").value)
        self.row_end   = int(self.get_parameter("row_end").value)
        self.img_pub  = self.create_publisher(Image,      "/spot/depth/frontleft/image_cropped",       10)
        self.info_pub = self.create_publisher(CameraInfo, "/spot/depth/frontleft/camera_info_cropped", 10)
        self.create_subscription(Image,      "/spot/depth/frontleft/image",       self._img_cb,  10)
        self.create_subscription(CameraInfo, "/spot/depth/frontleft/camera_info", self._info_cb, 10)
        self._latest_info = None
        self.get_logger().info(f"depth_crop_relay ready -- rows [{self.row_start}, {self.row_end})  band={self.row_end - self.row_start}px")

    def _make_cropped_info(self, msg):
        out = copy.deepcopy(msg)
        band = self.row_end - self.row_start
        out.height = band
        k = list(out.k)
        orig_cy = k[5] if len(k) >= 6 else 0.0
        k[5] = (orig_cy - self.row_start) if orig_cy > 0.0 else (band / 2.0)
        out.k = k
        p = list(out.p)
        if len(p) >= 7:
            orig_pcy = p[6]
            p[6] = (orig_pcy - self.row_start) if orig_pcy > 0.0 else (band / 2.0)
            out.p = p
        return out

    def _info_cb(self, msg):
        out = self._make_cropped_info(msg)
        self._latest_info = out
        self.info_pub.publish(out)

    def _img_cb(self, msg):
        h = msg.height
        r0, r1 = self.row_start, min(self.row_end, h)
        if r1 <= r0:
            self.get_logger().warn(f"crop [{r0},{r1}) invalid for h={h}", throttle_duration_sec=5.0)
            return
        raw2d = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)
        cropped = raw2d[r0:r1, :]
        out = Image()
        out.header = msg.header
        out.height = r1 - r0
        out.width = msg.width
        out.encoding = msg.encoding
        out.is_bigendian = msg.is_bigendian
        out.step = msg.step
        out.data = cropped.tobytes()
        self.img_pub.publish(out)
        if self._latest_info is not None:
            self._latest_info.header = msg.header
            self.info_pub.publish(self._latest_info)

def main():
    rclpy.init()
    node = DepthCropRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
