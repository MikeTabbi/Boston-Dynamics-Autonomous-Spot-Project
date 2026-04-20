#!/usr/bin/env python3
"""
debug_band_compare.py -- Capture scan data from BOTH the original scan and
the cropped-band scan simultaneously, so you can compare wall detection quality.

Run AFTER starting:
  - depthimage_to_laserscan on /scan (original, full image)
  - depth_crop_relay.py
  - depthimage_to_laserscan on /scan_cropped (cropped band)

Usage:
  python3 /root/debug_band_compare.py --samples 5 --tag baseline
  python3 /root/debug_band_compare.py --samples 5 --tag cropped_60_120

Output: /root/debug_capture/band_compare_<tag>/
  scan_original_N.json    -- range arrays from /scan
  scan_cropped_N.json     -- range arrays from /scan_cropped
  summary.txt             -- stats: valid readings, min/max/median range
"""
import argparse, json, math, os, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class BandCompare(Node):
    def __init__(self, tag, n_samples, out_dir):
        super().__init__("debug_band_compare")
        self.tag = tag
        self.n_samples = n_samples
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)

        self.orig_scans = []
        self.crop_scans = []

        self.create_subscription(LaserScan, "/scan",         self._orig_cb, 10)
        self.create_subscription(LaserScan, "/scan_cropped", self._crop_cb, 10)
        self.get_logger().info(f"BandCompare ready -- collecting {n_samples} samples each, tag={tag}")

    def _save(self, scans, label):
        for i, msg in enumerate(scans):
            ranges = [r if math.isfinite(r) else None for r in msg.ranges]
            valid  = [r for r in msg.ranges if math.isfinite(r)]
            data   = {
                "angle_min": msg.angle_min,
                "angle_max": msg.angle_max,
                "angle_increment": msg.angle_increment,
                "range_min": msg.range_min,
                "range_max": msg.range_max,
                "ranges": ranges,
                "valid_count": len(valid),
                "total_count": len(ranges),
                "median_range": sorted(valid)[len(valid)//2] if valid else None,
                "min_range": min(valid) if valid else None,
                "max_range": max(valid) if valid else None,
            }
            path = os.path.join(self.out_dir, f"{label}_{i+1}.json")
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
        self.get_logger().info(f"Saved {len(scans)} {label} scans -> {self.out_dir}")

    def _summarize(self):
        lines = [f"Band compare -- tag={self.tag}\n"]
        for label, scans in [("original", self.orig_scans), ("cropped",  self.crop_scans)]:
            if not scans:
                lines.append(f"{label}: no data\n")
                continue
            all_valid = []
            for msg in scans:
                all_valid.extend([r for r in msg.ranges if math.isfinite(r)])
            med = sorted(all_valid)[len(all_valid)//2] if all_valid else 0
            lines.append(
                f"{label:10s}  samples={len(scans)}  "
                f"valid_per_scan={len(all_valid)//max(len(scans),1)}  "
                f"median={med:.2f}m  min={min(all_valid):.2f}m  max={max(all_valid):.2f}m\n"
            )
        path = os.path.join(self.out_dir, "summary.txt")
        with open(path, "w") as f:
            f.writelines(lines)
        for l in lines:
            self.get_logger().info(l.strip())

    def _orig_cb(self, msg):
        if len(self.orig_scans) < self.n_samples:
            self.orig_scans.append(msg)
            self.get_logger().info(f"original {len(self.orig_scans)}/{self.n_samples}")
        self._check_done()

    def _crop_cb(self, msg):
        if len(self.crop_scans) < self.n_samples:
            self.crop_scans.append(msg)
            self.get_logger().info(f"cropped  {len(self.crop_scans)}/{self.n_samples}")
        self._check_done()

    def _check_done(self):
        if len(self.orig_scans) >= self.n_samples and len(self.crop_scans) >= self.n_samples:
            self._save(self.orig_scans, "scan_original")
            self._save(self.crop_scans, "scan_cropped")
            self._summarize()
            self.get_logger().info("Done. Shutting down.")
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=5)
    ap.add_argument("--tag",     default="compare")
    args, _ = ap.parse_known_args()

    out_dir = f"/root/debug_capture/band_compare_{args.tag}"
    rclpy.init()
    node = BandCompare(args.tag, args.samples, out_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()

if __name__ == "__main__":
    main()
