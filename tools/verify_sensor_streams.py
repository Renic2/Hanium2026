#!/usr/bin/env python3
"""Verify that every sensor stream is carrying changing ROS 2 messages."""

import argparse
import json
import math
import time
import zlib

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan


REQUIRED_STREAMS = {
    "/scan": LaserScan,
    "/image_left_raw": Image,
    "/image_right_raw": Image,
    "/image_combine_raw": Image,
    "/imu/left/data_raw": Imu,
    "/imu/left/data": Imu,
    "/StereoNetNode/stereonet_depth": Image,
    "/map": OccupancyGrid,
}

RIGHT_IMU_STREAMS = {
    "/imu/right/data_raw": Imu,
    "/imu/right/data": Imu,
}


class StreamVerifier(Node):
    def __init__(self, streams):
        super().__init__("sensor_stream_verifier")
        self.results = {
            topic: {
                "count": 0,
                "first_rx": None,
                "last_rx": None,
                "first_crc32": None,
                "last_crc32": None,
                "detail": None,
            }
            for topic in streams
        }
        self.subscriber_handles = []
        for topic, message_type in streams.items():
            self.subscriber_handles.append(
                self.create_subscription(
                    message_type,
                    topic,
                    lambda message, name=topic: self.record(name, message),
                    qos_profile_sensor_data,
                )
            )

    def record(self, topic, message):
        now = time.monotonic()
        result = self.results[topic]
        result["count"] += 1
        result["first_rx"] = result["first_rx"] or now
        result["last_rx"] = now

        if isinstance(message, Image):
            checksum = f"{zlib.crc32(message.data):08x}"
            result["first_crc32"] = result["first_crc32"] or checksum
            result["last_crc32"] = checksum
            detail = {
                "frame_id": message.header.frame_id,
                "width": message.width,
                "height": message.height,
                "encoding": message.encoding,
                "bytes": len(message.data),
            }
            if message.encoding == "nv12":
                luma_size = message.width * message.height
                luma_sample = message.data[:luma_size:257]
                if luma_sample:
                    detail.update(
                        {
                            "luma_sample_min": min(luma_sample),
                            "luma_sample_max": max(luma_sample),
                            "luma_sample_mean": round(
                                sum(luma_sample) / len(luma_sample), 2
                            ),
                        }
                    )
            elif message.encoding == "mono16" and not message.is_bigendian:
                pixels = memoryview(message.data).cast("H")
                depth_sample = [pixels[index] for index in range(0, len(pixels), 257)]
                if depth_sample:
                    valid = [value for value in depth_sample if 0 < value < 65535]
                    detail.update(
                        {
                            "depth_sample_valid": len(valid),
                            "depth_sample_zero_invalid": depth_sample.count(0),
                            "depth_sample_saturated_invalid": depth_sample.count(65535),
                            "depth_sample_min_mm": min(valid) if valid else None,
                            "depth_sample_max_mm": max(valid) if valid else None,
                            "depth_sample_mean_mm": round(sum(valid) / len(valid), 2)
                            if valid
                            else None,
                        }
                    )
            result["detail"] = detail
        elif isinstance(message, Imu):
            q = message.orientation
            a = message.linear_acceleration
            result["detail"] = {
                "frame_id": message.header.frame_id,
                "orientation_covariance_0": message.orientation_covariance[0],
                "orientation_norm": round(
                    math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w),
                    6,
                ),
                "acceleration_norm_mps2": round(
                    math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z), 6
                ),
            }
        elif isinstance(message, LaserScan):
            finite_ranges = [value for value in message.ranges if math.isfinite(value)]
            result["detail"] = {
                "frame_id": message.header.frame_id,
                "ranges": len(message.ranges),
                "finite_ranges": len(finite_ranges),
                "min_range_m": round(min(finite_ranges), 4) if finite_ranges else None,
                "max_range_m": round(max(finite_ranges), 4) if finite_ranges else None,
            }
        elif isinstance(message, OccupancyGrid):
            known_cells = sum(value >= 0 for value in message.data)
            occupied_cells = sum(value >= 50 for value in message.data)
            result["detail"] = {
                "frame_id": message.header.frame_id,
                "resolution_m": round(message.info.resolution, 4),
                "width": message.info.width,
                "height": message.info.height,
                "cells": len(message.data),
                "known_cells": known_cells,
                "occupied_cells": occupied_cells,
            }

    def report(self):
        output = {}
        for topic, result in self.results.items():
            elapsed = (
                result["last_rx"] - result["first_rx"]
                if result["count"] >= 2
                else 0.0
            )
            output[topic] = {
                "count": result["count"],
                "rate_hz": round((result["count"] - 1) / elapsed, 2)
                if elapsed > 0.0
                else 0.0,
                "payload_changed": (
                    result["first_crc32"] != result["last_crc32"]
                    if result["first_crc32"] is not None
                    else None
                ),
                "detail": result["detail"],
            }
        return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=12.0)
    parser.add_argument(
        "--require-right-imu",
        action="store_true",
        help="also require the optional UART3/right IMU streams",
    )
    args = parser.parse_args()

    streams = dict(REQUIRED_STREAMS)
    if args.require_right_imu:
        streams.update(RIGHT_IMU_STREAMS)

    rclpy.init()
    node = StreamVerifier(streams)
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        report = node.report()
        print(json.dumps(report, indent=2, sort_keys=True))
        missing = [topic for topic, result in report.items() if result["count"] == 0]
        raise SystemExit(1 if missing else 0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
