#!/usr/bin/env python3
"""Estimate whether the top half of the combined NV12 image is the left view."""

import argparse
import json
import statistics
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class StereoOrderChecker(Node):
    def __init__(self, target_frames):
        super().__init__("stereo_order_checker")
        self.target_frames = target_frames
        self.frame_results = []
        self.orb = cv2.ORB_create(nfeatures=2000, fastThreshold=10)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.subscription = self.create_subscription(
            Image,
            "/image_combine_raw",
            self.on_image,
            qos_profile_sensor_data,
        )

    def on_image(self, message):
        if len(self.frame_results) >= self.target_frames:
            return
        if message.encoding != "nv12" or message.height % 2:
            return

        width = message.width
        combined_height = message.height
        single_height = combined_height // 2
        expected_bytes = width * combined_height * 3 // 2
        if len(message.data) != expected_bytes:
            return

        raw = np.frombuffer(message.data, dtype=np.uint8)
        y_plane = raw[: width * combined_height].reshape(combined_height, width)
        top = y_plane[:single_height]
        bottom = y_plane[single_height:]

        top_keypoints, top_descriptors = self.orb.detectAndCompute(top, None)
        bottom_keypoints, bottom_descriptors = self.orb.detectAndCompute(bottom, None)
        if top_descriptors is None or bottom_descriptors is None:
            self.frame_results.append({"matches": 0, "disparities": []})
            return

        matches = sorted(
            self.matcher.match(top_descriptors, bottom_descriptors),
            key=lambda match: match.distance,
        )
        disparities = []
        for match in matches[:300]:
            top_point = top_keypoints[match.queryIdx].pt
            bottom_point = bottom_keypoints[match.trainIdx].pt
            if match.distance > 64 or abs(top_point[1] - bottom_point[1]) > 20:
                continue
            disparity = top_point[0] - bottom_point[0]
            if abs(disparity) >= 1.0:
                disparities.append(disparity)

        self.frame_results.append(
            {
                "top_keypoints": len(top_keypoints),
                "bottom_keypoints": len(bottom_keypoints),
                "matches": len(disparities),
                "disparities": disparities,
            }
        )

    def report(self):
        disparities = [
            disparity
            for frame in self.frame_results
            for disparity in frame["disparities"]
        ]
        positive = sum(disparity > 0 for disparity in disparities)
        negative = sum(disparity < 0 for disparity in disparities)
        positive_fraction = positive / len(disparities) if disparities else None
        if positive_fraction is None or len(disparities) < 20:
            interpretation = "inconclusive"
        elif positive_fraction >= 0.65:
            interpretation = "top_is_left"
        elif positive_fraction <= 0.35:
            interpretation = "top_is_right"
        else:
            interpretation = "inconclusive"

        return {
            "frames": len(self.frame_results),
            "usable_matches": len(disparities),
            "positive_disparities": positive,
            "negative_disparities": negative,
            "positive_fraction": round(positive_fraction, 3)
            if positive_fraction is not None
            else None,
            "median_disparity_px": round(statistics.median(disparities), 3)
            if disparities
            else None,
            "interpretation": interpretation,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = StereoOrderChecker(args.frames)
    deadline = time.monotonic() + args.timeout
    try:
        while len(node.frame_results) < args.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        report = node.report()
        print(json.dumps(report, indent=2, sort_keys=True))
        raise SystemExit(0 if report["interpretation"] != "inconclusive" else 1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
