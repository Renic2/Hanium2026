#!/usr/bin/env python3
"""Low-overhead binary WebSocket bridge for dashboard camera/depth images."""

from __future__ import annotations

import json
import math
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import tornado.ioloop
import tornado.web
import tornado.websocket


ALLOWED_TOPICS = (
    "/image_left_raw",
    "/image_right_raw",
    "/StereoNetNode/stereonet_depth",
)
DEFAULT_RATE_HZ = 2.0
MIN_RATE_HZ = 0.5
MAX_RATE_HZ = 4.0


@dataclass(frozen=True)
class ImageFrame:
    sequence: int
    topic: str
    width: int
    height: int
    encoding: str
    is_bigendian: bool
    step: int
    frame_id: str
    stamp_sec: int
    stamp_nanosec: int
    source_hz: float
    data: bytes


class ImageStore(Node):
    def __init__(self) -> None:
        super().__init__("dashboard_image_stream")
        self._lock = threading.Lock()
        self._frames: Dict[str, ImageFrame] = {}
        self._arrival_times: Dict[str, Deque[float]] = {
            topic: deque() for topic in ALLOWED_TOPICS
        }
        self._sequences: Dict[str, int] = {topic: 0 for topic in ALLOWED_TOPICS}
        self._subscriptions = [
            self.create_subscription(
                Image,
                topic,
                lambda message, selected_topic=topic: self._receive(
                    selected_topic, message
                ),
                qos_profile_sensor_data,
            )
            for topic in ALLOWED_TOPICS
        ]

    def _receive(self, topic: str, message: Image) -> None:
        now = time.monotonic()
        with self._lock:
            arrivals = self._arrival_times[topic]
            arrivals.append(now)
            while len(arrivals) > 2 and arrivals[0] < now - 5.0:
                arrivals.popleft()
            source_hz = 0.0
            if len(arrivals) > 1 and arrivals[-1] > arrivals[0]:
                source_hz = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0])
            self._sequences[topic] += 1
            self._frames[topic] = ImageFrame(
                sequence=self._sequences[topic],
                topic=topic,
                width=int(message.width),
                height=int(message.height),
                encoding=str(message.encoding),
                is_bigendian=bool(message.is_bigendian),
                step=int(message.step),
                frame_id=str(message.header.frame_id),
                stamp_sec=int(message.header.stamp.sec),
                stamp_nanosec=int(message.header.stamp.nanosec),
                source_hz=source_hz,
                data=bytes(message.data),
            )

    def latest(self, topic: str) -> Optional[ImageFrame]:
        with self._lock:
            return self._frames.get(topic)


@dataclass
class ClientSubscription:
    topic: str
    period_sec: float
    next_send_at: float
    last_sequence: int = 0


class ImageSocket(tornado.websocket.WebSocketHandler):
    clients = 0

    def initialize(self, store: ImageStore) -> None:
        self.store = store
        self.subscriptions: Dict[str, ClientSubscription] = {}
        self.push_timer: Optional[tornado.ioloop.PeriodicCallback] = None
        self.write_pending = False

    def check_origin(self, origin: str) -> bool:
        hostname = urlparse(origin).hostname
        return hostname in ("localhost", "127.0.0.1", "::1")

    def open(self) -> None:
        self.set_nodelay(True)
        ImageSocket.clients += 1
        self.push_timer = tornado.ioloop.PeriodicCallback(self._push_frames, 25)
        self.push_timer.start()
        self.write_message(
            json.dumps(
                {
                    "op": "ready",
                    "transport": "rdkx5-image-binary-v1",
                    "display_rate_hz": DEFAULT_RATE_HZ,
                }
            )
        )

    def on_message(self, message) -> None:
        if not isinstance(message, str):
            return
        try:
            request = json.loads(message)
        except (TypeError, ValueError):
            return
        subscription_id = str(request.get("id", ""))
        topic = str(request.get("topic", ""))
        if request.get("op") == "unsubscribe":
            self.subscriptions.pop(subscription_id, None)
            return
        if request.get("op") != "subscribe" or topic not in ALLOWED_TOPICS:
            return
        try:
            requested_rate = float(request.get("rate_hz", DEFAULT_RATE_HZ))
        except (TypeError, ValueError):
            requested_rate = DEFAULT_RATE_HZ
        if not math.isfinite(requested_rate):
            requested_rate = DEFAULT_RATE_HZ
        rate_hz = max(MIN_RATE_HZ, min(MAX_RATE_HZ, requested_rate))
        self.subscriptions[subscription_id] = ClientSubscription(
            topic=topic,
            period_sec=1.0 / rate_hz,
            next_send_at=time.monotonic(),
        )

    async def _push_frames(self) -> None:
        if self.write_pending or not self.subscriptions:
            return
        now = time.monotonic()
        for subscription in tuple(self.subscriptions.values()):
            if now < subscription.next_send_at:
                continue
            while subscription.next_send_at <= now:
                subscription.next_send_at += subscription.period_sec
            frame = self.store.latest(subscription.topic)
            if frame is None or frame.sequence == subscription.last_sequence:
                continue
            header = json.dumps(
                {
                    "topic": frame.topic,
                    "sequence": frame.sequence,
                    "width": frame.width,
                    "height": frame.height,
                    "encoding": frame.encoding,
                    "is_bigendian": frame.is_bigendian,
                    "step": frame.step,
                    "frame_id": frame.frame_id,
                    "stamp": {
                        "sec": frame.stamp_sec,
                        "nanosec": frame.stamp_nanosec,
                    },
                    "source_hz": round(frame.source_hz, 3),
                },
                separators=(",", ":"),
            ).encode("utf-8")
            packet = struct.pack("<I", len(header)) + header + frame.data
            subscription.last_sequence = frame.sequence
            self.write_pending = True
            try:
                await self.write_message(packet, binary=True)
            finally:
                self.write_pending = False

    def on_close(self) -> None:
        if self.push_timer is not None:
            self.push_timer.stop()
        self.subscriptions.clear()
        ImageSocket.clients = max(0, ImageSocket.clients - 1)


class HealthHandler(tornado.web.RequestHandler):
    def get(self) -> None:
        self.set_header("Cache-Control", "no-store")
        self.write({"status": "ok", "clients": ImageSocket.clients})


def spin_ros(store: ImageStore) -> None:
    try:
        rclpy.spin(store)
    finally:
        store.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    rclpy.init()
    store = ImageStore()
    ros_thread = threading.Thread(target=spin_ros, args=(store,), daemon=True)
    ros_thread.start()
    application = tornado.web.Application(
        [
            (r"/", ImageSocket, {"store": store}),
            (r"/health", HealthHandler),
        ],
        websocket_ping_interval=10,
        websocket_ping_timeout=5,
        websocket_max_message_size=1024 * 1024,
    )
    application.listen(9093, address="127.0.0.1")
    print(
        "[image-stream] binary camera/depth WebSocket listening on "
        "ws://127.0.0.1:9093",
        flush=True,
    )
    tornado.ioloop.IOLoop.current().start()


if __name__ == "__main__":
    main()
