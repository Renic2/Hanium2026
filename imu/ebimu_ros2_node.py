#!/usr/bin/env python3
"""Publish enabled EBIMU-9DOFV6 serial streams as ROS 2 sensor_msgs/Imu."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import serial
from sensor_msgs.msg import Imu


GRAVITY_MPS2 = 9.80665
DEG_TO_RAD = math.pi / 180.0
EXPECTED_FIELD_COUNT = 10
MAX_BUFFER_BYTES = 4096


@dataclass(frozen=True)
class Sample:
    # The EBIMU V6 ASCII quaternion field order is Z, Y, X, W.
    quaternion_xyzw: Tuple[float, float, float, float]
    angular_velocity_dps: Tuple[float, float, float]
    linear_acceleration_g: Tuple[float, float, float]


def parse_packet(packet: bytes) -> Optional[Sample]:
    """Parse *qz,qy,qx,qw,gx,gy,gz,ax,ay,az without accepting partial data."""
    try:
        text = packet.decode("ascii").strip()
    except UnicodeDecodeError:
        return None

    if not text.startswith("*"):
        return None

    fields = text[1:].split(",")
    if len(fields) != EXPECTED_FIELD_COUNT:
        return None

    try:
        values = tuple(float(field) for field in fields)
    except ValueError:
        return None

    if not all(math.isfinite(value) for value in values):
        return None

    qz, qy, qx, qw, gx, gy, gz, ax, ay, az = values
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1.0e-6:
        return None

    return Sample(
        quaternion_xyzw=(qx / norm, qy / norm, qz / norm, qw / norm),
        angular_velocity_dps=(gx, gy, gz),
        linear_acceleration_g=(ax, ay, az),
    )


class EbimuWorker:
    DESIRED_SETTINGS: Sequence[Tuple[bytes, bytes]] = (
        (b"soc:1", b"<soc1>"),    # ASCII output
        (b"sof:2", b"<sof2>"),    # quaternion orientation
        (b"sog:1", b"<sog1>"),    # angular velocity, deg/s
        (b"soa:1", b"<soa1>"),    # raw acceleration including gravity, g
        (b"sor:10", b"<sor10>"),  # 10 ms period (100 Hz)
    )

    def __init__(
        self,
        node: "EbimuNode",
        side: str,
        port: str,
        frame_id: str,
        baudrate: int,
        stale_timeout_sec: float,
        configure_device: bool,
    ) -> None:
        self.node = node
        self.side = side
        self.port = port
        self.frame_id = frame_id
        self.baudrate = baudrate
        self.stale_timeout_sec = stale_timeout_sec
        self.configure_device = configure_device
        self.raw_publisher = node.create_publisher(
            Imu, f"/imu/{side}/data_raw", qos_profile_sensor_data
        )
        self.data_publisher = node.create_publisher(
            Imu, f"/imu/{side}/data", qos_profile_sensor_data
        )
        self._stop = threading.Event()
        self._serial_lock = threading.Lock()
        self._serial: Optional[serial.Serial] = None
        self._last_log_time: Dict[str, float] = {}
        self._valid_count = 0
        self._invalid_count = 0
        self._has_announced_stream = False
        self._thread = threading.Thread(
            target=self._run,
            name=f"ebimu-{side}",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._serial_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except serial.SerialException:
                    pass
        self._thread.join(timeout=3.0)

    def _log_throttled(self, key: str, message: str, interval: float = 15.0) -> None:
        now = time.monotonic()
        if now - self._last_log_time.get(key, 0.0) >= interval:
            self.node.get_logger().warning(message)
            self._last_log_time[key] = now

    def _send_command(
        self, device: serial.Serial, command: bytes, expected: bytes
    ) -> bytes:
        device.reset_input_buffer()
        device.write(command)
        device.flush()
        response = device.read_until(b">", size=1024)
        if expected not in response:
            raise TimeoutError(
                f"no valid response to {command.decode('ascii', errors='replace')}"
            )
        # Commands update internal flash. The V6 hardware can drop the next
        # command when they are sent back-to-back, even after returning <ok>.
        time.sleep(1.0)
        return response

    def _start_stream(self, device: serial.Serial) -> Tuple[Sample, bytearray]:
        for _ in range(2):
            device.reset_input_buffer()
            device.write(b"<start>")
            device.flush()
            response = device.read_until(b">", size=64)
            if b"<ok>" not in response:
                continue

            deadline = time.monotonic() + self.stale_timeout_sec
            buffer = bytearray()
            while time.monotonic() < deadline and not self._stop.is_set():
                buffer.extend(device.read(256))
                while True:
                    end = buffer.find(b"\r\n")
                    if end < 0:
                        break
                    packet = bytes(buffer[:end])
                    del buffer[: end + 2]
                    sample = parse_packet(packet)
                    if sample is not None:
                        return sample, buffer
            time.sleep(1.0)
        raise TimeoutError("<start> acknowledged but no complete EBIMU frame arrived")

    def _configure(self, device: serial.Serial) -> bytearray:
        self._send_command(device, b"<stop>", b"<ok>")
        configuration = self._send_command(device, b"<cfg>", b"sor:")
        for marker, command in self.DESIRED_SETTINGS:
            if marker not in configuration:
                self._send_command(device, command, b"<ok>")
        # A configuration query is also handled by the device's flash-backed
        # command path. Give it one additional interval before restarting.
        time.sleep(1.0)
        sample, remaining_buffer = self._start_stream(device)
        self._publish(sample)
        return remaining_buffer

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                with serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=1.0,
                    write_timeout=1.0,
                ) as device:
                    with self._serial_lock:
                        self._serial = device
                    initial_buffer = bytearray()
                    if self.configure_device:
                        initial_buffer = self._configure(device)
                    self._read_loop(device, initial_buffer)
            except (OSError, serial.SerialException, TimeoutError) as exc:
                self._log_throttled(
                    "connection",
                    f"{self.side}: {self.port} unavailable ({exc}); retrying",
                )
            finally:
                with self._serial_lock:
                    self._serial = None

            self._stop.wait(1.0)

    def _read_loop(self, device: serial.Serial, buffer: bytearray) -> None:
        last_bytes_at = time.monotonic()

        while not self._stop.is_set():
            chunk = device.read(256)
            now = time.monotonic()
            if chunk:
                buffer.extend(chunk)
                last_bytes_at = now
            elif now - last_bytes_at > self.stale_timeout_sec:
                raise TimeoutError(f"no serial data for {self.stale_timeout_sec:.1f} s")

            while True:
                end = buffer.find(b"\r\n")
                if end < 0:
                    break
                packet = bytes(buffer[:end])
                del buffer[: end + 2]
                sample = parse_packet(packet)
                if sample is None:
                    self._invalid_count += 1
                    self._log_throttled(
                        "packet",
                        f"{self.side}: dropped malformed EBIMU packet "
                        f"(valid={self._valid_count}, invalid={self._invalid_count})",
                    )
                    continue
                self._valid_count += 1
                self._publish(sample)

            if len(buffer) > MAX_BUFFER_BYTES:
                # Preserve only the bytes after the newest possible frame start.
                newest_start = buffer.rfind(b"*")
                if newest_start >= 0:
                    del buffer[:newest_start]
                else:
                    buffer.clear()
                self._log_throttled(
                    "overflow", f"{self.side}: discarded overlong serial fragment"
                )

    def _publish(self, sample: Sample) -> None:
        if not self._has_announced_stream:
            self.node.get_logger().info(
                f"{self.side}: receiving valid EBIMU frames from {self.port} "
                f"at {self.baudrate} baud"
            )
            self._has_announced_stream = True
        stamp = self.node.get_clock().now().to_msg()
        raw_message = Imu()
        data_message = Imu()
        for message in (raw_message, data_message):
            message.header.stamp = stamp
            message.header.frame_id = self.frame_id
            message.angular_velocity.x = sample.angular_velocity_dps[0] * DEG_TO_RAD
            message.angular_velocity.y = sample.angular_velocity_dps[1] * DEG_TO_RAD
            message.angular_velocity.z = sample.angular_velocity_dps[2] * DEG_TO_RAD
            message.linear_acceleration.x = sample.linear_acceleration_g[0] * GRAVITY_MPS2
            message.linear_acceleration.y = sample.linear_acceleration_g[1] * GRAVITY_MPS2
            message.linear_acceleration.z = sample.linear_acceleration_g[2] * GRAVITY_MPS2

        # data_raw deliberately declares orientation unavailable.
        raw_message.orientation_covariance[0] = -1.0

        qx, qy, qz, qw = sample.quaternion_xyzw
        data_message.orientation.x = qx
        data_message.orientation.y = qy
        data_message.orientation.z = qz
        data_message.orientation.w = qw

        self.raw_publisher.publish(raw_message)
        self.data_publisher.publish(data_message)


class EbimuNode(Node):
    def __init__(self) -> None:
        super().__init__("ebimu")
        self.declare_parameter("left.enabled", True)
        self.declare_parameter("left.port", "/dev/imu_left")
        self.declare_parameter("left.frame_id", "imu_left_link")
        self.declare_parameter("right.enabled", True)
        self.declare_parameter("right.port", "/dev/imu_right")
        self.declare_parameter("right.frame_id", "imu_right_link")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("stale_timeout_sec", 2.0)
        self.declare_parameter("configure_device", True)

        baudrate = self.get_parameter("baudrate").value
        stale_timeout_sec = self.get_parameter("stale_timeout_sec").value
        configure_device = self.get_parameter("configure_device").value
        self.workers = []
        enabled_sides = []
        for side in ("left", "right"):
            if not self.get_parameter(f"{side}.enabled").value:
                self.get_logger().info(f"{side}: disabled by configuration")
                continue
            worker = EbimuWorker(
                node=self,
                side=side,
                port=self.get_parameter(f"{side}.port").value,
                frame_id=self.get_parameter(f"{side}.frame_id").value,
                baudrate=baudrate,
                stale_timeout_sec=stale_timeout_sec,
                configure_device=configure_device,
            )
            self.workers.append(worker)
            worker.start()
            enabled_sides.append(side)

        self.get_logger().info(
            f"EBIMU publisher started for {', '.join(enabled_sides)}; sensor axes "
            "are published in each sensor frame without an assumed mounting rotation"
        )

    def close(self) -> None:
        for worker in self.workers:
            worker.stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EbimuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
