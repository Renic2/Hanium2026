#!/usr/bin/env python3
"""Dependency-free vector and quaternion helpers for EBIMU calibration."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]
Matrix3 = Tuple[
    Tuple[float, float, float],
    Tuple[float, float, float],
    Tuple[float, float, float],
]


def vector_norm(vector: Vector3) -> float:
    return math.sqrt(sum(component * component for component in vector))


def quaternion_normalize(quaternion: Quaternion) -> Quaternion:
    norm = math.sqrt(sum(component * component for component in quaternion))
    if norm < 1.0e-12:
        raise ValueError("cannot normalize a zero quaternion")
    return tuple(component / norm for component in quaternion)  # type: ignore[return-value]


def quaternion_conjugate(quaternion: Quaternion) -> Quaternion:
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    """Rotate vector by an x,y,z,w quaternion without allocating quaternions."""
    qx, qy, qz, qw = quaternion_normalize(quaternion)
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def shortest_arc_quaternion(source: Vector3, target: Vector3) -> Quaternion:
    """Return the minimum rotation that maps source direction onto target."""
    source_norm = vector_norm(source)
    target_norm = vector_norm(target)
    if source_norm < 1.0e-12 or target_norm < 1.0e-12:
        raise ValueError("source and target vectors must be non-zero")

    sx, sy, sz = (component / source_norm for component in source)
    tx, ty, tz = (component / target_norm for component in target)
    dot = max(-1.0, min(1.0, sx * tx + sy * ty + sz * tz))

    if dot > 1.0 - 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    if dot < -1.0 + 1.0e-12:
        # Select a stable axis perpendicular to source for the 180 degree case.
        if abs(sx) <= abs(sy) and abs(sx) <= abs(sz):
            axis = (0.0, -sz, sy)
        elif abs(sy) <= abs(sz):
            axis = (-sz, 0.0, sx)
        else:
            axis = (-sy, sx, 0.0)
        axis_norm = vector_norm(axis)
        return (axis[0] / axis_norm, axis[1] / axis_norm, axis[2] / axis_norm, 0.0)

    cross = (
        sy * tz - sz * ty,
        sz * tx - sx * tz,
        sx * ty - sy * tx,
    )
    return quaternion_normalize((cross[0], cross[1], cross[2], 1.0 + dot))


def yaw_quaternion(yaw_rad: float) -> Quaternion:
    half = yaw_rad * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_to_matrix(quaternion: Quaternion) -> Matrix3:
    x, y, z, w = quaternion_normalize(quaternion)
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def sample_covariance(samples: Sequence[Vector3], mean: Vector3) -> Matrix3:
    denominator = max(1, len(samples) - 1)
    rows = [[0.0] * 3 for _ in range(3)]
    for sample in samples:
        delta = tuple(sample[index] - mean[index] for index in range(3))
        for row in range(3):
            for column in range(3):
                rows[row][column] += delta[row] * delta[column] / denominator
    return tuple(tuple(row) for row in rows)  # type: ignore[return-value]


def rotate_covariance(
    quaternion: Quaternion, covariance: Matrix3, scale: float = 1.0
) -> Matrix3:
    """Compute scale^2 * R * covariance * R^T."""
    rotation = quaternion_to_matrix(quaternion)
    result = [[0.0] * 3 for _ in range(3)]
    for row in range(3):
        for column in range(3):
            result[row][column] = scale * scale * sum(
                rotation[row][inner]
                * covariance[inner][other]
                * rotation[column][other]
                for inner in range(3)
                for other in range(3)
            )
    return tuple(tuple(row) for row in result)  # type: ignore[return-value]


@dataclass(frozen=True)
class CalibrationResult:
    base_from_sensor_xyzw: Quaternion
    gyro_bias_rad_s: Vector3
    accel_scale: float
    accel_mean_mps2: Vector3
    accel_covariance: Matrix3
    gyro_covariance: Matrix3


class StationaryCalibrator:
    """Estimate level mounting rotation and gyro bias from stationary samples."""

    def __init__(
        self,
        sample_count: int,
        gravity_mps2: float,
        yaw_rad: float,
        max_gyro_rad_s: float,
        accel_tolerance_mps2: float,
        max_gyro_stddev: float,
        max_accel_stddev: float,
    ) -> None:
        if sample_count < 2:
            raise ValueError("sample_count must be at least 2")
        self.sample_count = sample_count
        self.gravity_mps2 = gravity_mps2
        self.yaw_rad = yaw_rad
        self.max_gyro_rad_s = max_gyro_rad_s
        self.accel_tolerance_mps2 = accel_tolerance_mps2
        self.max_gyro_stddev = max_gyro_stddev
        self.max_accel_stddev = max_accel_stddev
        self.accel_samples: List[Vector3] = []
        self.gyro_samples: List[Vector3] = []
        self.result: Optional[CalibrationResult] = None

    @property
    def progress(self) -> int:
        return len(self.accel_samples)

    def reset(self) -> None:
        self.accel_samples.clear()
        self.gyro_samples.clear()

    @staticmethod
    def _mean(samples: Sequence[Vector3]) -> Vector3:
        count = len(samples)
        return tuple(
            sum(sample[index] for sample in samples) / count for index in range(3)
        )  # type: ignore[return-value]

    @staticmethod
    def _max_stddev(covariance: Matrix3) -> float:
        return math.sqrt(max(0.0, *(covariance[index][index] for index in range(3))))

    def add_sample(
        self, acceleration_mps2: Vector3, angular_velocity_rad_s: Vector3
    ) -> str:
        """Add one sample and return collecting, rejected, unstable, or complete."""
        if self.result is not None:
            return "complete"

        accel_norm = vector_norm(acceleration_mps2)
        gyro_norm = vector_norm(angular_velocity_rad_s)
        if (
            abs(accel_norm - self.gravity_mps2) > self.accel_tolerance_mps2
            or gyro_norm > self.max_gyro_rad_s
        ):
            self.reset()
            return "rejected"

        self.accel_samples.append(acceleration_mps2)
        self.gyro_samples.append(angular_velocity_rad_s)
        if len(self.accel_samples) < self.sample_count:
            return "collecting"

        accel_mean = self._mean(self.accel_samples)
        gyro_mean = self._mean(self.gyro_samples)
        accel_covariance = sample_covariance(self.accel_samples, accel_mean)
        gyro_covariance = sample_covariance(self.gyro_samples, gyro_mean)
        if (
            self._max_stddev(accel_covariance) > self.max_accel_stddev
            or self._max_stddev(gyro_covariance) > self.max_gyro_stddev
        ):
            self.reset()
            return "unstable"

        alignment = shortest_arc_quaternion(
            accel_mean, (0.0, 0.0, self.gravity_mps2)
        )
        base_from_sensor = quaternion_normalize(
            quaternion_multiply(yaw_quaternion(self.yaw_rad), alignment)
        )
        self.result = CalibrationResult(
            base_from_sensor_xyzw=base_from_sensor,
            gyro_bias_rad_s=gyro_mean,
            accel_scale=self.gravity_mps2 / vector_norm(accel_mean),
            accel_mean_mps2=accel_mean,
            accel_covariance=rotate_covariance(
                base_from_sensor,
                accel_covariance,
                self.gravity_mps2 / vector_norm(accel_mean),
            ),
            gyro_covariance=rotate_covariance(base_from_sensor, gyro_covariance),
        )
        return "complete"
