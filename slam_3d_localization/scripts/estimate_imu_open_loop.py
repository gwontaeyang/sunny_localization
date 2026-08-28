#!/usr/bin/env python3
"""Build a bounded, IMU-only relative trajectory from a ROS 2 sqlite bag.

This is a diagnostic reference for wheel-less localization experiments.  It is
not ground truth: position is obtained by double-integrating accelerometer data
without an external velocity or position observation, so it will drift.  The
recorded CV7 convention is corrected with a proper 180 degree roll before
calibration and integration.  Stationary/ZUPT detections are labels only; this
tool deliberately never forces velocity to zero.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import sqlite3
import sys
import tempfile
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable, Deque, Iterable, Iterator, Optional, Sequence, TextIO, Tuple


Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # x, y, z, w
Cov9 = Tuple[float, float, float, float, float, float, float, float, float]

GRAVITY_MPS2 = 9.80665
RX180_QUATERNION: Quat = (1.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ImuSample:
    stamp_ns: int
    record_time_ns: int
    frame_id: str
    angular_velocity_radps: Vec3
    linear_acceleration_mps2: Vec3
    angular_velocity_covariance: Cov9 = (0.0,) * 9
    linear_acceleration_covariance: Cov9 = (0.0,) * 9


@dataclass(frozen=True)
class TrajectoryAnchor:
    position_m: Vec3
    quaternion_xyzw: Quat
    frame_id: str = "map"
    source: str = ""


@dataclass(frozen=True)
class EstimatorConfig:
    calibration_duration_s: float = 2.0
    calibration_min_samples: int = 100
    calibration_max_gyro_mean_radps: float = 0.12
    calibration_max_gyro_rms_radps: float = 0.03
    calibration_min_accel_norm_mps2: float = 7.5
    calibration_max_accel_norm_mps2: float = 12.5
    calibration_max_accel_norm_std_mps2: float = 0.35
    stationary_window_s: float = 0.25
    stationary_max_gyro_rms_radps: float = 0.04
    stationary_max_accel_norm_error_mps2: float = 0.30
    stationary_max_accel_norm_std_mps2: float = 0.20
    max_integration_gap_s: float = 0.10
    max_valid_horizon_s: float = 10.0
    max_position_3sigma_m: float = 2.0
    max_orientation_3sigma_deg: float = 15.0
    fallback_gyro_sigma_radps: float = 0.015
    fallback_accel_sigma_mps2: float = 0.20
    gyro_bias_random_walk_radps_per_sqrt_s: float = 1.0e-4
    accel_bias_random_walk_mps2_per_sqrt_s: float = 2.0e-3


@dataclass(frozen=True)
class CalibrationResult:
    valid: bool
    reason: str
    sample_count: int
    duration_s: float
    gyro_bias_radps: Vec3
    accel_bias_mps2: Vec3
    gyro_bias_covariance: Vec3
    accel_bias_covariance: Vec3
    initial_quaternion_xyzw: Quat
    mean_accel_norm_mps2: float
    accel_norm_std_mps2: float
    gyro_mean_norm_radps: float
    gyro_residual_rms_radps: float


@dataclass
class FilterStats:
    input_count: int = 0
    accepted_count: int = 0
    duplicate_stamp_count: int = 0
    nonmonotonic_stamp_count: int = 0
    invalid_stamp_count: int = 0
    nonfinite_sample_count: int = 0


@dataclass
class RunStats:
    output_count: int = 0
    stationary_candidate_count: int = 0
    zupt_candidate_count: int = 0
    zupt_applied_count: int = 0
    integration_gap_count: int = 0
    first_stamp_ns: Optional[int] = None
    last_stamp_ns: Optional[int] = None
    last_valid_elapsed_s: float = 0.0
    first_invalid_reason: str = ""
    max_position_norm_m: float = 0.0
    max_velocity_norm_mps: float = 0.0
    max_position_3sigma_m: float = 0.0
    max_orientation_3sigma_deg: float = 0.0


def _add(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a: Vec3, scale: float) -> Vec3:
    return (a[0] * scale, a[1] * scale, a[2] * scale)


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a: Vec3) -> float:
    return math.sqrt(_dot(a, a))


def _mean_vec(values: Sequence[Vec3]) -> Vec3:
    if not values:
        return (0.0, 0.0, 0.0)
    inv = 1.0 / float(len(values))
    return (
        sum(value[0] for value in values) * inv,
        sum(value[1] for value in values) * inv,
        sum(value[2] for value in values) * inv,
    )


def _sample_variance(values: Sequence[float]) -> float:
    if len(values) < 2:
        return 0.0
    mean = sum(values) / float(len(values))
    return sum((value - mean) ** 2 for value in values) / float(len(values) - 1)


def _normalize_quaternion(q: Quat) -> Quat:
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isfinite(norm) or norm <= 1.0e-15:
        raise ValueError("quaternion must be finite and non-zero")
    normalized = tuple(value / norm for value in q)
    if normalized[3] < 0.0:
        normalized = tuple(-value for value in normalized)
    return normalized  # type: ignore[return-value]


def _quat_multiply(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return _normalize_quaternion(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def _quat_conjugate(q: Quat) -> Quat:
    return (-q[0], -q[1], -q[2], q[3])


def _rotate(q: Quat, vector: Vec3) -> Vec3:
    qv = (q[0], q[1], q[2])
    twice_cross = _scale(_cross(qv, vector), 2.0)
    return _add(vector, _add(_scale(twice_cross, q[3]), _cross(qv, twice_cross)))


def _quat_from_rotation_vector(rotation_vector: Vec3) -> Quat:
    angle = _norm(rotation_vector)
    if angle < 1.0e-12:
        return _normalize_quaternion(
            (
                0.5 * rotation_vector[0],
                0.5 * rotation_vector[1],
                0.5 * rotation_vector[2],
                1.0,
            )
        )
    scale = math.sin(0.5 * angle) / angle
    return (
        rotation_vector[0] * scale,
        rotation_vector[1] * scale,
        rotation_vector[2] * scale,
        math.cos(0.5 * angle),
    )


def _quat_from_two_vectors(source: Vec3, target: Vec3) -> Quat:
    source_norm = _norm(source)
    target_norm = _norm(target)
    if source_norm <= 1.0e-12 or target_norm <= 1.0e-12:
        return (0.0, 0.0, 0.0, 1.0)
    a = _scale(source, 1.0 / source_norm)
    b = _scale(target, 1.0 / target_norm)
    dot = max(-1.0, min(1.0, _dot(a, b)))
    if dot < -1.0 + 1.0e-10:
        trial = (1.0, 0.0, 0.0) if abs(a[0]) < 0.9 else (0.0, 1.0, 0.0)
        axis = _cross(a, trial)
        axis = _scale(axis, 1.0 / _norm(axis))
        return (axis[0], axis[1], axis[2], 0.0)
    cross = _cross(a, b)
    return _normalize_quaternion((cross[0], cross[1], cross[2], 1.0 + dot))


def _rotation_matrix(q: Quat) -> Tuple[Vec3, Vec3, Vec3]:
    x, y, z, w = q
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def _rotate_diagonal_covariance(q: Quat, diagonal: Vec3) -> Vec3:
    rotation = _rotation_matrix(q)
    return tuple(
        sum(rotation[row][column] ** 2 * diagonal[column] for column in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def _rotate_covariance(q: Quat, covariance: Cov9) -> Cov9:
    """Rotate a row-major covariance and preserve the ROS unknown sentinel."""
    if covariance[0] < 0.0:
        return covariance
    rotation = _rotation_matrix(q)
    source = tuple(
        tuple(covariance[3 * row + column] for column in range(3))
        for row in range(3)
    )
    rotated = []
    for row in range(3):
        for column in range(3):
            rotated.append(
                sum(
                    rotation[row][left]
                    * source[left][right]
                    * rotation[column][right]
                    for left in range(3)
                    for right in range(3)
                )
            )
    return tuple(rotated)  # type: ignore[return-value]


def apply_cv7_rx180(sample: ImuSample) -> ImuSample:
    """Map recorded cv7_link y/z vectors into the corrected base convention."""

    angular_velocity = _rotate(RX180_QUATERNION, sample.angular_velocity_radps)
    linear_acceleration = _rotate(RX180_QUATERNION, sample.linear_acceleration_mps2)
    return ImuSample(
        stamp_ns=sample.stamp_ns,
        record_time_ns=sample.record_time_ns,
        frame_id=sample.frame_id,
        angular_velocity_radps=angular_velocity,
        linear_acceleration_mps2=linear_acceleration,
        angular_velocity_covariance=_rotate_covariance(
            RX180_QUATERNION, sample.angular_velocity_covariance
        ),
        linear_acceleration_covariance=_rotate_covariance(
            RX180_QUATERNION, sample.linear_acceleration_covariance
        ),
    )


class OrderedSampleFilter:
    def __init__(self) -> None:
        self.stats = FilterStats()
        self._last_stamp_ns: Optional[int] = None

    def accept(self, sample: ImuSample) -> Optional[ImuSample]:
        self.stats.input_count += 1
        if sample.stamp_ns <= 0:
            self.stats.invalid_stamp_count += 1
            return None
        values = sample.angular_velocity_radps + sample.linear_acceleration_mps2
        if not all(math.isfinite(value) for value in values):
            self.stats.nonfinite_sample_count += 1
            return None
        if self._last_stamp_ns is not None:
            if sample.stamp_ns == self._last_stamp_ns:
                self.stats.duplicate_stamp_count += 1
                return None
            if sample.stamp_ns < self._last_stamp_ns:
                self.stats.nonmonotonic_stamp_count += 1
                return None
        self._last_stamp_ns = sample.stamp_ns
        self.stats.accepted_count += 1
        return sample


def calibrate_initial_stationary(
    samples: Sequence[ImuSample], config: EstimatorConfig
) -> CalibrationResult:
    if not samples:
        return CalibrationResult(
            False,
            "no_samples",
            0,
            0.0,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (math.inf, math.inf, math.inf),
            (math.inf, math.inf, math.inf),
            (0.0, 0.0, 0.0, 1.0),
            0.0,
            math.inf,
            math.inf,
            math.inf,
        )

    gyros = [sample.angular_velocity_radps for sample in samples]
    accels = [sample.linear_acceleration_mps2 for sample in samples]
    gyro_mean = _mean_vec(gyros)
    accel_mean = _mean_vec(accels)
    accel_norms = [_norm(value) for value in accels]
    mean_accel_norm = sum(accel_norms) / float(len(accel_norms))
    accel_norm_std = math.sqrt(_sample_variance(accel_norms))
    gyro_residual_rms = math.sqrt(
        sum(_dot(_sub(value, gyro_mean), _sub(value, gyro_mean)) for value in gyros)
        / float(len(gyros))
    )
    duration_s = (samples[-1].stamp_ns - samples[0].stamp_ns) * 1.0e-9

    reasons = []
    if len(samples) < config.calibration_min_samples:
        reasons.append("insufficient_samples")
    if duration_s < 0.95 * config.calibration_duration_s:
        reasons.append("insufficient_duration")
    if _norm(gyro_mean) > config.calibration_max_gyro_mean_radps:
        reasons.append("gyro_mean_too_large")
    if gyro_residual_rms > config.calibration_max_gyro_rms_radps:
        reasons.append("gyro_rms_too_large")
    if not (
        config.calibration_min_accel_norm_mps2
        <= mean_accel_norm
        <= config.calibration_max_accel_norm_mps2
    ):
        reasons.append("accel_norm_out_of_range")
    if accel_norm_std > config.calibration_max_accel_norm_std_mps2:
        reasons.append("accel_norm_unstable")

    gravity_direction_body = (
        _scale(accel_mean, 1.0 / _norm(accel_mean))
        if _norm(accel_mean) > 1.0e-9
        else (0.0, 0.0, 1.0)
    )
    expected_specific_force = _scale(gravity_direction_body, GRAVITY_MPS2)
    accel_bias = _sub(accel_mean, expected_specific_force)
    initial_quaternion = _quat_from_two_vectors(
        gravity_direction_body, (0.0, 0.0, 1.0)
    )

    sample_count = len(samples)
    gyro_axis_variance = tuple(
        _sample_variance([value[axis] for value in gyros]) for axis in range(3)
    )
    accel_axis_variance = tuple(
        _sample_variance([value[axis] for value in accels]) for axis in range(3)
    )
    gyro_reported = tuple(
        sum(
            _covariance_variance(
                sample.angular_velocity_covariance[axis * 3 + axis],
                config.fallback_gyro_sigma_radps,
            )
            for sample in samples
        )
        / float(sample_count)
        for axis in range(3)
    )
    accel_reported = tuple(
        sum(
            _covariance_variance(
                sample.linear_acceleration_covariance[axis * 3 + axis],
                config.fallback_accel_sigma_mps2,
            )
            for sample in samples
        )
        / float(sample_count)
        for axis in range(3)
    )
    gyro_bias_covariance = tuple(
        (gyro_axis_variance[axis] + gyro_reported[axis]) / float(sample_count)
        for axis in range(3)
    )
    accel_bias_covariance = tuple(
        (accel_axis_variance[axis] + accel_reported[axis]) / float(sample_count)
        for axis in range(3)
    )

    return CalibrationResult(
        valid=not reasons,
        reason="ok" if not reasons else ";".join(reasons),
        sample_count=sample_count,
        duration_s=duration_s,
        gyro_bias_radps=gyro_mean,
        accel_bias_mps2=accel_bias,
        gyro_bias_covariance=gyro_bias_covariance,  # type: ignore[arg-type]
        accel_bias_covariance=accel_bias_covariance,  # type: ignore[arg-type]
        initial_quaternion_xyzw=initial_quaternion,
        mean_accel_norm_mps2=mean_accel_norm,
        accel_norm_std_mps2=accel_norm_std,
        gyro_mean_norm_radps=_norm(gyro_mean),
        gyro_residual_rms_radps=gyro_residual_rms,
    )


def _covariance_variance(value: float, fallback_sigma: float) -> float:
    if math.isfinite(value) and value > 0.0:
        return value
    return fallback_sigma * fallback_sigma


class OpenLoopIntegrator:
    """Streaming strapdown integration with conservative diagonal covariance."""

    def __init__(
        self,
        calibration: CalibrationResult,
        config: EstimatorConfig,
        anchor: Optional[TrajectoryAnchor] = None,
    ) -> None:
        self.calibration = calibration
        self.config = config
        self.anchor = anchor
        self.position: Vec3 = (0.0, 0.0, 0.0)
        self.velocity: Vec3 = (0.0, 0.0, 0.0)
        self.quaternion: Quat = calibration.initial_quaternion_xyzw
        self.position_variance: Vec3 = (0.0, 0.0, 0.0)
        self.velocity_variance: Vec3 = (0.0, 0.0, 0.0)
        initial_tilt_variance = max(calibration.accel_bias_covariance) / (
            GRAVITY_MPS2 * GRAVITY_MPS2
        )
        self.orientation_variance: Vec3 = (
            initial_tilt_variance,
            initial_tilt_variance,
            0.0,
        )
        self.gyro_bias_variance: Vec3 = calibration.gyro_bias_covariance
        self.accel_bias_variance: Vec3 = calibration.accel_bias_covariance
        self.previous: Optional[ImuSample] = None
        self.start_stamp_ns: Optional[int] = None
        self.segment_id = 0
        self.continuity_valid = True
        self.stationary_window: Deque[ImuSample] = deque()
        self.stats = RunStats()
        self._anchor_from_relative = self._anchor_transform()

    def _anchor_transform(self) -> Quat:
        if self.anchor is None:
            return (0.0, 0.0, 0.0, 1.0)
        # M_R_W = M_R_B0 * inverse(W_R_B0); this makes the first pose exactly
        # equal to the supplied trajectory anchor without double-counting tilt.
        return _quat_multiply(
            self.anchor.quaternion_xyzw,
            _quat_conjugate(self.calibration.initial_quaternion_xyzw),
        )

    def process(self, sample: ImuSample) -> dict[str, object]:
        if self.start_stamp_ns is None:
            self.start_stamp_ns = sample.stamp_ns
            self.stats.first_stamp_ns = sample.stamp_ns
        elapsed_s = (sample.stamp_ns - self.start_stamp_ns) * 1.0e-9
        dt_s = 0.0
        integration_gap = False

        if self.previous is not None:
            dt_s = (sample.stamp_ns - self.previous.stamp_ns) * 1.0e-9
            if dt_s > self.config.max_integration_gap_s:
                integration_gap = True
                self.stats.integration_gap_count += 1
                self.segment_id += 1
                self.continuity_valid = False
            else:
                self._propagate(self.previous, sample, dt_s)

        self.stationary_window.append(sample)
        cutoff_ns = sample.stamp_ns - int(self.config.stationary_window_s * 1.0e9)
        while self.stationary_window and self.stationary_window[0].stamp_ns < cutoff_ns:
            self.stationary_window.popleft()
        stationary_candidate = self._stationary_candidate()
        zupt_candidate = stationary_candidate
        if stationary_candidate:
            self.stats.stationary_candidate_count += 1
        if zupt_candidate:
            self.stats.zupt_candidate_count += 1

        position_3sigma_m = 3.0 * math.sqrt(max(self.position_variance))
        orientation_3sigma_deg = math.degrees(
            3.0 * math.sqrt(max(self.orientation_variance))
        )
        validity_reason = "ok"
        within_valid_horizon = True
        if not self.calibration.valid:
            validity_reason = "initial_stationary_calibration_invalid"
            within_valid_horizon = False
        elif not self.continuity_valid:
            validity_reason = "timestamp_gap"
            within_valid_horizon = False
        elif elapsed_s > self.config.max_valid_horizon_s:
            validity_reason = "configured_max_valid_horizon"
            within_valid_horizon = False
        elif position_3sigma_m > self.config.max_position_3sigma_m:
            validity_reason = "position_covariance_limit"
            within_valid_horizon = False
        elif orientation_3sigma_deg > self.config.max_orientation_3sigma_deg:
            validity_reason = "orientation_covariance_limit"
            within_valid_horizon = False

        if within_valid_horizon:
            self.stats.last_valid_elapsed_s = elapsed_s
        elif not self.stats.first_invalid_reason:
            self.stats.first_invalid_reason = validity_reason

        output_position, output_velocity, output_quaternion = self._output_pose()
        output_position_variance = _rotate_diagonal_covariance(
            self._anchor_from_relative, self.position_variance
        )
        output_velocity_variance = _rotate_diagonal_covariance(
            self._anchor_from_relative, self.velocity_variance
        )
        output_orientation_variance = _rotate_diagonal_covariance(
            self._anchor_from_relative, self.orientation_variance
        )

        corrected_gyro = _sub(
            sample.angular_velocity_radps, self.calibration.gyro_bias_radps
        )
        corrected_accel = _sub(
            sample.linear_acceleration_mps2, self.calibration.accel_bias_mps2
        )
        self.stats.output_count += 1
        self.stats.last_stamp_ns = sample.stamp_ns
        self.stats.max_position_norm_m = max(
            self.stats.max_position_norm_m, _norm(output_position)
        )
        self.stats.max_velocity_norm_mps = max(
            self.stats.max_velocity_norm_mps, _norm(output_velocity)
        )
        self.stats.max_position_3sigma_m = max(
            self.stats.max_position_3sigma_m, position_3sigma_m
        )
        self.stats.max_orientation_3sigma_deg = max(
            self.stats.max_orientation_3sigma_deg, orientation_3sigma_deg
        )

        row: dict[str, object] = {
            "stamp_ns": sample.stamp_ns,
            "record_time_ns": sample.record_time_ns,
            "elapsed_s": elapsed_s,
            "dt_s": dt_s,
            "segment_id": self.segment_id,
            "input_frame_id": sample.frame_id,
            "output_frame_id": self.anchor.frame_id if self.anchor else "imu_relative_gravity",
            "position_x_m": output_position[0],
            "position_y_m": output_position[1],
            "position_z_m": output_position[2],
            "velocity_x_mps": output_velocity[0],
            "velocity_y_mps": output_velocity[1],
            "velocity_z_mps": output_velocity[2],
            "quaternion_x": output_quaternion[0],
            "quaternion_y": output_quaternion[1],
            "quaternion_z": output_quaternion[2],
            "quaternion_w": output_quaternion[3],
            "gyro_bias_x_radps": self.calibration.gyro_bias_radps[0],
            "gyro_bias_y_radps": self.calibration.gyro_bias_radps[1],
            "gyro_bias_z_radps": self.calibration.gyro_bias_radps[2],
            "accel_bias_x_mps2": self.calibration.accel_bias_mps2[0],
            "accel_bias_y_mps2": self.calibration.accel_bias_mps2[1],
            "accel_bias_z_mps2": self.calibration.accel_bias_mps2[2],
            "corrected_gyro_x_radps": corrected_gyro[0],
            "corrected_gyro_y_radps": corrected_gyro[1],
            "corrected_gyro_z_radps": corrected_gyro[2],
            "corrected_accel_x_mps2": corrected_accel[0],
            "corrected_accel_y_mps2": corrected_accel[1],
            "corrected_accel_z_mps2": corrected_accel[2],
            "position_cov_xx_m2": output_position_variance[0],
            "position_cov_yy_m2": output_position_variance[1],
            "position_cov_zz_m2": output_position_variance[2],
            "velocity_cov_xx_m2ps2": output_velocity_variance[0],
            "velocity_cov_yy_m2ps2": output_velocity_variance[1],
            "velocity_cov_zz_m2ps2": output_velocity_variance[2],
            "orientation_cov_xx_rad2": output_orientation_variance[0],
            "orientation_cov_yy_rad2": output_orientation_variance[1],
            "orientation_cov_zz_rad2": output_orientation_variance[2],
            "gyro_bias_cov_xx_rad2ps2": self.gyro_bias_variance[0],
            "gyro_bias_cov_yy_rad2ps2": self.gyro_bias_variance[1],
            "gyro_bias_cov_zz_rad2ps2": self.gyro_bias_variance[2],
            "accel_bias_cov_xx_m2ps4": self.accel_bias_variance[0],
            "accel_bias_cov_yy_m2ps4": self.accel_bias_variance[1],
            "accel_bias_cov_zz_m2ps4": self.accel_bias_variance[2],
            "position_3sigma_m": position_3sigma_m,
            "orientation_3sigma_deg": orientation_3sigma_deg,
            "stationary_candidate": int(stationary_candidate),
            "zupt_candidate": int(zupt_candidate),
            "zupt_applied": 0,
            "integration_gap": int(integration_gap),
            "propagation_valid": int(self.calibration.valid and self.continuity_valid),
            "within_diagnostic_horizon": int(within_valid_horizon),
            "diagnostic_horizon_reason": validity_reason,
            "quality_gate_eligible": 0,
            # Backward-compatible aliases. These are diagnostic labels, not a
            # calibrated statistical confidence interval or an acceptance gate.
            "within_valid_horizon": int(within_valid_horizon),
            "validity_reason": validity_reason,
            # Filled with the final run-wide value during the bounded CSV pass.
            "valid_horizon_s": "",
        }
        self.previous = sample
        return row

    def _propagate(self, previous: ImuSample, current: ImuSample, dt_s: float) -> None:
        previous_gyro = _sub(
            previous.angular_velocity_radps, self.calibration.gyro_bias_radps
        )
        current_gyro = _sub(
            current.angular_velocity_radps, self.calibration.gyro_bias_radps
        )
        midpoint_gyro = _scale(_add(previous_gyro, current_gyro), 0.5)
        next_quaternion = _quat_multiply(
            self.quaternion,
            _quat_from_rotation_vector(_scale(midpoint_gyro, dt_s)),
        )

        gravity_world: Vec3 = (0.0, 0.0, -GRAVITY_MPS2)
        previous_specific_force = _sub(
            previous.linear_acceleration_mps2, self.calibration.accel_bias_mps2
        )
        current_specific_force = _sub(
            current.linear_acceleration_mps2, self.calibration.accel_bias_mps2
        )
        previous_world_accel = _add(
            _rotate(self.quaternion, previous_specific_force), gravity_world
        )
        current_world_accel = _add(
            _rotate(next_quaternion, current_specific_force), gravity_world
        )
        midpoint_world_accel = _scale(
            _add(previous_world_accel, current_world_accel), 0.5
        )
        self.position = _add(
            self.position,
            _add(
                _scale(self.velocity, dt_s),
                _scale(midpoint_world_accel, 0.5 * dt_s * dt_s),
            ),
        )
        self.velocity = _add(self.velocity, _scale(midpoint_world_accel, dt_s))
        self.quaternion = next_quaternion
        self._propagate_diagonal_covariance(previous, current, dt_s)

    def _propagate_diagonal_covariance(
        self, previous: ImuSample, current: ImuSample, dt_s: float
    ) -> None:
        gyro_measurement_variance = tuple(
            0.5
            * (
                _covariance_variance(
                    previous.angular_velocity_covariance[axis * 3 + axis],
                    self.config.fallback_gyro_sigma_radps,
                )
                + _covariance_variance(
                    current.angular_velocity_covariance[axis * 3 + axis],
                    self.config.fallback_gyro_sigma_radps,
                )
            )
            for axis in range(3)
        )
        accel_measurement_variance = tuple(
            0.5
            * (
                _covariance_variance(
                    previous.linear_acceleration_covariance[axis * 3 + axis],
                    self.config.fallback_accel_sigma_mps2,
                )
                + _covariance_variance(
                    current.linear_acceleration_covariance[axis * 3 + axis],
                    self.config.fallback_accel_sigma_mps2,
                )
            )
            for axis in range(3)
        )
        old_velocity_variance = self.velocity_variance
        gyro_rw_variance = self.config.gyro_bias_random_walk_radps_per_sqrt_s ** 2
        accel_rw_variance = self.config.accel_bias_random_walk_mps2_per_sqrt_s ** 2
        self.gyro_bias_variance = tuple(
            value + gyro_rw_variance * dt_s for value in self.gyro_bias_variance
        )  # type: ignore[assignment]
        self.accel_bias_variance = tuple(
            value + accel_rw_variance * dt_s for value in self.accel_bias_variance
        )  # type: ignore[assignment]
        self.orientation_variance = tuple(
            self.orientation_variance[axis]
            + (gyro_measurement_variance[axis] + self.gyro_bias_variance[axis])
            * dt_s
            * dt_s
            for axis in range(3)
        )  # type: ignore[assignment]

        # A diagonal first-order approximation.  The maximum attitude variance
        # is coupled through gravity to avoid presenting optimistic position
        # covariance when roll/pitch drift dominates.
        attitude_accel_variance = (
            GRAVITY_MPS2 * GRAVITY_MPS2 * max(self.orientation_variance)
        )
        body_accel_variance = max(accel_measurement_variance)
        world_accel_variance = tuple(
            body_accel_variance
            + self.accel_bias_variance[axis]
            + attitude_accel_variance
            for axis in range(3)
        )
        self.position_variance = tuple(
            self.position_variance[axis]
            + old_velocity_variance[axis] * dt_s * dt_s
            + 0.25 * world_accel_variance[axis] * dt_s ** 4
            for axis in range(3)
        )  # type: ignore[assignment]
        self.velocity_variance = tuple(
            old_velocity_variance[axis]
            + world_accel_variance[axis] * dt_s * dt_s
            for axis in range(3)
        )  # type: ignore[assignment]

    def _stationary_candidate(self) -> bool:
        if len(self.stationary_window) < 3:
            return False
        span_s = (
            self.stationary_window[-1].stamp_ns
            - self.stationary_window[0].stamp_ns
        ) * 1.0e-9
        if span_s < 0.8 * self.config.stationary_window_s:
            return False
        gyro_residuals = [
            _sub(sample.angular_velocity_radps, self.calibration.gyro_bias_radps)
            for sample in self.stationary_window
        ]
        gyro_rms = math.sqrt(
            sum(_dot(value, value) for value in gyro_residuals)
            / float(len(gyro_residuals))
        )
        accel_norms = [
            _norm(_sub(sample.linear_acceleration_mps2, self.calibration.accel_bias_mps2))
            for sample in self.stationary_window
        ]
        mean_accel_norm = sum(accel_norms) / float(len(accel_norms))
        accel_norm_std = math.sqrt(_sample_variance(accel_norms))
        return (
            gyro_rms <= self.config.stationary_max_gyro_rms_radps
            and abs(mean_accel_norm - GRAVITY_MPS2)
            <= self.config.stationary_max_accel_norm_error_mps2
            and accel_norm_std <= self.config.stationary_max_accel_norm_std_mps2
        )

    def _output_pose(self) -> Tuple[Vec3, Vec3, Quat]:
        if self.anchor is None:
            return self.position, self.velocity, self.quaternion
        position = _add(
            self.anchor.position_m,
            _rotate(self._anchor_from_relative, self.position),
        )
        velocity = _rotate(self._anchor_from_relative, self.velocity)
        quaternion = _quat_multiply(self._anchor_from_relative, self.quaternion)
        return position, velocity, quaternion


def _extract_covariance(values: Sequence[float]) -> Cov9:
    if len(values) != 9:
        return (0.0,) * 9
    return tuple(float(value) for value in values)  # type: ignore[return-value]


def iter_ros2_sqlite_imu(
    bag_db: Path, topic_name: str = "/imu/data"
) -> Iterator[ImuSample]:
    """Stream sensor_msgs/msg/Imu records without loading a bag into memory."""

    try:
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Imu
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Python packages are required; source /opt/ros/<distro>/setup.bash"
        ) from error

    database_paths = _resolve_database_paths(bag_db)
    found_topic = False
    for database_path in database_paths:
        uri = f"file:{database_path}?mode=ro"
        connection = sqlite3.connect(uri, uri=True)
        try:
            topic_rows = connection.execute(
                "SELECT id, type, serialization_format FROM topics WHERE name = ?",
                (topic_name,),
            ).fetchall()
            if not topic_rows:
                continue
            if len(topic_rows) != 1:
                raise RuntimeError(
                    f"{database_path}: topic {topic_name!r} has {len(topic_rows)} schemas"
                )
            topic_id, message_type, serialization_format = topic_rows[0]
            if message_type != "sensor_msgs/msg/Imu":
                raise RuntimeError(
                    f"{database_path}: {topic_name} is {message_type}, expected sensor_msgs/msg/Imu"
                )
            if serialization_format not in ("cdr", ""):
                raise RuntimeError(
                    f"{database_path}: unsupported serialization {serialization_format!r}"
                )
            found_topic = True
            cursor = connection.execute(
                "SELECT timestamp, data FROM messages "
                "WHERE topic_id = ? ORDER BY timestamp, id",
                (topic_id,),
            )
            for record_time_ns, payload in cursor:
                message = deserialize_message(bytes(payload), Imu)
                stamp_ns = (
                    int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec)
                )
                yield ImuSample(
                    stamp_ns=stamp_ns,
                    record_time_ns=int(record_time_ns),
                    frame_id=str(message.header.frame_id),
                    angular_velocity_radps=(
                        float(message.angular_velocity.x),
                        float(message.angular_velocity.y),
                        float(message.angular_velocity.z),
                    ),
                    linear_acceleration_mps2=(
                        float(message.linear_acceleration.x),
                        float(message.linear_acceleration.y),
                        float(message.linear_acceleration.z),
                    ),
                    angular_velocity_covariance=_extract_covariance(
                        message.angular_velocity_covariance
                    ),
                    linear_acceleration_covariance=_extract_covariance(
                        message.linear_acceleration_covariance
                    ),
                )
        finally:
            connection.close()
    if not found_topic:
        raise RuntimeError(
            f"topic {topic_name!r} was not found in {', '.join(map(str, database_paths))}"
        )


def _natural_path_key(path: Path) -> tuple[object, ...]:
    return tuple(
        int(token) if token.isdigit() else token.lower()
        for token in re.split(r"([0-9]+)", path.name)
    )


def _metadata_database_paths(directory: Path) -> Optional[list[Path]]:
    metadata_path = directory / "metadata.yaml"
    if not metadata_path.exists():
        return None
    if not metadata_path.is_file() or metadata_path.is_symlink():
        raise ValueError(f"invalid rosbag metadata file: {metadata_path}")
    try:
        import yaml
    except ImportError as error:
        raise RuntimeError(
            "python3-yaml is required to honor rosbag metadata storage order"
        ) from error
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError(f"cannot read rosbag metadata {metadata_path}: {error}") from error
    if not isinstance(metadata, dict):
        raise ValueError(f"invalid rosbag metadata root: {metadata_path}")
    information = metadata.get("rosbag2_bagfile_information")
    if not isinstance(information, dict):
        raise ValueError(f"invalid rosbag metadata body: {metadata_path}")
    storage_identifier = information.get("storage_identifier")
    if storage_identifier not in (None, "sqlite3"):
        raise ValueError(
            f"unsupported rosbag storage_identifier {storage_identifier!r}: {metadata_path}"
        )
    relative_paths = information.get("relative_file_paths")
    if not isinstance(relative_paths, list) or not relative_paths:
        raise ValueError(f"rosbag metadata lacks relative_file_paths: {metadata_path}")

    result: list[Path] = []
    seen: set[Path] = set()
    for index, raw_path in enumerate(relative_paths):
        if not isinstance(raw_path, str) or not raw_path:
            raise ValueError(
                f"invalid relative_file_paths[{index}] in {metadata_path}"
            )
        relative = Path(raw_path)
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"unsafe rosbag metadata path {raw_path!r}: {metadata_path}")
        candidate = directory / relative
        if candidate.suffix != ".db3":
            raise ValueError(f"non-sqlite rosbag storage: {candidate}")
        if not candidate.is_file() or candidate.is_symlink():
            raise FileNotFoundError(f"missing regular rosbag storage: {candidate}")
        resolved = candidate.resolve()
        if directory != resolved and directory not in resolved.parents:
            raise ValueError(f"rosbag metadata path escapes bag directory: {candidate}")
        if resolved in seen:
            raise ValueError(f"duplicate rosbag storage path: {candidate}")
        seen.add(resolved)
        result.append(resolved)

    discovered = {
        candidate.resolve()
        for candidate in directory.glob("*.db3")
        if candidate.is_file()
    }
    if discovered != seen:
        raise ValueError(
            f"rosbag metadata/storage set mismatch under {directory}; "
            f"unlisted={sorted(str(path) for path in discovered - seen)}, "
            f"missing={sorted(str(path) for path in seen - discovered)}"
        )
    return result


def _resolve_database_paths(bag_db: Path) -> list[Path]:
    path = bag_db.expanduser().resolve()
    if path.is_file():
        if path.suffix != ".db3":
            raise ValueError(f"--bag-db must be a .db3 file or bag directory: {path}")
        return [path]
    if not path.is_dir():
        raise FileNotFoundError(path)
    databases = _metadata_database_paths(path)
    if databases is None:
        databases = sorted(path.glob("*.db3"), key=_natural_path_key)
    if not databases:
        raise FileNotFoundError(f"no .db3 files found under {path}")
    return databases


def load_trajectory_anchor(path: Path) -> TrajectoryAnchor:
    source = path.expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    if source.suffix.lower() == ".json":
        data = json.loads(source.read_text(encoding="utf-8"))
        pose = data.get("pose", data)
        position = pose.get("position", pose)
        orientation = pose.get("orientation", pose)
        anchor = TrajectoryAnchor(
            position_m=(
                float(_mapping_value(position, "x", "position_x_m")),
                float(_mapping_value(position, "y", "position_y_m")),
                float(_mapping_value(position, "z", "position_z_m")),
            ),
            quaternion_xyzw=_normalize_quaternion(
                (
                    float(_mapping_value(orientation, "qx", "x", "quaternion_x")),
                    float(_mapping_value(orientation, "qy", "y", "quaternion_y")),
                    float(_mapping_value(orientation, "qz", "z", "quaternion_z")),
                    float(_mapping_value(orientation, "qw", "w", "quaternion_w")),
                )
            ),
            frame_id=str(data.get("frame_id", pose.get("frame_id", "map"))),
            source=str(source),
        )
        return anchor

    with source.open("r", encoding="utf-8", newline="") as stream:
        first_line = stream.readline()
        stream.seek(0)
        if _looks_like_tum(first_line):
            values = [float(value) for value in first_line.split()]
            return TrajectoryAnchor(
                position_m=(values[1], values[2], values[3]),
                quaternion_xyzw=_normalize_quaternion(
                    (values[4], values[5], values[6], values[7])
                ),
                frame_id="map",
                source=str(source),
            )
        reader = csv.DictReader(stream)
        row = next(reader, None)
        if row is None:
            raise ValueError(f"trajectory anchor CSV is empty: {source}")
        return TrajectoryAnchor(
            position_m=(
                float(_mapping_value(row, "position_x_m", "x_m", "x", "pose_x")),
                float(_mapping_value(row, "position_y_m", "y_m", "y", "pose_y")),
                float(_mapping_value(row, "position_z_m", "z_m", "z", "pose_z")),
            ),
            quaternion_xyzw=_normalize_quaternion(
                (
                    float(_mapping_value(row, "quaternion_x", "qx")),
                    float(_mapping_value(row, "quaternion_y", "qy")),
                    float(_mapping_value(row, "quaternion_z", "qz")),
                    float(_mapping_value(row, "quaternion_w", "qw")),
                )
            ),
            frame_id=str(row.get("frame_id", "map") or "map"),
            source=str(source),
        )


def _looks_like_tum(line: str) -> bool:
    parts = line.split()
    if len(parts) != 8:
        return False
    try:
        [float(part) for part in parts]
    except ValueError:
        return False
    return True


def _mapping_value(mapping: dict, *names: str) -> object:
    for name in names:
        if name in mapping and mapping[name] not in (None, ""):
            return mapping[name]
    raise ValueError(f"none of {names!r} are present in trajectory anchor")


def _accepted_corrected_samples(
    raw_samples: Iterable[ImuSample],
    ordered_filter: OrderedSampleFilter,
    expected_frame: str,
) -> Iterator[ImuSample]:
    for raw_sample in raw_samples:
        if expected_frame and raw_sample.frame_id != expected_frame:
            raise RuntimeError(
                f"CV7 Rx180 correction requires frame {expected_frame!r}; "
                f"received {raw_sample.frame_id!r} at {raw_sample.stamp_ns}"
            )
        corrected = apply_cv7_rx180(raw_sample)
        accepted = ordered_filter.accept(corrected)
        if accepted is not None:
            yield accepted


def run_estimation(
    raw_samples: Iterable[ImuSample],
    config: EstimatorConfig,
    row_sink: Callable[[dict[str, object]], None],
    anchor: Optional[TrajectoryAnchor] = None,
    expected_frame: str = "cv7_link",
) -> Tuple[CalibrationResult, FilterStats, RunStats]:
    ordered_filter = OrderedSampleFilter()
    accepted = iter(
        _accepted_corrected_samples(raw_samples, ordered_filter, expected_frame)
    )
    calibration_samples: list[ImuSample] = []
    for sample in accepted:
        calibration_samples.append(sample)
        duration_s = (
            calibration_samples[-1].stamp_ns - calibration_samples[0].stamp_ns
        ) * 1.0e-9
        if (
            duration_s >= config.calibration_duration_s
            and len(calibration_samples) >= config.calibration_min_samples
        ):
            break
    calibration = calibrate_initial_stationary(calibration_samples, config)
    integrator = OpenLoopIntegrator(calibration, config, anchor)
    for sample in calibration_samples:
        row_sink(integrator.process(sample))
    for sample in accepted:
        row_sink(integrator.process(sample))
    return calibration, ordered_filter.stats, integrator.stats


def estimate_samples(
    raw_samples: Iterable[ImuSample],
    config: Optional[EstimatorConfig] = None,
    anchor: Optional[TrajectoryAnchor] = None,
    expected_frame: str = "cv7_link",
) -> Tuple[list[dict[str, object]], CalibrationResult, FilterStats, RunStats]:
    """Memory-backed convenience wrapper used only by bounded synthetic tests."""

    rows: list[dict[str, object]] = []
    calibration, filter_stats, run_stats = run_estimation(
        raw_samples,
        config or EstimatorConfig(),
        rows.append,
        anchor,
        expected_frame,
    )
    for row in rows:
        row["valid_horizon_s"] = run_stats.last_valid_elapsed_s
    return rows, calibration, filter_stats, run_stats


def _write_final_csv_with_horizon(
    temporary_path: Path,
    final_path: Path,
    fieldnames: Sequence[str],
    valid_horizon_s: float,
) -> None:
    finalizing_path = final_path.with_name(f".{final_path.name}.finalizing")
    with temporary_path.open("r", encoding="utf-8", newline="") as source:
        reader = csv.DictReader(source)
        with finalizing_path.open("x", encoding="utf-8", newline="") as destination:
            writer = csv.DictWriter(destination, fieldnames=fieldnames)
            writer.writeheader()
            for row in reader:
                row["valid_horizon_s"] = f"{valid_horizon_s:.9f}"
                writer.writerow(row)
            destination.flush()
            os.fsync(destination.fileno())
    os.replace(finalizing_path, final_path)
    temporary_path.unlink()


def _write_text_atomic(path: Path, payload: str) -> None:
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    finally:
        temporary_path.unlink(missing_ok=True)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def _database_provenance(bag_db: Path) -> dict[str, object]:
    source = bag_db.expanduser().resolve()
    database_paths = _resolve_database_paths(source)
    base = source if source.is_dir() else source.parent
    set_digest = hashlib.sha256()
    records = []
    for database_path in database_paths:
        before = database_path.stat()
        digest = _sha256(database_path)
        after = database_path.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise RuntimeError(f"source bag changed while hashing: {database_path}")
        relative_name = database_path.relative_to(base).as_posix()
        set_digest.update(f"{digest}  {relative_name}\n".encode("utf-8"))
        records.append(
            {
                "path": str(database_path),
                "relative_path": relative_name,
                "size_bytes": after.st_size,
                "mtime_ns": after.st_mtime_ns,
                "sha256": digest,
            }
        )
    metadata_path = source / "metadata.yaml" if source.is_dir() else None
    return {
        "storage_files": records,
        "storage_set_sha256": set_digest.hexdigest(),
        "storage_set_hash_format": "sha256sum-lines-v1",
        "storage_order_source": (
            "metadata.yaml"
            if metadata_path is not None and metadata_path.is_file()
            else "natural_filename_order"
        ),
        "metadata": (
            {
                "path": str(metadata_path),
                "sha256": _sha256(metadata_path),
            }
            if metadata_path is not None and metadata_path.is_file()
            else None
        ),
    }


def _metrics(
    args: argparse.Namespace,
    config: EstimatorConfig,
    calibration: CalibrationResult,
    filter_stats: FilterStats,
    run_stats: RunStats,
    anchor: Optional[TrajectoryAnchor],
    csv_path: Path,
) -> dict[str, object]:
    duration_s = 0.0
    if run_stats.first_stamp_ns is not None and run_stats.last_stamp_ns is not None:
        duration_s = (run_stats.last_stamp_ns - run_stats.first_stamp_ns) * 1.0e-9
    warnings = [
        "IMU-only position is an open-loop double integral, not absolute ground truth.",
        "The diagonal covariance is a non-conservative diagnostic proxy; it is not a calibrated confidence bound or quality gate.",
        "Stationary and ZUPT columns are candidates only; no hard zero-velocity update is applied.",
    ]
    if not calibration.valid:
        warnings.append(f"initial stationary calibration invalid: {calibration.reason}")
    if filter_stats.duplicate_stamp_count or filter_stats.nonmonotonic_stamp_count:
        warnings.append("duplicate/nonmonotonic header stamps were dropped")
    if run_stats.integration_gap_count:
        warnings.append("integration gaps invalidate the continuous trajectory after the first gap")
    bag_provenance = _database_provenance(Path(args.bag_db))
    anchor_record: Optional[dict[str, object]] = None
    if anchor is not None:
        anchor_record = asdict(anchor)
        if anchor.source:
            anchor_source = Path(anchor.source)
            anchor_record["source_sha256"] = _sha256(anchor_source)
            anchor_record["source_size_bytes"] = anchor_source.stat().st_size
    return {
        "schema_version": 1,
        "estimator": "cv7_rx180_imu_open_loop_relative_v1",
        "classification": "diagnostic_relative_reference_not_ground_truth",
        "input": {
            "bag_db": str(Path(args.bag_db).expanduser().resolve()),
            "imu_topic": args.imu_topic,
            "expected_frame": args.expected_frame,
            **bag_provenance,
        },
        "frame_convention": {
            "sensor_to_base_correction": "Rx(180 deg), vector mapping (x, y, z) -> (x, -y, -z)",
            "covariance_correction": (
                "R * covariance * R^T; ROS unknown sentinel is preserved"
            ),
            "unanchored_output_frame": "imu_relative_gravity",
            "gravity_world_mps2": [0.0, 0.0, -GRAVITY_MPS2],
            "yaw_observability": "unobservable; initialized by convention unless an external trajectory anchor is supplied",
        },
        "trajectory_anchor": anchor_record,
        "configuration": asdict(config),
        "calibration": asdict(calibration),
        "timestamp_filter": asdict(filter_stats),
        "integration": {
            **asdict(run_stats),
            "duration_s": duration_s,
        },
        "validity": {
            "classification": "diagnostic_horizon_not_statistical_confidence",
            "quality_gate_eligible": False,
            "covariance_quality_gate_eligible": False,
            "valid_horizon_s": run_stats.last_valid_elapsed_s,
            "first_invalid_reason": run_stats.first_invalid_reason or "end_of_data",
            "full_output_remains_available_beyond_horizon": True,
            "legacy_csv_aliases": ["within_valid_horizon", "validity_reason"],
        },
        "zupt_policy": {
            "mode": "candidate_label_only",
            "hard_zero_velocity_applied": False,
            "external_scan_or_slam_confirmation_required_for_future_hard_zupt": True,
        },
        "outputs": {
            "trajectory_csv": str(csv_path),
            "trajectory_csv_sha256": _sha256(csv_path),
        },
        "warnings": warnings,
    }


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return parsed


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Stream /imu/data from a ROS 2 sqlite bag and write a CV7 Rx180-corrected "
            "open-loop relative trajectory. This output is not absolute ground truth."
        )
    )
    parser.add_argument("--bag-db", required=True, help="ROS 2 .db3 file or bag directory")
    parser.add_argument("--trajectory-anchor", help="optional JSON, CSV, or TUM first pose")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--expected-frame", default="cv7_link")
    parser.add_argument(
        "--calibration-duration-s", type=_positive_float, default=2.0
    )
    parser.add_argument("--calibration-min-samples", type=int, default=100)
    parser.add_argument(
        "--max-integration-gap-s", type=_positive_float, default=0.10
    )
    parser.add_argument("--max-valid-horizon-s", type=_positive_float, default=10.0)
    parser.add_argument("--max-position-3sigma-m", type=_positive_float, default=2.0)
    parser.add_argument(
        "--max-orientation-3sigma-deg", type=_positive_float, default=15.0
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="replace only this tool's derived CSV/JSON in output-dir",
    )
    args = parser.parse_args(argv)
    if args.calibration_min_samples < 3:
        parser.error("--calibration-min-samples must be at least 3")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "imu_open_loop_trajectory.csv"
    temporary_csv_path = output_dir / ".imu_open_loop_trajectory.incomplete.csv"
    finalizing_csv_path = output_dir / ".imu_open_loop_trajectory.csv.finalizing"
    metrics_path = output_dir / "imu_open_loop_metrics.json"
    owned_paths = (csv_path, temporary_csv_path, finalizing_csv_path, metrics_path)
    existing = [path for path in owned_paths if path.exists()]
    if existing and not args.overwrite:
        raise FileExistsError(
            "refusing to overwrite derived evidence; use a new output directory or "
            f"--overwrite: {', '.join(map(str, existing))}"
        )
    for path in existing:
        path.unlink()

    config = EstimatorConfig(
        calibration_duration_s=args.calibration_duration_s,
        calibration_min_samples=args.calibration_min_samples,
        max_integration_gap_s=args.max_integration_gap_s,
        max_valid_horizon_s=args.max_valid_horizon_s,
        max_position_3sigma_m=args.max_position_3sigma_m,
        max_orientation_3sigma_deg=args.max_orientation_3sigma_deg,
    )
    anchor = (
        load_trajectory_anchor(Path(args.trajectory_anchor))
        if args.trajectory_anchor
        else None
    )
    raw_samples = iter_ros2_sqlite_imu(Path(args.bag_db), args.imu_topic)

    writer: Optional[csv.DictWriter] = None
    fieldnames: list[str] = []
    stream: Optional[TextIO] = None
    try:
        stream = temporary_csv_path.open("w", encoding="utf-8", newline="")

        def write_row(row: dict[str, object]) -> None:
            nonlocal writer, fieldnames
            if writer is None:
                fieldnames = list(row.keys())
                writer = csv.DictWriter(stream, fieldnames=fieldnames)
                writer.writeheader()
            writer.writerow(row)

        calibration, filter_stats, run_stats = run_estimation(
            raw_samples,
            config,
            write_row,
            anchor,
            args.expected_frame,
        )
    finally:
        if stream is not None:
            stream.close()

    if writer is None:
        temporary_csv_path.unlink(missing_ok=True)
        raise RuntimeError("no valid IMU samples were available")
    _write_final_csv_with_horizon(
        temporary_csv_path,
        csv_path,
        fieldnames,
        run_stats.last_valid_elapsed_s,
    )
    metrics = _metrics(
        args,
        config,
        calibration,
        filter_stats,
        run_stats,
        anchor,
        csv_path,
    )
    _write_text_atomic(
        metrics_path,
        json.dumps(metrics, indent=2, sort_keys=True) + "\n",
    )
    print(json.dumps({"csv": str(csv_path), "metrics": str(metrics_path), "valid_horizon_s": run_stats.last_valid_elapsed_s}))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (FileNotFoundError, FileExistsError, RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(2)
