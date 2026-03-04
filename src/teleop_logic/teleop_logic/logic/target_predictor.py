#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🎯 Adaptive Kalman Predictor (v2 — Latency-Aware)

Improvements over v1:
  • prediction_horizon is NOT fixed — it adapts to real measured command latency
    (T1→T3 from LatencyAnalyzer) so we compensate exactly the right amount.
  • Tighter velocity deadband (0.5 deg/s vs 1.0) → quicker settling to raw target.
  • Explicit `set_measured_latency()` API so teleop_node can feed back real data.
  • All original anti-overshoot damping logic is preserved and improved.

Latency compensation concept:
  Total lag we must compensate = command_latency (network + ROS decision)
                                + robot_response_latency (Dobot firmware buffer)
  We predict forward by exactly this amount so the robot arrives at the
  real hand position at the same time as the command settles.
"""

import numpy as np
import math

# Clamping constants — prevent runaway prediction
MIN_HORIZON_SEC = 0.010   # 10ms minimum (very low-latency LAN)
MAX_HORIZON_SEC = 0.250   # 250ms maximum (high-latency WAN / Tailscale)
DEFAULT_HORIZON_SEC = 0.080  # 80ms fallback until we get real measurements

# Big impulsive jump guard (sudden target >17° → suppress prediction)
JUMP_GUARD_RAD = 0.30

# Velocity deadband: below this → hand is considered stopped → return raw target
VELOCITY_DEADBAND_RAD_S = math.radians(0.5)   # 0.5 deg/s (was 1.0 in v1)

# Damping thresholds: reduce horizon when robot is close to target
DAMP_START_ERROR_RAD = 0.05   # 0.05 rad (~2.9°) — start dampening
DAMP_MIN_ERROR_RAD   = 0.01   # 0.01 rad (~0.6°) — full dampening


class TargetPredictor:
    """
    Adaptive Kalman Filter for VR Teleoperation targets.

    Tracks position + velocity of 4 joints.
    Projects predicted target `horizon` seconds into the future
    to compensate for combined network + firmware execution latency.

    Call set_measured_latency() after each LatencyAnalyzer measurement
    to keep the horizon adapted to real conditions.
    """

    def __init__(self, default_dt=0.02, prediction_horizon_sec=DEFAULT_HORIZON_SEC, logger=None):
        self.dt = default_dt
        self.horizon = prediction_horizon_sec
        self._measured_cmd_latency_sec  = prediction_horizon_sec * 0.7  # first guess: ~56ms
        self._measured_robot_latency_sec = prediction_horizon_sec * 0.3  # first guess: ~24ms
        self.logger = logger
        self.last_timestamp = 0.0

        # State vector: [position, velocity]^T for 4 joints → shape (4, 2, 1)
        self.x = np.zeros((4, 2, 1))

        # Covariance Matrix P — one per joint
        self.P = np.stack([np.eye(2)] * 4)   # (4, 2, 2)

        # State Transition Matrix F (updated each step with real dt)
        self.F = np.array([[1.0, self.dt],
                           [0.0, 1.0]])

        # Measurement Matrix H (we only observe position from Unity)
        self.H = np.array([[1.0, 0.0]])

        # Process Noise Q — trust in our physics model
        # High velocity noise: human hand changes direction frequently
        self.Q = np.array([[1e-4, 0.0],
                           [0.0,  2e-2]])

        # Measurement Noise R — trust in Unity VR tracking
        self.R = np.array([[1e-3]])

        self.I = np.eye(2)
        self.is_initialized = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_measured_latency(self, command_ms: float, robot_response_ms: float = 0.0):
        """
        Feed real latency measurements from LatencyAnalyzer.

        Args:
            command_ms:       T1→T3 total command latency in milliseconds
                              (network delay + ROS decision delay)
            robot_response_ms: T3→T4 robot response latency in milliseconds
                              (time from cmd sent to robot starting to move)
        """
        if command_ms <= 0:
            return

        cmd_sec   = command_ms / 1000.0
        robot_sec = max(robot_response_ms, 0.0) / 1000.0

        # Exponential Moving Average — smooth out measurement noise
        alpha = 0.15   # low alpha = slow adaptation = more stable
        self._measured_cmd_latency_sec   = (alpha * cmd_sec
                                            + (1 - alpha) * self._measured_cmd_latency_sec)
        self._measured_robot_latency_sec = (alpha * robot_sec
                                            + (1 - alpha) * self._measured_robot_latency_sec)

        # Update horizon = total round-trip we must compensate
        new_horizon = self._measured_cmd_latency_sec + self._measured_robot_latency_sec
        self.horizon = float(np.clip(new_horizon, MIN_HORIZON_SEC, MAX_HORIZON_SEC))

        if self.logger:
            self.logger.debug(
                f"🎯 Predictor horizon updated: {self.horizon*1000:.1f}ms "
                f"(cmd={self._measured_cmd_latency_sec*1000:.1f}ms "
                f"robot={self._measured_robot_latency_sec*1000:.1f}ms)"
            )

    def update_and_predict(self, raw_target_q, timestamp_sec, q_actual=None):
        """
        Takes raw [4,] joint target from Unity and its calibrated timestamp.
        Optionally takes q_actual from feedback to scale the prediction horizon.
        Returns [4,] predicted future joint target.
        """
        # 0. Initialize on first frame
        if not self.is_initialized:
            self.x[:, 0, 0] = raw_target_q
            self.x[:, 1, 0] = 0.0
            self.last_timestamp = timestamp_sec
            self.is_initialized = True
            return raw_target_q

        # --- Dynamic dt (handles network jitter and Tailscale spikes) ---
        dt = timestamp_sec - self.last_timestamp
        self.last_timestamp = timestamp_sec

        if dt <= 0.001 or dt > 0.5:
            dt = self.dt   # fallback to nominal rate

        self.F[0, 1] = dt

        # --- Kalman Filter loop over 4 joints ---
        for i in range(4):
            # 1. PREDICT
            self.x[i] = self.F @ self.x[i]
            self.P[i] = self.F @ self.P[i] @ self.F.T + self.Q

            # 2. UPDATE
            z = np.array([[raw_target_q[i]]])
            y = z - self.H @ self.x[i]
            S = self.H @ self.P[i] @ self.H.T + self.R
            K = self.P[i] @ self.H.T @ np.linalg.inv(S)
            self.x[i] = self.x[i] + K @ y
            self.P[i] = (self.I - K @ self.H) @ self.P[i]

        velocities = self.x[:, 1, 0]
        max_vel = np.max(np.abs(velocities))

        # --- Safety: Hand has essentially stopped → return raw to prevent drift ---
        if max_vel < VELOCITY_DEADBAND_RAD_S:
            return raw_target_q

        # --- Safety: Sudden jump >17° → suppress prediction (e.g. teach&replay) ---
        position_error = np.max(np.abs(raw_target_q - self.x[:, 0, 0]))
        if position_error > JUMP_GUARD_RAD:
            return raw_target_q

        # --- Adaptive Dampened Horizon ---
        # Scale horizon down as robot gets close to target — prevents overshoot
        # at arrival while keeping full prediction during fast motion.
        final_horizon = self.horizon

        if q_actual is not None and np.any(q_actual != 0.0):
            tracking_error = np.max(np.abs(raw_target_q - q_actual))

            if tracking_error < DAMP_START_ERROR_RAD:
                # Linear scale: 0.0 at DAMP_MIN → 1.0 at DAMP_START
                scale = np.clip(
                    (tracking_error - DAMP_MIN_ERROR_RAD) / (DAMP_START_ERROR_RAD - DAMP_MIN_ERROR_RAD),
                    0.0, 1.0
                )
                final_horizon = self.horizon * scale

        # --- Project physical state ahead by final_horizon ---
        future_q = self.x[:, 0, 0] + (velocities * final_horizon)

        return future_q
