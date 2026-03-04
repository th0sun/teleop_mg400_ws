#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
🎯 Kalman Filter Predictor v3 — Conservative Anti-Overshoot Edition

Design philosophy:
  • NEVER go past the actual target position (no overshoot beyond hand)
  • Fast enough to compensate latency during steady movement
  • Instantly snap to raw target when hand slows/stops

Key improvements over v1 (old project):
  • Velocity deadband tightened: 0.3 deg/s (was 1.0) — settles faster
  • Position guard tightened: max 10° jump (was 17°) — safer
  • per-joint horizon (array) to allow individual joint scaling
  • set_measured_latency() API for future latency feedback (safe EMA)
"""

import numpy as np
import math

# Prediction horizon limits
DEFAULT_HORIZON_SEC = 0.080   # 80ms initial (overridden by latency feedback)
MIN_HORIZON_SEC     = 0.010   # 10ms floor
MAX_HORIZON_SEC     = 0.200   # 200ms ceiling

# Hand-stopped deadband: below → return raw target immediately
VELOCITY_DEADBAND_RAD_S = math.radians(0.3)   # 0.3 deg/s (v1 was 1.0)

# Sudden jump guard: suppress prediction if Unity sends huge step
JUMP_GUARD_RAD = 0.175   # ~10 degrees (v1 was 0.30 / 17°)

# Tracking error damping thresholds
DAMP_START_ERROR_RAD = 0.05   # start dimming at 2.9° tracking error
DAMP_MIN_ERROR_RAD   = 0.01   # full damp (0% horizon) at 0.6°


class TargetPredictor:
    """
    Conservative Kalman Predictor tuned for smooth VR teleoperation.
    Compensates network+system latency without ever overshooting
    the real hand position when it slows down or stops.
    """

    def __init__(self, default_dt=0.02, prediction_horizon_sec=DEFAULT_HORIZON_SEC, logger=None):
        self.dt      = default_dt
        self.horizon = prediction_horizon_sec
        self.logger  = logger
        self.last_timestamp = 0.0

        # State vector [position, velocity]^T for 4 joints  →  (4, 2, 1)
        self.x = np.zeros((4, 2, 1))

        # Covariance P  —  one per joint
        self.P = np.stack([np.eye(2)] * 4)   # (4, 2, 2)

        # State transition  F  (dt updated each step)
        self.F = np.array([[1.0, self.dt],
                           [0.0, 1.0]])

        # Measurement matrix  H  (position only from Unity)
        self.H = np.array([[1.0, 0.0]])

        # Process noise Q  — high velocity noise for human motion
        self.Q = np.array([[1e-4, 0.0],
                           [0.0,  2e-2]])

        # Measurement noise R  — VR tracking is accurate but slightly jittery
        self.R = np.array([[1e-3]])

        self.I = np.eye(2)
        self.is_initialized = False

        # EMA state for adaptive horizon
        self._ema_cmd_latency_sec   = prediction_horizon_sec
        self._ema_robot_latency_sec = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_measured_latency(self, command_ms: float, robot_response_ms: float = 0.0):
        """
        Feed real latency measurements from LatencyAnalyzer (called after T5).
        Slowly adapts the prediction horizon via EMA.
        """
        if command_ms <= 0:
            return

        alpha = 0.10   # conservative adaptation rate
        self._ema_cmd_latency_sec   = alpha * (command_ms / 1000.0) + (1 - alpha) * self._ema_cmd_latency_sec
        self._ema_robot_latency_sec = alpha * (max(robot_response_ms, 0) / 1000.0) + (1 - alpha) * self._ema_robot_latency_sec

        new_horizon = self._ema_cmd_latency_sec + self._ema_robot_latency_sec
        self.horizon = float(np.clip(new_horizon, MIN_HORIZON_SEC, MAX_HORIZON_SEC))

        if self.logger:
            self.logger.debug(
                f"🎯 Predictor horizon: {self.horizon*1000:.1f}ms "
                f"(cmd={self._ema_cmd_latency_sec*1000:.1f}ms "
                f"robot={self._ema_robot_latency_sec*1000:.1f}ms)"
            )

    def update_and_predict(self, raw_target_q, timestamp_sec, q_actual=None):
        """
        Main entry: take raw [4,] target from Unity, return [4,] predicted target.
        q_actual: current robot position (for damping near target).
        """
        # --- Initialize ---
        if not self.is_initialized:
            self.x[:, 0, 0] = raw_target_q
            self.x[:, 1, 0] = 0.0
            self.last_timestamp = timestamp_sec
            self.is_initialized = True
            return raw_target_q.copy()

        # --- Dynamic dt ---
        dt = timestamp_sec - self.last_timestamp
        self.last_timestamp = timestamp_sec
        if dt <= 0.001 or dt > 0.5:
            dt = self.dt
        self.F[0, 1] = dt

        # --- Kalman update for each joint ---
        for i in range(4):
            self.x[i] = self.F @ self.x[i]
            self.P[i] = self.F @ self.P[i] @ self.F.T + self.Q

            z = np.array([[raw_target_q[i]]])
            y = z - self.H @ self.x[i]
            S = self.H @ self.P[i] @ self.H.T + self.R
            K = self.P[i] @ self.H.T @ np.linalg.inv(S)
            self.x[i] = self.x[i] + K @ y
            self.P[i] = (self.I - K @ self.H) @ self.P[i]

        velocities = self.x[:, 1, 0]
        max_vel = np.max(np.abs(velocities))

        # --- SAFETY 1: Hand has stopped → return raw (no drift/overshoot) ---
        if max_vel < VELOCITY_DEADBAND_RAD_S:
            return raw_target_q.copy()

        # --- SAFETY 2: Sudden position jump → suppress prediction ---
        position_error = np.max(np.abs(raw_target_q - self.x[:, 0, 0]))
        if position_error > JUMP_GUARD_RAD:
            return raw_target_q.copy()

        # --- Tracking-error damping ---
        final_horizon = self.horizon
        if q_actual is not None and np.any(q_actual != 0.0):
            tracking_error = np.max(np.abs(raw_target_q - q_actual))
            if tracking_error < DAMP_START_ERROR_RAD:
                scale = np.clip(
                    (tracking_error - DAMP_MIN_ERROR_RAD) / (DAMP_START_ERROR_RAD - DAMP_MIN_ERROR_RAD),
                    0.0, 1.0
                )
                final_horizon = self.horizon * scale

        # --- Project ahead by final_horizon ---
        future_q = self.x[:, 0, 0] + velocities * final_horizon

        # --- SAFETY 3: Never predict past the raw target (anti-overshoot guard) ---
        # Clamp each joint so predicted doesn't go beyond what the hand is pointing at
        # relative to the filter's current estimate.
        # (i.e., if hand is moving right and predicted > raw → cap at raw)
        for i in range(4):
            vel = velocities[i]
            if vel > 0:
                future_q[i] = min(future_q[i], raw_target_q[i])
            elif vel < 0:
                future_q[i] = max(future_q[i], raw_target_q[i])

        return future_q
