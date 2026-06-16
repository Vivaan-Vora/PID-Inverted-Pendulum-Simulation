"""Manual PID controller implementation with practical features."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np


@dataclass
class PIDConfig:
    kp: float
    ki: float
    kd: float
    setpoint: float = 0.0
    output_limits: Tuple[float, float] = (-25.0, 25.0)
    integral_limits: Tuple[float, float] = (-10.0, 10.0)
    derivative_filter_tau: float = 0.03
    anti_windup_gain: float = 0.6


class PIDController:
    """PID controller with anti-windup and derivative filtering."""

    def __init__(self, cfg: PIDConfig) -> None:
        self.cfg = cfg
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.filtered_derivative = 0.0
        self.initialized = False
        self.last_terms: Dict[str, float] = {
            "error": 0.0,
            "p_term": 0.0,
            "i_term": 0.0,
            "d_term": 0.0,
            "output": 0.0,
        }

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        self.cfg.kp = float(kp)
        self.cfg.ki = float(ki)
        self.cfg.kd = float(kd)

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.filtered_derivative = 0.0
        self.initialized = False

    def compute(self, measurement: float, dt: float) -> float:
        if dt <= 0.0:
            raise ValueError("dt must be > 0")

        # For this model we define positive error when measured angle is above setpoint.
        error = measurement - self.cfg.setpoint

        if not self.initialized:
            self.prev_error = error
            self.prev_measurement = measurement
            self.initialized = True

        # Derivative on measurement (same sign convention as error above).
        raw_derivative = (measurement - self.prev_measurement) / dt
        alpha = self.cfg.derivative_filter_tau / (self.cfg.derivative_filter_tau + dt)
        self.filtered_derivative = alpha * self.filtered_derivative + (1.0 - alpha) * raw_derivative

        p_term = self.cfg.kp * error
        d_term = self.cfg.kd * self.filtered_derivative

        self.integral += error * dt
        self.integral = float(
            np.clip(self.integral, self.cfg.integral_limits[0], self.cfg.integral_limits[1])
        )
        i_term = self.cfg.ki * self.integral

        unsat = p_term + i_term + d_term
        output = float(np.clip(unsat, self.cfg.output_limits[0], self.cfg.output_limits[1]))

        # Back-calculation anti-windup: pull integrator toward saturated output.
        if self.cfg.ki != 0.0:
            correction = (output - unsat) * self.cfg.anti_windup_gain / self.cfg.ki
            self.integral += correction * dt
            self.integral = float(
                np.clip(self.integral, self.cfg.integral_limits[0], self.cfg.integral_limits[1])
            )
            i_term = self.cfg.ki * self.integral
            unsat = p_term + i_term + d_term
            output = float(np.clip(unsat, self.cfg.output_limits[0], self.cfg.output_limits[1]))

        self.prev_error = error
        self.prev_measurement = measurement

        self.last_terms = {
            "error": float(error),
            "p_term": float(p_term),
            "i_term": float(i_term),
            "d_term": float(d_term),
            "output": float(output),
        }
        return output
