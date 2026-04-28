"""Nonlinear inverted pendulum physics and integrators."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

try:
    from scipy import linalg
except Exception:  # pragma: no cover - optional dependency
    linalg = None


@dataclass
class PendulumParams:
    """Physical parameters for a cart-pole system."""

    gravity: float = 9.81
    cart_mass: float = 1.0
    pole_mass: float = 0.2
    pole_length: float = 0.5
    cart_friction: float = 0.05


def cartpole_dynamics(
    state: np.ndarray, force: float, params: PendulumParams
) -> np.ndarray:
    """Return [x_dot, x_ddot, theta_dot, theta_ddot] for a state.

    State order is [x, x_dot, theta, theta_dot], where theta=0 is upright.
    """

    x, x_dot, theta, theta_dot = state
    del x  # position is not used directly in this model

    total_mass = params.cart_mass + params.pole_mass
    polemass_length = params.pole_mass * params.pole_length
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)

    friction_force = -params.cart_friction * x_dot
    net_force = force + friction_force

    temp = (net_force + polemass_length * theta_dot**2 * sin_theta) / total_mass
    denom = params.pole_length * (4.0 / 3.0 - params.pole_mass * cos_theta**2 / total_mass)

    theta_ddot = (params.gravity * sin_theta - cos_theta * temp) / denom
    x_ddot = temp - polemass_length * theta_ddot * cos_theta / total_mass

    return np.array([x_dot, x_ddot, theta_dot, theta_ddot], dtype=float)


def euler_step(state: np.ndarray, force: float, dt: float, params: PendulumParams) -> np.ndarray:
    """Euler integration."""

    return state + dt * cartpole_dynamics(state, force, params)


def rk4_step(state: np.ndarray, force: float, dt: float, params: PendulumParams) -> np.ndarray:
    """Classic RK4 integration."""

    k1 = cartpole_dynamics(state, force, params)
    k2 = cartpole_dynamics(state + 0.5 * dt * k1, force, params)
    k3 = cartpole_dynamics(state + 0.5 * dt * k2, force, params)
    k4 = cartpole_dynamics(state + dt * k3, force, params)
    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def integrate(
    state: np.ndarray,
    force: float,
    dt: float,
    params: PendulumParams,
    method: Literal["euler", "rk4"] = "rk4",
) -> np.ndarray:
    """Integrate one timestep using the chosen method."""

    if method == "euler":
        return euler_step(state, force, dt, params)
    return rk4_step(state, force, dt, params)


def compute_lqr_gain(params: PendulumParams) -> np.ndarray | None:
    """Optional continuous-time LQR gain for comparison.

    Returns a gain matrix K for u = -Kx on state [x, x_dot, theta, theta_dot].
    """

    if linalg is None:
        return None

    g = params.gravity
    m = params.pole_mass
    M = params.cart_mass
    l = params.pole_length

    A = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, -(m * g) / M, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, ((M + m) * g) / (M * l), 0.0],
        ]
    )
    B = np.array([[0.0], [1.0 / M], [0.0], [-1.0 / (M * l)]])

    Q = np.diag([0.5, 0.1, 12.0, 0.8])
    R = np.array([[0.2]])

    p = linalg.solve_continuous_are(A, B, Q, R)
    k = np.linalg.inv(R) @ B.T @ p
    return k
