# PID Inverted Pendulum Simulation

## Project Overview

This project demonstrates the design and implementation of a closed-loop control system applied to a classic cart-pole (inverted pendulum) configuration - an inherently unstable, nonlinear dynamic system. The pendulum initializes at a randomized angle, and a fully hand-implemented PID controller continuously computes and applies corrective forces to the cart in order to drive the pole to a stable upright equilibrium.

The objective was to develop a complete, production-quality simulation rather than a minimal proof-of-concept. To that end, the physics engine, control logic, real-time visualization, live signal plotting, interactive gain tuning, disturbance injection, and data logging were each implemented as distinct, modular components.

---

## What Was Built

This project includes the following components:

- A nonlinear physics engine modeling cart position, cart velocity, pendulum angle, and angular velocity
- RK4 and Euler numerical integration support for accurate state propagation
- A hand-implemented PID controller with no reliance on external control libraries
- Anti-windup logic and first-order derivative filtering embedded within the PID implementation
- A Pygame-based real-time animation of the cart-pole system
- A live Matplotlib signal dashboard displaying angle, angular rate, control output, and tracking error
- Slider-based interactive gain tuning (Kp, Ki, Kd) operable during active simulation
- Mid-run disturbance injection via force impulse, with automatic step-response analysis
- CSV state logging and JSON performance metrics output generated at the conclusion of every run

---

## Visual Preview

### Stable Control Response

Angle, angular rate, control output, and tracking error during successful stabilization and disturbance recovery:

![Stable run summary plot](/pid_pendulum/examples/stable_run.png)

### Unstable Control Response

Behavior under intentionally degraded PID tuning, showing loss of balance and divergence:

![Unstable run summary plot](/pid_pendulum/examples/unstable_run.png)

---

## Tools And Libraries

The following tools and libraries were used throughout this project:

- **Python 3** - primary language for the full simulation and control pipeline
- **NumPy** - vector mathematics, dynamics calculations, and numerical integration
- **Matplotlib** - real-time signal plotting and interactive gain slider interface
- **Pygame** - live rendering of the cart-pole animation
- **CSV / JSON** - persistent run logging and computed step-response metrics

All physics constants and controller parameters are managed through a JSON-driven configuration system, enabling parameter iteration without requiring any modification to the underlying source code.

---

## Repository Structure

```text
PID-Inverted-Pendulum-Simulation/
└── pid_pendulum/
    ├── main.py               # Entry point and simulation loop orchestration
    ├── physics.py            # Nonlinear dynamics and RK4/Euler integrators
    ├── pid.py                # PID controller with anti-windup and derivative filtering
    ├── visualizer.py         # Pygame rendering module
    ├── plotter.py            # Matplotlib live plots and gain sliders
    ├── logger.py             # CSV logging and step-response metrics computation
    ├── config.json           # Default stable parameter configuration
    ├── config_unstable.json  # Intentionally degraded tuning for comparative testing
    ├── requirements.txt
    ├── examples/
    └── logs/
```

---

## How The Control Loop Works

At each simulation time step, the following operations are performed in sequence:

1. The current state vector `(x, x_dot, theta, theta_dot)` is read from the physics model
2. The PID controller computes a corrective force using the instantaneous angle error
3. An optional disturbance impulse is applied at the configured trigger time
4. The nonlinear dynamics are integrated forward to produce the next system state
5. The animation and live signal plots are updated to reflect the current state
6. All state variables and PID internals are written to the active log file

This architecture ensures that the complete closed-loop behavior remains visible, measurable, and fully reproducible across all runs.

---

## PID Implementation Details

The PID controller was implemented entirely from first principles and incorporates several practical safeguards that are essential in deployed control systems but frequently omitted in academic implementations:

- **Proportional Term (P)** - provides immediate corrective response proportional to instantaneous error
- **Integral Term (I)** - accumulates error over time to eliminate steady-state bias
- **Derivative Term (D)** - introduces damping to reduce overshoot and improve transient response
- **Output Clamping** - constrains actuator force to physically realistic bounds
- **Integral Clamping** - prevents integrator windup during periods of sustained saturation
- **Back-Calculation Anti-Windup** - ensures graceful recovery from output saturation events
- **First-Order Derivative Filter** - attenuates high-frequency noise on the derivative channel to improve signal quality

---

## Disturbance Testing And Performance Metrics

Controller robustness is evaluated by injecting a short-duration force impulse mid-simulation. Following the disturbance, three quantitative performance metrics are automatically computed from the logged data:

- **Recovery Time** - elapsed time for the system to return within acceptable tolerance of the setpoint
- **Overshoot Percentage** - peak angular deviation relative to the target equilibrium
- **Steady-State Error** - residual tracking error following transient response decay

Each simulation run exports the following files:

```text
logs/run_<timestamp>.csv
logs/run_<timestamp>_metrics.json
```

### Example Results - Stable Configuration

| Metric | Value |
|---|---|
| Recovery Time | 0.14 s |
| Overshoot | 16.64% |
| Steady-State Error | 0.0028 deg |

---

## Summary

**PID Inverted Pendulum Simulation** is a complete, practical demonstration of applied feedback control - spanning nonlinear physics modeling, RK4 numerical integration, a production-quality PID implementation with full safeguard coverage, real-time visualization, quantitative disturbance robustness analysis, and structured data logging. The system is designed to be readable, modular, and extensible for future control experimentation.
