# PID Controller Simulation For A Self-Balancing Inverted Pendulum

## Project Overview

I built this project to demonstrate how I design a closed-loop control system from scratch for an unstable nonlinear system. The system is a classic cart-pole (inverted pendulum) setup where the pendulum starts at a random angle, and my PID controller continuously drives the cart to stabilize the pole upright.

My goal was to make this a complete, practical control project rather than a toy script. I implemented the physics model, control logic, live visualization, real-time plotting, gain tuning interface, disturbance testing, and data logging as separate modules.

## What I Built

This project includes:

- A nonlinear physics engine for cart position, cart velocity, pendulum angle, and angular velocity
- RK4 and Euler integration support for state propagation
- A hand-built PID controller (no control library shortcuts)
- Anti-windup logic and derivative filtering inside the PID implementation
- A `pygame` animation of the cart and pendulum in real time
- A live `matplotlib` dashboard for angle, angular rate, control output, and error
- Slider-based gain tuning (`Kp`, `Ki`, `Kd`) while the simulation is running
- Mid-run disturbance injection and automatic step-response analysis
- CSV state logging and JSON metrics output for every run

## What I Used

I used the following tools and libraries:

- **Python 3** for the full simulation and control pipeline
- **NumPy** for vector math, dynamics calculations, and numerical integration
- **Matplotlib** for real-time signal plotting and interactive gain sliders
- **Pygame** for live rendering of the cart-pole scene
- **SciPy (Optional)** for reference LQR gain computation comparison
- **CSV / JSON** for persistent run logs and computed response metrics

I also used a JSON-driven configuration approach so I can tune physics and controller parameters without changing source code.

## Repository Structure

```text
pid_pendulum/
  main.py              # entry point and simulation loop orchestration
  physics.py           # nonlinear dynamics + RK4/Euler integrators
  pid.py               # PID controller with anti-windup and filtering
  visualizer.py        # pygame rendering
  plotter.py           # matplotlib live plots and gain sliders
  logger.py            # CSV logging and step-response metrics
  config.json          # default (stable) parameters
  config_unstable.json # intentionally poor tuning for comparison
  requirements.txt
  examples/
  logs/
```

## How The Control Loop Works

At each simulation time step, I:

1. Read the current state (`x`, `x_dot`, `theta`, `theta_dot`)
2. Compute control force from my PID controller using the angle error
3. Apply an optional disturbance impulse at the configured time
4. Integrate the nonlinear dynamics to get the next state
5. Render and plot the new state in real time
6. Log state variables and PID internals to disk

This makes the full closed-loop behavior visible and measurable from start to finish.

## PID Implementation Details

I implemented the PID terms manually and included practical safeguards:

- **Proportional Term (`P`)** for immediate error correction
- **Integral Term (`I`)** for bias removal over time
- **Derivative Term (`D`)** for damping and overshoot reduction
- **Output Limits** to bound actuator force
- **Integral Clamping** to prevent integral runaway
- **Back-Calculation Anti-Windup** when output saturation occurs
- **First-Order Derivative Filtering** to reduce derivative noise

## Disturbance Testing And Metrics

To evaluate recovery behavior, I inject a short force impulse mid-run. From logged data, I compute:

- recovery time,
- overshoot percentage,
- steady-state error.

Each run exports:

- `logs/run_<timestamp>.csv`
- `logs/run_<timestamp>_metrics.json`

## How To Run

### Install Dependencies

```bash
cd pid_pendulum
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Run Interactive Simulation

```bash
cd pid_pendulum
python3 main.py --config config.json
```

### Run Headless Simulation

```bash
cd pid_pendulum
python3 main.py --config config.json --headless --no-realtime
```

### Generate Stable Vs Unstable Example Plots

```bash
cd pid_pendulum

# Stable behavior
python3 main.py --config config.json --headless --no-realtime --duration 10 --save-plot examples/stable_run.png

# Intentionally unstable behavior
python3 main.py --config config_unstable.json --headless --no-realtime --duration 10 --save-plot examples/unstable_run.png
```

## Example Stable-Run Metrics

From my stable configuration:

- recovery time: `0.14 s`
- overshoot: `16.64%`
- steady-state error: `0.0028 deg`

## What I Learned From This Project

Building this end-to-end reinforced a few key control and simulation lessons for me:

1. **Numerical Stability Matters**  
   Integrator choice and time step size significantly affect controller behavior, especially in nonlinear systems.

2. **Real PID Is More Than Three Terms**  
   Anti-windup and derivative filtering are essential in practical control loops; without them, tuning becomes fragile.

3. **Visualization Speeds Up Debugging**  
   Seeing animation and live traces together made it much easier for me to diagnose oscillation, saturation, and poor damping.

4. **Disturbance Metrics Improve Tuning Decisions**  
   Quantitative metrics (recovery time, overshoot, steady-state error) were more useful than visual judgment alone.

5. **Configuration-Driven Design Improves Iteration Speed**  
   Keeping parameters in JSON allowed me to test many scenarios quickly without touching the core code.

## Summary

I built this project as a complete demonstration of applied feedback control: from nonlinear modeling to real-time stabilization, tuning, and performance analysis. It is designed to be readable, testable, and easy to extend for future control experiments.

