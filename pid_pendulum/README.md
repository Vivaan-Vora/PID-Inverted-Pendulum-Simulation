# PID Inverted Pendulum Simulation

## Project Overview

This project demonstrates the design and implementation of a closed-loop control system applied to a classic cart-pole (inverted pendulum) configuration - an inherently unstable, nonlinear dynamic system. The pendulum initializes at a randomized angle, and a fully hand-implemented PID controller continuously computes and applies corrective forces to the cart in order to drive the pole to a stable upright equilibrium.

The objective was to develop a complete, production-quality simulation rather than a minimal proof-of-concept. To that end, the physics engine, control logic, real-time visualization, live signal plotting, interactive gain tuning, disturbance injection, and data logging were each implemented as distinct, modular components.

---

## Features

- Physics for the cart and pendulum (position, speed, angle)
- A PID controller I wrote myself in Python (no control libraries)
- Live animation with **Pygame**
- Live graphs with **Matplotlib** (angle, speed, force, error)
- Sliders to tune Kp, Ki, and Kd during the run
- Ability to give the cart a push mid-run and see if it recovers
- Saves each run to CSV/JSON so I can look at the results later

---

## Preview

### Stable run

When the gains are tuned well, the pendulum stays up and recovers after a push:

![Stable run](examples/stable_run.png)

### Unstable run

When the gains are off, it falls over:

![Unstable run](examples/unstable_run.png)

---

## Tools I Used

Based on what I already use in school and other projects:

- **Python** — main language for the whole project
- **NumPy** — math and physics calculations
- **Matplotlib** — live graphs and gain sliders
- **Pygame** — cart-and-pendulum animation
- **Git** — version control
- **CSV / JSON** — saving run data and settings

Settings (physics values and PID gains) live in JSON files, so I can try different setups without changing the code.

---

## Project Layout

```text
pid_pendulum/
├── main.py               # starts the simulation
├── physics.py            # cart + pendulum physics
├── pid.py                # PID controller
├── visualizer.py         # Pygame animation
├── plotter.py            # Matplotlib graphs + sliders
├── logger.py             # saves run data
├── config.json           # normal (stable) settings
├── config_unstable.json  # bad gains on purpose
├── requirements.txt
├── examples/
└── logs/
```

---

## How It Runs

Each step of the simulation does this:

1. Read where the cart and pendulum are
2. Have the PID decide how hard to push the cart
3. Optionally add a small push (disturbance)
4. Update the physics
5. Update the animation and graphs
6. Log the data

---

## How To Run

```bash
pip install -r requirements.txt
python main.py
```

Use `config_unstable.json` if you want to see it fail on purpose.

---

## What I Learned

- How P, I, and D each affect balance and recovery
- Why bad gains make the system overshoot or fall
- How to connect a controller to a live animation and graphs
- How to test a controller by disturbing it and measuring recovery

This project helped me connect class math (like ODEs and linear algebra) to something visual I could tune by hand.
