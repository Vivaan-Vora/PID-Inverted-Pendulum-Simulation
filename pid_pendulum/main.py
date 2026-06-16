"""Entry point for PID-controlled inverted pendulum simulation."""

from __future__ import annotations

import argparse
import json
import time
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, Dict

import numpy as np

from logger import SimulationLogger
from physics import PendulumParams, compute_lqr_gain, integrate
from pid import PIDConfig, PIDController
from plotter import PlotConfig, RealTimePlotter
from visualizer import PendulumVisualizer, VisualConfig


def load_config(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def build_initial_state(cfg: Dict[str, Any], rng: np.random.Generator) -> np.ndarray:
    init = cfg["initial_conditions"]

    if init.get("randomize_angle", True):
        lo, hi = init.get("angle_deg_range", [-12.0, 12.0])
        angle_deg = float(rng.uniform(lo, hi))
    else:
        angle_deg = float(init.get("angle_deg", 5.0))

    state = np.array(
        [
            float(init.get("cart_position_m", 0.0)),
            float(init.get("cart_velocity_mps", 0.0)),
            float(np.deg2rad(angle_deg)),
            float(np.deg2rad(init.get("angular_velocity_deg_s", 0.0))),
        ],
        dtype=float,
    )
    return state


def disturbance_force(sim_time: float, cfg: Dict[str, Any]) -> float:
    if not cfg.get("enabled", True):
        return 0.0
    t0 = float(cfg.get("time_s", 6.0))
    dt = float(cfg.get("duration_s", 0.08))
    amp = float(cfg.get("force_n", 10.0))
    if t0 <= sim_time < t0 + dt:
        return amp
    return 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PID inverted pendulum simulation")
    parser.add_argument("--config", type=str, default="config.json", help="Path to JSON config")
    parser.add_argument("--headless", action="store_true", help="Disable pygame + interactive plots")
    parser.add_argument("--duration", type=float, default=None, help="Override run duration [s]")
    parser.add_argument("--seed", type=int, default=None, help="Override random seed")
    parser.add_argument("--no-realtime", action="store_true", help="Run as fast as possible")
    parser.add_argument(
        "--save-plot",
        type=str,
        default=None,
        help="Save a static summary plot at the end of the run",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config_path = Path(args.config).resolve()
    cfg = load_config(config_path)

    sim_cfg = cfg["simulation"]
    dt = float(sim_cfg["dt"])
    duration = float(args.duration if args.duration is not None else sim_cfg["duration_s"])
    method = sim_cfg.get("integrator", "rk4")

    seed = int(args.seed if args.seed is not None else sim_cfg.get("seed", 11))
    rng = np.random.default_rng(seed)

    params = PendulumParams(**cfg["physics"])
    state = build_initial_state(cfg, rng)

    pid_cfg = PIDConfig(**cfg["pid"])
    controller = PIDController(pid_cfg)

    vis_cfg = VisualConfig(**cfg["visualization"]["pygame"])
    plot_cfg = PlotConfig(**cfg["visualization"]["plotter"])

    headless = bool(args.headless or sim_cfg.get("headless", False))
    plotter = RealTimePlotter(
        plot_cfg,
        initial_gains={"kp": pid_cfg.kp, "ki": pid_cfg.ki, "kd": pid_cfg.kd},
        enabled=not headless,
    )
    visualizer = PendulumVisualizer(vis_cfg, enabled=not headless)
    logger = SimulationLogger()

    # Optional LQR benchmark for comparison/debugging (not used as controller).
    lqr_gain = compute_lqr_gain(params)
    if lqr_gain is not None and cfg.get("reference_lqr", {}).get("enabled", False):
        print(f"LQR reference gain available: {np.array2string(lqr_gain, precision=3)}")

    disturbance_cfg = cfg["disturbance"]
    max_angle_deg = float(sim_cfg.get("max_angle_before_fall_deg", 75.0))
    stop_on_fall = bool(sim_cfg.get("stop_on_fall", False))

    sim_time = 0.0
    step_count = int(duration / dt)
    wall_start = time.perf_counter()

    for k in range(step_count):
        if not visualizer.process_events():
            print("Simulation stopped by window close.")
            break

        gains = plotter.get_slider_gains()
        controller.set_gains(gains["kp"], gains["ki"], gains["kd"])

        theta = state[2]
        theta_dot = state[3]
        u_pid = controller.compute(measurement=theta, dt=dt)
        d_force = disturbance_force(sim_time, disturbance_cfg)
        u_total = u_pid + d_force

        logger.log(
            t=sim_time,
            x=state[0],
            x_dot=state[1],
            theta=state[2],
            theta_dot=state[3],
            disturbance=d_force,
            pid_terms=controller.last_terms,
        )
        plotter.update(
            sim_time=sim_time,
            theta=theta,
            theta_dot=theta_dot,
            control=u_pid,
            error=controller.last_terms["error"],
        )
        visualizer.draw(state, control_force=u_total, sim_time=sim_time, gains=gains)

        state = integrate(state, u_total, dt=dt, params=params, method=method)
        sim_time = (k + 1) * dt

        if stop_on_fall and abs(np.degrees(state[2])) > max_angle_deg:
            print(f"Pendulum exceeded {max_angle_deg:.1f} deg. Stopping run.")
            break

        if not args.no_realtime and not headless:
            target = wall_start + sim_time
            sleep_time = target - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    visualizer.close()
    plotter.close()

    now = datetime.now(UTC).strftime("%Y%m%d_%H%M%S")
    logs_dir = (config_path.parent / cfg["logging"]["output_dir"]).resolve()
    csv_path = logs_dir / f"run_{now}.csv"
    metrics_path = logs_dir / f"run_{now}_metrics.json"
    logger.save_csv(csv_path)

    metrics = logger.compute_step_metrics(
        disturbance_time=float(disturbance_cfg.get("time_s", duration / 2.0)),
        disturbance_duration_s=float(disturbance_cfg.get("duration_s", 0.0)),
        settle_tolerance_rad=float(np.deg2rad(cfg["logging"].get("settle_tolerance_deg", 2.0))),
        settle_window_s=float(cfg["logging"].get("settle_window_s", 0.5)),
    )
    logger.save_metrics_json(metrics_path, metrics)

    if args.save_plot:
        logger.save_summary_plot(Path(args.save_plot).resolve(), title="Pendulum Run Summary")

    print(f"Run complete. CSV: {csv_path}")
    print(f"Metrics: {metrics_path}")
    print(
        "Step response => "
        f"recovery_time_s={metrics.recovery_time_s}, "
        f"overshoot_percent={metrics.overshoot_percent:.2f}, "
        f"steady_state_error_deg={metrics.steady_state_error_deg:.3f}"
    )


if __name__ == "__main__":
    main()
