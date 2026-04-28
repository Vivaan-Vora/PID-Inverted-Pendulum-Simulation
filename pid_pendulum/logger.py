"""CSV/JSON logging and step-response metric computation."""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List

import numpy as np

try:
    import matplotlib.pyplot as plt
except Exception:  # pragma: no cover
    plt = None


@dataclass
class StepResponseMetrics:
    recovery_time_s: float | None
    overshoot_percent: float
    steady_state_error_rad: float
    steady_state_error_deg: float


class SimulationLogger:
    def __init__(self) -> None:
        self.rows: List[Dict[str, float]] = []

    def log(
        self,
        t: float,
        x: float,
        x_dot: float,
        theta: float,
        theta_dot: float,
        disturbance: float,
        pid_terms: Dict[str, float],
    ) -> None:
        self.rows.append(
            {
                "time_s": float(t),
                "x_m": float(x),
                "x_dot_mps": float(x_dot),
                "theta_rad": float(theta),
                "theta_deg": float(np.degrees(theta)),
                "theta_dot_radps": float(theta_dot),
                "theta_dot_degps": float(np.degrees(theta_dot)),
                "disturbance_force_n": float(disturbance),
                "error_rad": float(pid_terms["error"]),
                "error_deg": float(np.degrees(pid_terms["error"])),
                "p_term": float(pid_terms["p_term"]),
                "i_term": float(pid_terms["i_term"]),
                "d_term": float(pid_terms["d_term"]),
                "control_output_n": float(pid_terms["output"]),
            }
        )

    def save_csv(self, path: Path) -> None:
        if not self.rows:
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)

    def compute_step_metrics(
        self,
        disturbance_time: float,
        disturbance_duration_s: float = 0.0,
        settle_tolerance_rad: float = np.deg2rad(2.0),
        settle_window_s: float = 0.5,
    ) -> StepResponseMetrics:
        if not self.rows:
            return StepResponseMetrics(None, 0.0, 0.0, 0.0)

        t = np.array([r["time_s"] for r in self.rows], dtype=float)
        e = np.array([r["error_rad"] for r in self.rows], dtype=float)
        idx = np.where(t >= disturbance_time)[0]
        if idx.size == 0:
            return StepResponseMetrics(None, 0.0, float(np.mean(np.abs(e[-20:]))), float(np.degrees(np.mean(np.abs(e[-20:])))))

        disturbance_end = disturbance_time + max(0.0, disturbance_duration_s)
        during_mask = (t >= disturbance_time) & (t <= disturbance_end + 1e-12)
        after_mask = t >= disturbance_end
        post_t = t[after_mask]
        post_e = np.abs(e[after_mask])
        if post_e.size == 0:
            post_t = t[idx[0] :]
            post_e = np.abs(e[idx[0] :])

        if np.any(during_mask):
            peak_during = float(np.max(np.abs(e[during_mask])))
        else:
            peak_during = float(post_e[0])
        peak_after = float(np.max(post_e))
        denom = max(peak_during, 1e-6)
        overshoot = max(0.0, (peak_after - peak_during) / denom * 100.0)

        dt = float(np.mean(np.diff(t))) if t.size > 1 else 0.01
        window_n = max(1, int(settle_window_s / max(dt, 1e-6)))
        recovery_time = None
        for i in range(0, len(post_e) - window_n + 1):
            if np.all(post_e[i : i + window_n] <= settle_tolerance_rad):
                recovery_time = float(post_t[i] - disturbance_end)
                break

        tail_n = max(window_n, min(100, len(post_e)))
        steady_state = float(np.mean(post_e[-tail_n:]))

        return StepResponseMetrics(
            recovery_time_s=recovery_time,
            overshoot_percent=float(overshoot),
            steady_state_error_rad=steady_state,
            steady_state_error_deg=float(np.degrees(steady_state)),
        )

    def save_metrics_json(self, path: Path, metrics: StepResponseMetrics) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as f:
            json.dump(asdict(metrics), f, indent=2)

    def save_summary_plot(self, path: Path, title: str = "Pendulum Run Summary") -> None:
        if plt is None or not self.rows:
            return

        t = np.array([r["time_s"] for r in self.rows], dtype=float)
        theta = np.array([r["theta_deg"] for r in self.rows], dtype=float)
        theta_dot = np.array([r["theta_dot_degps"] for r in self.rows], dtype=float)
        u = np.array([r["control_output_n"] for r in self.rows], dtype=float)
        err = np.array([r["error_deg"] for r in self.rows], dtype=float)

        fig, axs = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
        fig.suptitle(title)
        axs[0].plot(t, theta, color="tab:red")
        axs[0].set_ylabel("theta [deg]")
        axs[1].plot(t, theta_dot, color="tab:orange")
        axs[1].set_ylabel("theta dot [deg/s]")
        axs[2].plot(t, u, color="tab:blue")
        axs[2].set_ylabel("u [N]")
        axs[3].plot(t, err, color="tab:green")
        axs[3].set_ylabel("error [deg]")
        axs[3].set_xlabel("time [s]")
        for ax in axs:
            ax.grid(alpha=0.3)
        fig.tight_layout(rect=[0, 0.02, 1, 0.97])
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=150)
        plt.close(fig)
