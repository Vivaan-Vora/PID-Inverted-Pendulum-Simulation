"""Real-time matplotlib plotter + gain tuning sliders."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Dict, List

import numpy as np

try:
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider
except Exception:  # pragma: no cover - optional when running headless
    plt = None
    Slider = None


@dataclass
class PlotConfig:
    window_seconds: float = 10.0
    max_points: int = 1200


class RealTimePlotter:
    """Handles live state plotting and PID gain sliders."""

    def __init__(
        self,
        cfg: PlotConfig,
        initial_gains: Dict[str, float],
        enabled: bool = True,
    ) -> None:
        self.cfg = cfg
        self.enabled = bool(enabled and plt is not None and Slider is not None)
        self.gains = dict(initial_gains)

        self.time_data: deque[float] = deque(maxlen=cfg.max_points)
        self.theta_data: deque[float] = deque(maxlen=cfg.max_points)
        self.theta_dot_data: deque[float] = deque(maxlen=cfg.max_points)
        self.control_data: deque[float] = deque(maxlen=cfg.max_points)
        self.error_data: deque[float] = deque(maxlen=cfg.max_points)

        self.fig = None
        self.axs = None
        self.lines: List = []
        self.slider_kp = None
        self.slider_ki = None
        self.slider_kd = None

        if self.enabled:
            plt.ion()
            self.fig, self.axs = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
            self.fig.suptitle("Inverted Pendulum Closed-Loop State")
            plt.subplots_adjust(left=0.1, right=0.97, top=0.93, bottom=0.22, hspace=0.35)

            labels = ["Angle [deg]", "Angular rate [deg/s]", "Control [N]", "Error [deg]"]
            colors = ["tab:red", "tab:orange", "tab:blue", "tab:green"]
            for ax, label, color in zip(self.axs, labels, colors):
                (line,) = ax.plot([], [], color=color, linewidth=1.8)
                self.lines.append(line)
                ax.grid(alpha=0.3)
                ax.set_ylabel(label)
            self.axs[-1].set_xlabel("Time [s]")

            ax_kp = self.fig.add_axes([0.12, 0.13, 0.78, 0.03])
            ax_ki = self.fig.add_axes([0.12, 0.09, 0.78, 0.03])
            ax_kd = self.fig.add_axes([0.12, 0.05, 0.78, 0.03])
            self.slider_kp = Slider(ax_kp, "Kp", 0.0, 120.0, valinit=self.gains["kp"])
            self.slider_ki = Slider(ax_ki, "Ki", 0.0, 60.0, valinit=self.gains["ki"])
            self.slider_kd = Slider(ax_kd, "Kd", 0.0, 30.0, valinit=self.gains["kd"])

    def update(self, sim_time: float, theta: float, theta_dot: float, control: float, error: float) -> None:
        self.time_data.append(sim_time)
        self.theta_data.append(np.degrees(theta))
        self.theta_dot_data.append(np.degrees(theta_dot))
        self.control_data.append(control)
        self.error_data.append(np.degrees(error))

        if not self.enabled or self.fig is None or self.axs is None:
            return

        t = np.array(self.time_data, dtype=float)
        y_series = [
            np.array(self.theta_data, dtype=float),
            np.array(self.theta_dot_data, dtype=float),
            np.array(self.control_data, dtype=float),
            np.array(self.error_data, dtype=float),
        ]

        if t.size < 2:
            return

        for ax, line, y in zip(self.axs, self.lines, y_series):
            line.set_data(t, y)
            t_min = max(0.0, sim_time - self.cfg.window_seconds)
            ax.set_xlim(t_min, t_min + self.cfg.window_seconds)
            y_pad = max(1e-3, 0.1 * (np.max(y) - np.min(y) + 1e-6))
            ax.set_ylim(np.min(y) - y_pad, np.max(y) + y_pad)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def get_slider_gains(self) -> Dict[str, float]:
        if self.enabled and self.slider_kp and self.slider_ki and self.slider_kd:
            self.gains["kp"] = float(self.slider_kp.val)
            self.gains["ki"] = float(self.slider_ki.val)
            self.gains["kd"] = float(self.slider_kd.val)
        return dict(self.gains)

    def close(self) -> None:
        if self.enabled and plt is not None:
            plt.ioff()
            plt.close(self.fig)
