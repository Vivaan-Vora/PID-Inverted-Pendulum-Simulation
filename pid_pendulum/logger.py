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
