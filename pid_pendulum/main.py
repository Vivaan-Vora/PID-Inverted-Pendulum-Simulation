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
