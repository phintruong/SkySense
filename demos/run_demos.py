#!/usr/bin/env python
"""SkySense GNC failure recovery demos."""

from __future__ import annotations

import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from main import Simulation
from src.telemetry.plotter import (
    plot_altitude,
    plot_attitude,
    plot_control_output,
    plot_ekf_health,
    plot_position_error,
    plot_sensor_status,
    plot_trajectory_3d,
)

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")


def save_plots(name: str, log: dict, events: list[tuple[float, str]] | None = None) -> None:
    """Generate and save all standard plots for a demo."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    prefix = os.path.join(OUTPUT_DIR, name)

    plots = {
        "altitude": plot_altitude(log, events),
        "attitude": plot_attitude(log, events),
        "control": plot_control_output(log),
        "ekf_health": plot_ekf_health(log, events),
        "sensors": plot_sensor_status(log, events),
        "trajectory_3d": plot_trajectory_3d(log),
        "position_error": plot_position_error(log, events),
    }

    for suffix, fig in plots.items():
        fig.savefig(f"{prefix}_{suffix}.png", dpi=150, bbox_inches="tight")
        plt.close(fig)

    print(f"  Saved {len(plots)} plots to {OUTPUT_DIR}/{name}_*.png")


def demo_hover() -> None:
    """Baseline hover at 2 m for 10 seconds."""
    print("\n[1/5] Hover Stability")
    sim = Simulation()
    sim.run(duration=10.0)
    log = sim.logger.get_log()

    final_pos = sim.drone.state.position
    target = sim.target_position
    error = np.linalg.norm(final_pos - target)
    print(f"  Final error: {error:.4f} m")

    save_plots("hover", log)


def demo_disturbance() -> None:
    """Inject a 5 N wind gust after the hover has settled."""
    print("\n[2/5] Disturbance Rejection")
    sim = Simulation()
    events: list[tuple[float, str]] = []

    sim.run(duration=3.0)
    sim.inject_disturbance(np.array([5.0, 0.0, 0.0]), duration=0.5)
    events.append((3.0, "5 N Wind Gust"))
    sim.run(duration=7.0)

    save_plots("disturbance", sim.logger.get_log(), events)


def demo_ultrasonic_dropout() -> None:
    """Disable ultrasonic altitude updates, then recover the sensor."""
    print("\n[3/5] Ultrasonic Dropout and Recovery")
    sim = Simulation()
    events: list[tuple[float, str]] = []

    sim.run(duration=3.0)
    sim.inject_sensor_failure("ultrasonic", "off")
    events.append((3.0, "Ultrasonic OFF"))

    sim.run(duration=5.0)
    sim.inject_sensor_failure("ultrasonic", "recover")
    events.append((8.0, "Ultrasonic Recover"))

    sim.run(duration=4.0)
    save_plots("ultrasonic_dropout", sim.logger.get_log(), events)


def demo_accel_degradation() -> None:
    """Increase accelerometer noise and show EKF resilience."""
    print("\n[4/5] Accelerometer Degradation")
    sim = Simulation()
    events: list[tuple[float, str]] = []

    sim.run(duration=3.0)
    sim.inject_sensor_failure("accel", "noisy")
    events.append((3.0, "Accel 10x Noise"))
    sim.run(duration=7.0)

    save_plots("accel_degradation", sim.logger.get_log(), events)


def demo_imu_failure() -> None:
    """Trigger a full IMU failure and emergency descent."""
    print("\n[5/5] Full IMU Failure to Emergency Descent")
    sim = Simulation()
    events: list[tuple[float, str]] = []

    sim.run(duration=3.0)
    sim.inject_sensor_failure("imu", "off")
    events.append((3.0, "IMU Failure"))
    sim.run(duration=7.0)

    save_plots("imu_failure", sim.logger.get_log(), events)


def main() -> None:
    print("=" * 50)
    print("SkySense GNC Failure Recovery Demos")
    print("=" * 50)

    demo_hover()
    demo_disturbance()
    demo_ultrasonic_dropout()
    demo_accel_degradation()
    demo_imu_failure()

    print(f"\nAll demos complete. Plots saved to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
