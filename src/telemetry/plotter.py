"""Plotting helpers for SkySense GNC telemetry demos."""

from __future__ import annotations

from collections.abc import Sequence

import matplotlib
import numpy as np

EventList = Sequence[tuple[float, str]] | None

matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.style.use("dark_background")

TRUE_COLOR = "#4BA3F2"
EST_COLOR = "#FFB05C"
TARGET_COLOR = "#B8B8B8"
EVENT_COLOR = "#FF5A5F"
GREEN = "#4ADE80"
RED = "#F87171"
AXIS_COLORS = ("#4BA3F2", "#4ADE80", "#C084FC")
MOTOR_COLORS = ("#4BA3F2", "#4ADE80", "#FFB05C", "#F87171")


def _time(log: dict) -> np.ndarray:
    return np.asarray(log["time"], dtype=float)


def _style_axis(ax, xlabel: str | None = None, ylabel: str | None = None) -> None:
    if xlabel:
        ax.set_xlabel(xlabel)
    if ylabel:
        ax.set_ylabel(ylabel)
    ax.grid(True, color="#2F3744", linewidth=0.8, alpha=0.75)
    ax.tick_params(colors="#D8DEE9")
    for spine in ax.spines.values():
        spine.set_color("#4B5563")


def _mark_events(ax, events: EventList) -> None:
    if not events:
        return

    ymin, ymax = ax.get_ylim()
    y_text = ymin + 0.94 * (ymax - ymin)
    for event_time, label in events:
        ax.axvline(event_time, color=EVENT_COLOR, linestyle="--", linewidth=1.2, alpha=0.9)
        ax.text(
            event_time,
            y_text,
            label,
            color=EVENT_COLOR,
            fontsize=9,
            rotation=90,
            va="top",
            ha="right",
            backgroundcolor="#10141C",
        )


def _finalize(fig: plt.Figure, title: str) -> plt.Figure:
    fig.suptitle(title, fontsize=16, fontweight="bold", color="#F3F4F6")
    fig.tight_layout()
    return fig


def plot_altitude(log: dict, events: EventList = None) -> plt.Figure:
    """
    Plot positive-up altitude over time for true and estimated state.

    The sim default hover target is 2 m, equivalent to NED z = -2 m.
    """
    t = _time(log)
    true_altitude = -np.asarray(log["true_position"])[:, 2]
    est_altitude = -np.asarray(log["est_position"])[:, 2]

    fig, ax = plt.subplots(figsize=(12, 6))
    ax.plot(t, true_altitude, color=TRUE_COLOR, linewidth=2.0, label="True altitude")
    ax.plot(t, est_altitude, color=EST_COLOR, linestyle="--", linewidth=2.0, label="Estimated altitude")
    ax.axhline(2.0, color=TARGET_COLOR, linestyle="--", linewidth=1.4, label="Target 2.0 m")
    _style_axis(ax, "Time (s)", "Altitude (m)")
    _mark_events(ax, events)
    ax.legend(loc="best", frameon=True, facecolor="#151A24", edgecolor="#4B5563")
    return _finalize(fig, "Altitude Tracking")


def plot_attitude(log: dict, events: EventList = None) -> plt.Figure:
    """Plot roll, pitch, and yaw in degrees."""
    t = _time(log)
    true_euler = np.degrees(np.asarray(log["true_euler"]))
    est_euler = np.degrees(np.asarray(log["est_euler"]))

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    labels = ("Roll", "Pitch", "Yaw")
    for i, ax in enumerate(axes):
        ax.plot(t, true_euler[:, i], color=TRUE_COLOR, linewidth=1.8, label=f"True {labels[i]}")
        ax.plot(
            t,
            est_euler[:, i],
            color=EST_COLOR,
            linestyle="--",
            linewidth=1.8,
            label=f"Estimated {labels[i]}",
        )
        _style_axis(ax, ylabel=f"{labels[i]} (deg)")
        _mark_events(ax, events)
        ax.legend(loc="best", frameon=True, facecolor="#151A24", edgecolor="#4B5563")
    axes[-1].set_xlabel("Time (s)")
    return _finalize(fig, "Attitude Tracking")


def plot_trajectory_3d(log: dict) -> plt.Figure:
    """Plot true and estimated NED position as North, East, altitude."""
    true_pos = np.asarray(log["true_position"])
    est_pos = np.asarray(log["est_position"])

    fig = plt.figure(figsize=(12, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(true_pos[:, 0], true_pos[:, 1], -true_pos[:, 2], color=TRUE_COLOR, linewidth=2.0, label="True path")
    ax.plot(
        est_pos[:, 0],
        est_pos[:, 1],
        -est_pos[:, 2],
        color=EST_COLOR,
        linestyle="--",
        linewidth=2.0,
        label="Estimated path",
    )
    ax.scatter(true_pos[0, 0], true_pos[0, 1], -true_pos[0, 2], color=GREEN, s=55, label="Start")
    ax.scatter(true_pos[-1, 0], true_pos[-1, 1], -true_pos[-1, 2], color=RED, s=55, label="End")
    ax.set_xlabel("North (m)")
    ax.set_ylabel("East (m)")
    ax.set_zlabel("Altitude (m)")
    ax.grid(True, color="#2F3744", linewidth=0.8, alpha=0.75)
    ax.legend(loc="best", frameon=True, facecolor="#151A24", edgecolor="#4B5563")
    return _finalize(fig, "3D Trajectory")


def plot_control_output(log: dict) -> plt.Figure:
    """Plot thrust, body torques, motor commands, and actual motor thrusts."""
    t = _time(log)
    thrust = np.asarray(log["thrust"])
    torques = np.asarray(log["torques"])
    motor_commands = np.asarray(log["motor_commands"])
    motor_actual = np.asarray(log["motor_actual"])

    fig, axes = plt.subplots(2, 2, figsize=(12, 10), sharex=True)
    ax = axes[0, 0]
    ax.plot(t, thrust, color=TRUE_COLOR, linewidth=2.0)
    _style_axis(ax, "Time (s)", "Thrust (N)")
    ax.set_title("Total thrust")

    ax = axes[0, 1]
    for i, label in enumerate(("Roll", "Pitch", "Yaw")):
        ax.plot(t, torques[:, i], color=AXIS_COLORS[i], linewidth=1.7, label=label)
    _style_axis(ax, "Time (s)", "Torque (N m)")
    ax.set_title("Body torques")
    ax.legend(loc="best", frameon=True, facecolor="#151A24", edgecolor="#4B5563")

    ax = axes[1, 0]
    for i in range(4):
        ax.plot(t, motor_commands[:, i], color=MOTOR_COLORS[i], linewidth=1.6, label=f"Motor {i + 1}")
    _style_axis(ax, "Time (s)", "Command (N)")
    ax.set_title("Motor commands")
    ax.legend(loc="best", frameon=True, facecolor="#151A24", edgecolor="#4B5563")

    ax = axes[1, 1]
    for i in range(4):
        ax.plot(t, motor_actual[:, i], color=MOTOR_COLORS[i], linewidth=1.6, label=f"Motor {i + 1}")
    _style_axis(ax, "Time (s)", "Actual thrust (N)")
    ax.set_title("Motor actual")
    ax.legend(loc="best", frameon=True, facecolor="#151A24", edgecolor="#4B5563")

    return _finalize(fig, "Control Outputs")


def plot_ekf_health(log: dict, events: EventList = None) -> plt.Figure:
    """Plot EKF innovation norm and covariance trace."""
    t = _time(log)
    innovation = np.asarray(log["innovation_norm"])
    covariance = np.asarray(log["covariance_trace"])

    fig, axes = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    axes[0].plot(t, innovation, color=TRUE_COLOR, linewidth=2.0, label="Innovation norm")
    _style_axis(axes[0], ylabel="Innovation")
    _mark_events(axes[0], events)
    axes[0].legend(loc="best", frameon=True, facecolor="#151A24", edgecolor="#4B5563")

    axes[1].plot(t, np.maximum(covariance, 1e-12), color=EST_COLOR, linewidth=2.0, label="Covariance trace")
    axes[1].set_yscale("log")
    _style_axis(axes[1], "Time (s)", "Covariance trace")
    _mark_events(axes[1], events)
    axes[1].legend(loc="best", frameon=True, facecolor="#151A24", edgecolor="#4B5563")

    return _finalize(fig, "EKF Health")


def plot_sensor_status(log: dict, events: EventList = None) -> plt.Figure:
    """Plot IMU and ultrasonic health as binary timelines."""
    t = _time(log)
    imu = np.asarray(log["imu_healthy"], dtype=float)
    ultrasonic = np.asarray(log["ultrasonic_healthy"], dtype=float)

    fig, ax = plt.subplots(figsize=(12, 6))
    ax.step(t, imu + 1.1, where="post", color=GREEN, linewidth=2.2, label="IMU healthy")
    ax.fill_between(t, 1.1, imu + 1.1, step="post", color=GREEN, alpha=0.22)
    ax.step(t, ultrasonic, where="post", color=GREEN, linewidth=2.2, label="Ultrasonic healthy")
    ax.fill_between(t, 0.0, ultrasonic, step="post", color=GREEN, alpha=0.22)

    ax.step(t, (1.0 - imu) + 1.1, where="post", color=RED, linewidth=1.6, alpha=0.8, label="Failed")
    ax.step(t, 1.0 - ultrasonic, where="post", color=RED, linewidth=1.6, alpha=0.8)
    ax.set_yticks([0.5, 1.6])
    ax.set_yticklabels(["Ultrasonic", "IMU"])
    ax.set_ylim(-0.15, 2.25)
    _style_axis(ax, "Time (s)", "Sensor")
    _mark_events(ax, events)
    ax.legend(loc="upper right", frameon=True, facecolor="#151A24", edgecolor="#4B5563")
    return _finalize(fig, "Sensor Health Timeline")


def plot_position_error(log: dict, events: EventList = None) -> plt.Figure:
    """Plot total and per-axis position estimation error."""
    t = _time(log)
    error = np.asarray(log["true_position"]) - np.asarray(log["est_position"])
    norm = np.linalg.norm(error, axis=1)

    fig, ax = plt.subplots(figsize=(12, 6))
    ax.plot(t, norm, color=TRUE_COLOR, linewidth=2.3, label="Position error norm")
    for i, label in enumerate(("North", "East", "Down")):
        ax.plot(t, error[:, i], color=AXIS_COLORS[i], linewidth=1.2, alpha=0.75, label=f"{label} error")
    _style_axis(ax, "Time (s)", "Error (m)")
    _mark_events(ax, events)
    ax.legend(loc="best", frameon=True, facecolor="#151A24", edgecolor="#4B5563")
    return _finalize(fig, "Position Estimation Error")
