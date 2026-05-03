"""WebSocket protocol definitions for SkySense GNC telemetry."""

from dataclasses import asdict, dataclass
from numbers import Real
from typing import Any

import numpy as np


@dataclass
class TelemetryMessage:
    """Server-to-client telemetry payload."""

    type: str = "telemetry"
    timestamp: float = 0.0
    true_state: dict | None = None
    estimated_state: dict | None = None
    control: dict | None = None
    sensors: dict | None = None
    ekf: dict | None = None
    emergency: bool = False

    def to_json(self) -> dict:
        """Convert to a JSON-serializable dict."""
        return _convert_numpy(asdict(self))


VALID_COMMANDS = {"inject_disturbance", "sensor_failure", "reset_sim", "set_target"}
VALID_SENSORS = {"imu", "accel", "ultrasonic"}
VALID_SENSOR_MODES = {"off", "noisy", "recover"}


def parse_command(data: dict) -> dict:
    """
    Parse and validate an incoming WebSocket command.

    Expected formats:
    {"type": "inject_disturbance", "force": [fx, fy, fz], "duration": 0.1}
    {"type": "sensor_failure", "sensor": "ultrasonic"|"accel"|"imu", "mode": "off"|"noisy"|"recover"}
    {"type": "reset_sim"}
    {"type": "set_target", "position": [x, y, z]}
    """
    if not isinstance(data, dict):
        raise ValueError("Command payload must be an object")

    command_type = data.get("type")
    if command_type not in VALID_COMMANDS:
        raise ValueError(f"Unknown command type: {command_type}")

    if command_type == "inject_disturbance":
        force = _parse_vector(data.get("force"), "force")
        duration = data.get("duration", 0.1)
        if not _is_number(duration) or float(duration) <= 0:
            raise ValueError("duration must be a positive number")
        return {"type": command_type, "force": force, "duration": float(duration)}

    if command_type == "sensor_failure":
        sensor = data.get("sensor")
        mode = data.get("mode")
        if sensor not in VALID_SENSORS:
            raise ValueError(f"Invalid sensor: {sensor}")
        if mode not in VALID_SENSOR_MODES:
            raise ValueError(f"Invalid sensor failure mode: {mode}")
        return {"type": command_type, "sensor": sensor, "mode": mode}

    if command_type == "set_target":
        position = _parse_vector(data.get("position"), "position")
        return {"type": command_type, "position": position}

    return {"type": command_type}


def _parse_vector(value: Any, field_name: str) -> list[float]:
    if isinstance(value, np.ndarray):
        value = value.tolist()
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{field_name} must be a 3-element list")
    if not all(_is_number(component) for component in value):
        raise ValueError(f"{field_name} must contain only numbers")
    return [float(component) for component in value]


def _is_number(value: Any) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool)


def _convert_numpy(obj):
    """Recursively convert numpy arrays and scalar types to JSON-safe values."""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, np.floating):
        return float(obj)
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.bool_):
        return bool(obj)
    if isinstance(obj, dict):
        return {key: _convert_numpy(value) for key, value in obj.items()}
    if isinstance(obj, list):
        return [_convert_numpy(item) for item in obj]
    if isinstance(obj, tuple):
        return [_convert_numpy(item) for item in obj]
    return obj
