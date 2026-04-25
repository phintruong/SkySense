import numpy as np

from src.config import SimParams


class UltrasonicSim:
    # HC-SR04 specifications
    MIN_RANGE = 0.02  # 2 cm minimum
    MAX_RANGE = 4.0  # 4 m maximum

    def __init__(self, params: SimParams):
        """Simulated HC-SR04 downward-facing ultrasonic sensor."""
        self.params = params
        self._healthy = True
        self._failure_mode = "normal"

    def read(self, true_position_ned: np.ndarray) -> dict:
        """Generate noisy altitude reading from true NED position."""
        if self._failure_mode == "off":
            return {"altitude": 0.0, "valid": False}

        true_altitude = -float(np.asarray(true_position_ned, dtype=float)[2])
        noisy_altitude = true_altitude + float(
            np.random.normal(0.0, self.params.ultrasonic_noise_std)
        )
        in_range = self.MIN_RANGE <= true_altitude <= self.MAX_RANGE

        return {"altitude": noisy_altitude, "valid": in_range}

    def inject_failure(self, mode: str):
        """Set failure mode: off or recover."""
        if mode == "recover":
            self._failure_mode = "normal"
            self._healthy = True
            return

        if mode not in {"normal", "off"}:
            raise ValueError(f"Unsupported ultrasonic failure mode: {mode}")

        self._failure_mode = mode
        self._healthy = mode != "off"

    def is_healthy(self) -> bool:
        return self._healthy

    def reset(self):
        self._healthy = True
        self._failure_mode = "normal"
