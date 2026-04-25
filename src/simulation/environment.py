import numpy as np


class Environment:
    def __init__(self):
        self.constant_wind = np.zeros(3, dtype=float)
        self.gust_force = np.zeros(3, dtype=float)
        self._gust_remaining = 0.0

    def set_wind(self, wind_force: np.ndarray) -> None:
        """Set constant wind force in NED frame."""
        self.constant_wind = np.asarray(wind_force, dtype=float)

    def inject_disturbance(self, force: np.ndarray, duration: float = 0.1) -> None:
        """Inject a temporary force disturbance."""
        self.gust_force = np.asarray(force, dtype=float)
        self._gust_remaining = max(0.0, float(duration))

    def get_external_force(self, position: np.ndarray, time: float, dt: float) -> np.ndarray:
        """Get constant wind plus any active gust force."""
        del position, time
        gust = self.gust_force if self._gust_remaining > 0.0 else np.zeros(3, dtype=float)
        total = self.constant_wind + gust

        if self._gust_remaining > 0.0:
            self._gust_remaining = max(0.0, self._gust_remaining - dt)
            if self._gust_remaining == 0.0:
                self.gust_force = np.zeros(3, dtype=float)

        return total.copy()

    def reset(self) -> None:
        """Reset all disturbances."""
        self.constant_wind = np.zeros(3, dtype=float)
        self.gust_force = np.zeros(3, dtype=float)
        self._gust_remaining = 0.0
