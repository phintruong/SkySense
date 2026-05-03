import numpy as np

from src.math_utils import quat_to_euler


class DataLogger:
    def __init__(self):
        self._log = {
            "time": [],
            "true_position": [],
            "true_velocity": [],
            "true_quaternion": [],
            "true_euler": [],
            "est_position": [],
            "est_velocity": [],
            "est_euler": [],
            "est_gyro_bias": [],
            "thrust": [],
            "torques": [],
            "motor_commands": [],
            "motor_actual": [],
            "imu_healthy": [],
            "ultrasonic_healthy": [],
            "innovation_norm": [],
            "covariance_trace": [],
            "emergency": [],
        }

    def log(self, timestamp: float, true_state, estimated_state: dict,
            control: dict, health: dict):
        self._log["time"].append(timestamp)
        self._log["true_position"].append(true_state.position.copy())
        self._log["true_velocity"].append(true_state.velocity.copy())
        self._log["true_quaternion"].append(true_state.quaternion.copy())
        self._log["true_euler"].append(quat_to_euler(true_state.quaternion).copy())
        self._log["est_position"].append(np.array(estimated_state["position"]))
        self._log["est_velocity"].append(np.array(estimated_state["velocity"]))
        self._log["est_euler"].append(np.array(estimated_state["euler"]))
        self._log["est_gyro_bias"].append(np.array(estimated_state["gyro_bias"]))
        self._log["thrust"].append(control["thrust"])
        self._log["torques"].append(np.array(control["torques"]))
        self._log["motor_commands"].append(np.array(control["motor_commands"]))
        self._log["motor_actual"].append(np.array(control["motor_actual"]))
        self._log["imu_healthy"].append(health["imu_healthy"])
        self._log["ultrasonic_healthy"].append(health["ultrasonic_healthy"])
        self._log["innovation_norm"].append(health["innovation_norm"])
        self._log["covariance_trace"].append(health["covariance_trace"])
        self._log["emergency"].append(health["emergency"])

    def get_log(self) -> dict:
        return {k: np.array(v) if v else np.array([]) for k, v in self._log.items()}

    def clear(self):
        for k in self._log:
            self._log[k] = []

    def save(self, filename: str):
        log = self.get_log()
        np.savez(filename, **log)
