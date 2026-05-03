import numpy as np

from src.config import DroneParams, SimParams
from src.control import AttitudeController, MotorMixer, PositionController
from src.math_utils import body_to_ned, quat_to_euler
from src.navigation import StateEstimator
from src.sensors import IMUSim, UltrasonicSim
from src.simulation import DroneModel, MotorModel, Environment
from src.telemetry import DataLogger


class Simulation:
    def __init__(self, drone_params: DroneParams = None, sim_params: SimParams = None):
        self.drone_params = drone_params or DroneParams()
        self.sim_params = sim_params or SimParams()

        self.drone = DroneModel(self.drone_params)
        self.motors = MotorModel(self.drone_params)
        self.environment = Environment()
        self.imu = IMUSim(self.sim_params)
        self.ultrasonic = UltrasonicSim(self.sim_params)
        self.estimator = StateEstimator(self.sim_params)
        self.attitude_ctrl = AttitudeController()
        self.position_ctrl = PositionController(
            hover_thrust=self.drone_params.mass * self.drone_params.gravity,
        )
        self.mixer = MotorMixer(self.drone_params)
        self.logger = DataLogger()

        self.time = 0.0
        self.step_count = 0
        self.target_position = np.array([0.0, 0.0, -2.0])

        self._ultrasonic_interval = int(
            self.sim_params.imu_rate_hz / self.sim_params.ultrasonic_rate_hz
        )

        self._last_thrust = self.drone_params.mass * self.drone_params.gravity
        self._last_torques = np.zeros(3)
        self._last_motor_commands = np.zeros(4)
        self._last_motor_actual = np.zeros(4)
        self._desired_roll = 0.0
        self._desired_pitch = 0.0
        self._desired_yaw = 0.0

        # Outer loop (position control) at 50 Hz, inner loop (attitude) at 200 Hz
        self._position_ctrl_interval = int(self.sim_params.imu_rate_hz / 50.0)
        self._last_altitude = -self.target_position[2]

        self._initialize_at_hover()

    def _initialize_at_hover(self):
        from src.simulation import DroneState
        hover = self.drone_params.hover_thrust_per_motor
        self.drone.reset(DroneState(
            position=self.target_position.copy(),
        ))
        self.motors.thrusts = np.full(4, hover)
        self.estimator.ekf.x[0:3] = self.target_position.copy()
        # Run one dynamics step so stored acceleration reflects hover equilibrium
        self.drone.step(np.full(4, hover), self.sim_params.dt)
        self.drone.state.position = self.target_position.copy()
        self.drone.state.velocity = np.zeros(3)

    def step(self, dt: float = None):
        dt = dt or self.sim_params.dt

        # 1. Read sensors from true state
        true_accel_body = self.drone.get_body_acceleration()
        true_accel_ned = body_to_ned(self.drone.state.quaternion, true_accel_body)
        true_omega_body = self.drone.get_angular_velocity()

        imu_data = self.imu.read(true_accel_ned, true_omega_body,
                                 self.drone.state.quaternion, dt)

        ultrasonic_data = None
        if self.step_count % self._ultrasonic_interval == 0:
            ultrasonic_data = self.ultrasonic.read(self.drone.state.position)
            if ultrasonic_data["valid"]:
                self._last_altitude = ultrasonic_data["altitude"]

        # 2. EKF predict
        self.estimator.predict(imu_data, dt)

        # 3. EKF update altitude (when ultrasonic fires)
        if ultrasonic_data is not None:
            self.estimator.update_altitude(ultrasonic_data)

        # 4. EKF update accel
        self.estimator.update_accel(imu_data)

        # 5. Get estimated state
        est = self.estimator.get_state()
        est_pos = np.array(est["position"])
        est_vel = np.array(est["velocity"])
        est_euler = np.array(est["euler"])

        # 6. Emergency check
        emergency = self.estimator.is_emergency()

        if emergency:
            self._desired_roll = 0.0
            self._desired_pitch = 0.0
            self._desired_yaw = est_euler[2]
            self._last_thrust = 0.8 * self.drone_params.mass * self.drone_params.gravity
        elif self.step_count % self._position_ctrl_interval == 0:
            # 7. Outer loop at 50 Hz — position controller
            # Use ultrasonic altitude directly for z (more reliable than EKF z)
            # x/y unobservable without GPS — zero out x/y error
            ctrl_pos = np.array([
                self.target_position[0],
                self.target_position[1],
                -self._last_altitude,
            ])
            outer_dt = dt * self._position_ctrl_interval
            self._desired_roll, self._desired_pitch, self._last_thrust = \
                self.position_ctrl.compute(
                    ctrl_pos, self.target_position, est_vel, est_euler[2], outer_dt,
                )
            self._desired_yaw = 0.0

        # 8. Inner loop at 200 Hz — attitude controller
        desired_euler = np.array([self._desired_roll, self._desired_pitch, self._desired_yaw])
        body_rates = np.asarray(imu_data["gyro"], dtype=float) - np.asarray(est["gyro_bias"], dtype=float)
        torques = self.attitude_ctrl.compute(est_euler, desired_euler, dt, body_rates)

        # 9. Motor mixer
        motor_commands = self.mixer.mix(self._last_thrust, torques[0], torques[1], torques[2])

        # 10. Motor model
        motor_actual = self.motors.update(motor_commands, dt)

        # 11. Drone dynamics
        ext_force = self.environment.get_external_force(
            self.drone.state.position, self.time, dt,
        )
        self.drone.step(motor_actual, dt, ext_force)

        # Store for telemetry snapshot
        self._last_torques = torques.copy()
        self._last_motor_commands = motor_commands.copy()
        self._last_motor_actual = motor_actual.copy()

        # 12. Log
        health = self.estimator.get_health()
        self.logger.log(
            self.time,
            self.drone.state,
            est,
            {
                "thrust": self._last_thrust,
                "torques": torques,
                "motor_commands": motor_commands,
                "motor_actual": motor_actual,
            },
            health,
        )

        self.time += dt
        self.step_count += 1

    def run(self, duration: float = None):
        duration = duration or self.sim_params.duration
        steps = int(duration / self.sim_params.dt)
        for _ in range(steps):
            self.step()

    def get_telemetry_snapshot(self) -> dict:
        true_state = self.drone.state
        true_euler = quat_to_euler(true_state.quaternion)
        est = self.estimator.get_state()
        health = self.estimator.get_health()

        return {
            "timestamp": self.time,
            "true_state": {
                "position": true_state.position.tolist(),
                "velocity": true_state.velocity.tolist(),
                "quaternion": true_state.quaternion.tolist(),
                "euler": true_euler.tolist(),
                "angular_velocity": true_state.angular_velocity.tolist(),
            },
            "estimated_state": {
                "position": list(est["position"]),
                "velocity": list(est["velocity"]),
                "quaternion": list(est["quaternion"]),
                "euler": list(est["euler"]),
                "gyro_bias": list(est["gyro_bias"]),
            },
            "control": {
                "thrust": float(self._last_thrust),
                "torques": self._last_torques.tolist(),
                "motor_commands": self._last_motor_commands.tolist(),
                "motor_actual": self._last_motor_actual.tolist(),
            },
            "sensors": {
                "imu_healthy": health["imu_healthy"],
                "ultrasonic_healthy": health["ultrasonic_healthy"],
            },
            "ekf": {
                "innovation_norm": float(health["innovation_norm"]),
                "covariance_trace": float(health["covariance_trace"]),
            },
            "emergency": health["emergency"],
        }

    def inject_disturbance(self, force: np.ndarray, duration: float = 0.1):
        self.environment.inject_disturbance(force, duration)

    def inject_sensor_failure(self, sensor: str, mode: str):
        if sensor in ("imu", "accel"):
            self.imu.inject_failure(mode)
        elif sensor == "ultrasonic":
            self.ultrasonic.inject_failure(mode)

    def set_target(self, position: np.ndarray):
        self.target_position = np.array(position, dtype=float)

    def reset(self):
        self.drone.reset()
        self.motors.reset()
        self.environment.reset()
        self.imu.reset()
        self.ultrasonic.reset()
        self.estimator.reset()
        self.attitude_ctrl.reset()
        self.position_ctrl.reset()
        self.mixer.reset()
        self.logger.clear()
        self.time = 0.0
        self.step_count = 0
        self.target_position = np.array([0.0, 0.0, -2.0])
        self._initialize_at_hover()


def main():
    print("SkySense GNC Simulation")
    print("=" * 40)

    sim = Simulation()
    print(f"Target: {sim.target_position} (NED)")
    print(f"Running for 10.0s at {1/sim.sim_params.dt:.0f} Hz...")

    sim.run(duration=10.0)

    est = sim.estimator.get_state()
    true_state = sim.drone.state
    print(f"\nFinal true position:      {true_state.position}")
    print(f"Final estimated position: {np.array(est['position'])}")
    print(f"Position error:           {np.linalg.norm(true_state.position - np.array(est['position'])):.4f} m")
    print(f"Target:                   {sim.target_position}")
    print(f"Target error:             {np.linalg.norm(true_state.position - sim.target_position):.4f} m")

    log = sim.logger.get_log()
    if len(log.get("time", [])) > 0:
        print(f"\nLogged {len(log['time'])} timesteps")

    print("\nDone.")


if __name__ == "__main__":
    main()
