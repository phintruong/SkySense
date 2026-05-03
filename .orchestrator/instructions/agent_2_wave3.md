# Agent 2 Instructions — Wave 3

## Assignment
- **Assigned Tool:** Codex
- **Wave:** 3
- **Role:** Main simulation loop + telemetry logger (integration point)

## Objective
Wire all Wave 2 modules (dynamics, control, sensors) and the Wave 3 EKF into a single simulation loop that runs at 200 Hz. This is the integration point — the `Simulation` class that everything else (server, demos) will drive.

## Context

**Conventions:**
- **NED coordinate system**: X=North, Y=East, Z=Down. Gravity = [0, 0, +9.81].
- **Sim rate**: 200 Hz (dt=0.005). EKF predicts every step. Ultrasonic fires at 20 Hz (every 10th step).
- **Hover target**: [0, 0, -2.0] in NED = 2 meters above ground.

**Module interfaces (all from Wave 2, verified working):**

```python
# Simulation
from src.simulation import DroneModel, DroneState, MotorModel, Environment
# DroneModel(params).step(motor_thrusts[4], dt, external_force[3]) -> DroneState
# DroneModel.get_body_acceleration() -> np.array[3]  (for IMU sim)
# DroneModel.get_angular_velocity() -> np.array[3]   (for IMU sim)
# MotorModel(params).update(commanded[4], dt) -> actual[4]
# Environment().get_external_force(pos, time, dt) -> force[3]
# Environment().inject_disturbance(force[3], duration=0.1)

# Control
from src.control import PIDController, AttitudeController, PositionController, MotorMixer
# PositionController().compute(current_pos, desired_pos, current_vel, current_yaw, dt) -> (roll, pitch, thrust)
# AttitudeController().compute(current_euler[3], desired_euler[3], dt) -> torques[3]
# MotorMixer(params).mix(thrust, roll_torque, pitch_torque, yaw_torque) -> motor_thrusts[4]

# Sensors
from src.sensors import IMUSim, UltrasonicSim
# IMUSim(sim_params).read(true_accel_ned[3], true_angular_vel_body[3], quaternion[4], dt) -> {"accel": [3], "gyro": [3], "healthy": bool}
# UltrasonicSim(sim_params).read(true_position_ned[3]) -> {"altitude": float, "valid": bool}

# Navigation (Wave 3, Task 3.1 — will exist by the time you run)
from src.navigation import StateEstimator
# StateEstimator(sim_params).predict(imu_data, dt)
# StateEstimator.update_accel(imu_data)
# StateEstimator.update_altitude(ultrasonic_data)
# StateEstimator.get_state() -> {"position": [3], "velocity": [3], "quaternion": [4], "euler": [3], "gyro_bias": [3]}
# StateEstimator.is_emergency() -> bool
# StateEstimator.get_health() -> {"imu_healthy": bool, "ultrasonic_healthy": bool, "emergency": bool, "covariance_trace": float, "innovation_norm": float}
```

**DroneParams key values:**
- mass=1.8, gravity=9.81, hover_thrust_per_motor≈4.41
- motor_max_thrust=12.0

**SimParams key values:**
- dt=0.005, ultrasonic_rate_hz=20.0, imu_rate_hz=200.0

## Tasks

### Task 3.2a: Main Simulation Loop
- **Description:** Create `main.py`

```python
import numpy as np
from src.config import DroneParams, SimParams
from src.simulation import DroneModel, MotorModel, Environment
from src.sensors import IMUSim, UltrasonicSim
from src.navigation import StateEstimator
from src.control import AttitudeController, PositionController, MotorMixer
from src.telemetry import DataLogger
from src.math_utils import quat_to_euler, ned_to_body

class Simulation:
    def __init__(self, drone_params: DroneParams = None, sim_params: SimParams = None):
        self.drone_params = drone_params or DroneParams()
        self.sim_params = sim_params or SimParams()
        
        # Subsystems
        self.drone = DroneModel(self.drone_params)
        self.motors = MotorModel(self.drone_params)
        self.environment = Environment()
        self.imu = IMUSim(self.sim_params)
        self.ultrasonic = UltrasonicSim(self.sim_params)
        self.estimator = StateEstimator(self.sim_params)
        self.attitude_ctrl = AttitudeController()
        self.position_ctrl = PositionController(
            hover_thrust=self.drone_params.mass * self.drone_params.gravity
        )
        self.mixer = MotorMixer(self.drone_params)
        self.logger = DataLogger()
        
        # State
        self.time = 0.0
        self.step_count = 0
        self.target_position = np.array([0.0, 0.0, -2.0])  # hover at 2m
        
        # Sensor timing
        self._ultrasonic_interval = int(self.sim_params.imu_rate_hz / self.sim_params.ultrasonic_rate_hz)
    
    def step(self, dt: float = None):
        """
        One simulation timestep. Order:
        
        1. Read sensors from true state
           - IMU every step (200 Hz)
           - Ultrasonic every Nth step (20 Hz)
        2. EKF predict (every step, using IMU)
        3. EKF update altitude (when ultrasonic fires)
        4. EKF update accel (every step)
        5. Get estimated state
        6. Check emergency -> if emergency, command zero roll/pitch + slow descent thrust
        7. Position controller: estimated state + target -> desired angles + thrust
        8. Attitude controller: estimated euler vs desired euler -> torques
        9. Motor mixer: thrust + torques -> 4 motor commands
        10. Motor model: commanded -> actual (first-order lag)
        11. Drone model: actual thrusts + environment -> new true state
        12. Log everything
        """
        dt = dt or self.sim_params.dt
        
        # --- Sensors ---
        true_state = self.drone.state
        true_accel_ned = ...  # need to compute from forces or use get_body_acceleration
        # Actually: DroneModel.get_body_acceleration() returns body-frame accel.
        # But IMUSim.read() wants true_acceleration_ned.
        # The IMUSim internally computes specific_force = a_ned - g_ned, then rotates to body.
        # So we need to pass true acceleration in NED frame.
        #
        # True acceleration in NED = (F_total / mass) where F_total includes gravity.
        # But specific force measured by IMU = a_ned - g_ned = F_nongravity / mass
        # 
        # Simplest approach: pass the body acceleration from DroneModel (which is 
        # total_force/mass in NED, including gravity). The IMUSim subtracts gravity internally.
        #
        # WAIT: DroneModel.get_body_acceleration() returns acceleration in BODY frame.
        # But IMUSim.read() expects true_acceleration_ned (NED frame).
        # So we need NED acceleration. We can get this from the drone's last force computation.
        # 
        # Alternative: store last_acceleration_ned in DroneModel during step().
        # For now: convert body accel to NED using body_to_ned.
        #
        # Actually look at the IMUSim.read signature:
        #   read(true_acceleration_ned, true_angular_velocity_body, quaternion, dt)
        # It expects NED acceleration. We need to either:
        #   a) Have DroneModel expose NED acceleration, or
        #   b) Rotate body accel to NED ourselves
        #
        # Use body_to_ned(quaternion, body_accel) to convert.
        
        # ... implement the full step ...
        
        self.time += dt
        self.step_count += 1
    
    def run(self, duration: float = None):
        """Run simulation for given duration (default: sim_params.duration)."""
        duration = duration or self.sim_params.duration
        steps = int(duration / self.sim_params.dt)
        for _ in range(steps):
            self.step()
    
    def get_telemetry_snapshot(self) -> dict:
        """
        Current state snapshot for WebSocket streaming.
        Returns:
        {
            "timestamp": float,
            "true_state": {
                "position": [3], "velocity": [3],
                "quaternion": [4], "euler": [3],
                "angular_velocity": [3]
            },
            "estimated_state": {
                "position": [3], "velocity": [3],
                "quaternion": [4], "euler": [3],
                "gyro_bias": [3]
            },
            "control": {
                "thrust": float, "torques": [3],
                "motor_commands": [4], "motor_actual": [4]
            },
            "sensors": {
                "imu_healthy": bool, "ultrasonic_healthy": bool
            },
            "ekf": {
                "innovation_norm": float, "covariance_trace": float
            },
            "emergency": bool
        }
        """
    
    def inject_disturbance(self, force: np.ndarray, duration: float = 0.1):
        """Pass disturbance to environment."""
        self.environment.inject_disturbance(force, duration)
    
    def inject_sensor_failure(self, sensor: str, mode: str):
        """
        Inject sensor failure.
        sensor: "imu", "accel", "ultrasonic"
        mode: "off", "noisy", "recover"
        """
        if sensor in ("imu", "accel"):
            self.imu.inject_failure(mode)
        elif sensor == "ultrasonic":
            self.ultrasonic.inject_failure(mode)
    
    def set_target(self, position: np.ndarray):
        """Set target position (NED)."""
        self.target_position = np.array(position, dtype=float)
    
    def reset(self):
        """Reset all subsystems."""
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


def main():
    """Run standalone simulation, print summary."""
    print("SkySense GNC Simulation")
    print("=" * 40)
    
    sim = Simulation()
    print(f"Target: {sim.target_position} (NED)")
    print(f"Running for {sim.sim_params.duration}s at {1/sim.sim_params.dt:.0f} Hz...")
    
    sim.run(duration=10.0)  # 10 seconds for quick test
    
    # Print final state
    est = sim.estimator.get_state()
    true_state = sim.drone.state
    print(f"\nFinal true position:      {true_state.position}")
    print(f"Final estimated position: {est['position']}")
    print(f"Position error:           {np.linalg.norm(true_state.position - est['position']):.4f} m")
    print(f"Target:                   {sim.target_position}")
    print(f"Target error:             {np.linalg.norm(true_state.position - sim.target_position):.4f} m")
    
    # Plot if logger has data
    log = sim.logger.get_log()
    if log and len(log.get("time", [])) > 0:
        print(f"\nLogged {len(log['time'])} timesteps")
    
    print("\nDone.")


if __name__ == "__main__":
    main()
```

**Key implementation note — getting NED acceleration for IMU sim:**

The DroneModel stores its last computed acceleration. You have two options:
1. `body_to_ned(quaternion, drone.get_body_acceleration())` — convert body accel to NED
2. Add a `get_ned_acceleration()` method to DroneModel (but you can't modify simulation files)

Use option 1. The body acceleration from DroneModel is the total acceleration (including gravity effects) in body frame. When you rotate it to NED, you get the true NED acceleration that includes gravity. The IMUSim then subtracts gravity internally to get specific force.

**Emergency handling:**
When `estimator.is_emergency()` is True:
- Override control: set desired_roll=0, desired_pitch=0
- Set thrust to ~80% of hover thrust (controlled descent)
- This gives a slow, stable descent without attitude commands

- **Acceptance criteria:**
  - [ ] Simulation class wires all modules correctly
  - [ ] step() follows the correct order (sensors -> EKF -> control -> dynamics)
  - [ ] Ultrasonic fires at 20 Hz (every 10th step)
  - [ ] Emergency override works (zero angles + descent thrust)
  - [ ] get_telemetry_snapshot() returns complete dict
  - [ ] inject_disturbance/inject_sensor_failure/reset work
  - [ ] `python main.py` runs without error and drone converges to target
  - [ ] Standalone main() prints meaningful output

### Task 3.2b: Telemetry Logger
- **Description:** Create `src/telemetry/logger.py`

```python
import numpy as np

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
        """
        Record one timestep.
        
        Args:
            timestamp: simulation time
            true_state: DroneState (has .position, .velocity, .quaternion)
            estimated_state: dict from StateEstimator.get_state()
            control: {"thrust": float, "torques": [3], "motor_commands": [4], "motor_actual": [4]}
            health: dict from StateEstimator.get_health()
        """
        self._log["time"].append(timestamp)
        self._log["true_position"].append(true_state.position.copy())
        self._log["true_velocity"].append(true_state.velocity.copy())
        self._log["true_quaternion"].append(true_state.quaternion.copy())
        # ... fill all fields ...
    
    def get_log(self) -> dict:
        """Return log as dict of numpy arrays."""
        return {k: np.array(v) if v else np.array([]) for k, v in self._log.items()}
    
    def clear(self):
        """Reset all logs."""
        for k in self._log:
            self._log[k] = []
    
    def save(self, filename: str):
        """Save to .npz file."""
        log = self.get_log()
        np.savez(filename, **log)
```

- **Acceptance criteria:**
  - [ ] log() records all fields
  - [ ] get_log() returns dict of numpy arrays
  - [ ] clear() resets everything
  - [ ] save() writes .npz file

### Task 3.2c: Module Init
- **Description:** Create `src/telemetry/__init__.py`
```python
from .logger import DataLogger
```

- **Acceptance criteria:**
  - [ ] `from src.telemetry import DataLogger` works

## Scope

### Files you OWN:
- `main.py`
- `src/telemetry/logger.py`
- `src/telemetry/__init__.py`

### Files to AVOID (read only):
- `src/simulation/` (Wave 2)
- `src/control/` (Wave 2)
- `src/sensors/` (Wave 2)
- `src/navigation/` (Agent 1, Wave 3)
- `src/config/`
- `src/math_utils/`

## Dependencies

### Before you start:
- All Wave 2 modules complete and tested (38 tests passing)
- Task 3.1 (EKF + StateEstimator) must be complete
- Verify: `from src.navigation import StateEstimator` works

### What depends on YOUR output:
- **Task 3.3 (WebSocket Server)**: imports Simulation class from main.py
- **Task 4.1 (Demos)**: imports Simulation class
- **Task 4.2 (Frontend)**: consumes telemetry via WebSocket

## Status Updates
Write to `.orchestrator/status/agent_2_wave3_status.md`.

## Definition of Done
- [ ] `Simulation` class wires all subsystems
- [ ] `python main.py` runs, drone hovers at target altitude
- [ ] Telemetry logger records full state history
- [ ] get_telemetry_snapshot() returns WebSocket-ready dict
- [ ] Emergency mode triggers controlled descent
- [ ] All sensor failure injection methods work
- [ ] No files outside scope were modified
