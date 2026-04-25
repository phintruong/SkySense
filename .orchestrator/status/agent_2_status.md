## Task 2.2a: PID Controller
- status: DONE
- blocker: none
- notes: Implemented proportional, integral, derivative terms with first-call derivative suppression, integral anti-windup, output clamping, and reset.

## Task 2.2b: Attitude Controller
- status: DONE
- blocker: none
- notes: Implemented independent roll, pitch, and yaw PIDs. Yaw error uses atan2(sin(error), cos(error)) wrapping and lower yaw gains.

## Task 2.2c: Position Controller
- status: DONE
- blocker: none
- notes: Implemented NED position error rotation into body frame using current yaw, tilt-limited roll/pitch commands, altitude thrust adjustment around hover thrust, and total thrust clamping.

## Task 2.2d: Motor Mixer
- status: DONE
- blocker: none
- notes: Implemented quad-X mixer signs matching the DroneModel torque equations. Motor outputs clamp to DroneParams min/max thrust.

## Task 2.2e: Module Init + Tests
- status: DONE
- blocker: none
- notes: Exported PIDController, AttitudeController, PositionController, and MotorMixer from src.control. `pytest tests/test_pid.py -v` and `pytest tests/test_motor_mixer.py -v` pass.
