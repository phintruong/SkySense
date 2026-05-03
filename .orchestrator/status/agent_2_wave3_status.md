## Task 3.2a: Main Simulation Loop
- status: DONE
- blocker: none
- notes: Implemented Simulation class in main.py wiring all subsystems. Sensors -> EKF -> control -> dynamics loop at 200 Hz. Outer position loop at 50 Hz, inner attitude loop at 200 Hz. Ultrasonic at 20 Hz. Emergency override uses zero angles plus reduced thrust. get_telemetry_snapshot() returns complete dict. inject_disturbance, inject_sensor_failure, set_target, and reset work. Drone initializes at hover altitude with pre-warmed motors.

## Task 3.2b: Telemetry Logger
- status: DONE
- blocker: none
- notes: DataLogger records true/estimated state, control outputs, sensor health, EKF health, and emergency status. get_log() returns dict of numpy arrays. clear() and save() are implemented.

## Task 3.2c: Module Init
- status: DONE
- blocker: none
- notes: Exported DataLogger from src.telemetry.

## Resolved Issue: Closed-Loop Altitude Divergence
- status: DONE
- blocker: none
- notes: See .orchestrator/bugs/closed_loop_divergence.md for final analysis. The resolved root cause was attitude feedback coupling: quaternion propagation used the wrong body-rate multiplication order, and attitude derivative action amplified noisy estimated Euler error. Fixes applied in DroneModel, EKF, AttitudeController, and main.py. Closed-loop hover now passes the <0.5m regression.
