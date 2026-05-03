## Task 3.2a: Main Simulation Loop
- status: DONE
- blocker: none
- notes: Implemented Simulation class in main.py wiring all subsystems. Sensors -> EKF -> control -> dynamics loop at 200 Hz. Outer position loop at 50 Hz, inner attitude loop at 200 Hz. Ultrasonic at 20 Hz. Emergency override (zero angles + 80% hover thrust). get_telemetry_snapshot() returns complete dict. inject_disturbance, inject_sensor_failure, set_target, reset all work. Drone initialized at hover altitude with pre-warmed motors.

## Task 3.2b: Telemetry Logger
- status: DONE
- blocker: none
- notes: DataLogger records all 18 fields (true/estimated state, control, health). get_log() returns dict of numpy arrays. clear() and save() implemented.

## Task 3.2c: Module Init
- status: DONE
- blocker: none
- notes: Exported DataLogger from src.telemetry.

## Known Issue: Closed-Loop Altitude Divergence
- status: BUG
- blocker: Simulation runs but drone does not hold altitude — EKF position estimate drifts, causing control loop instability
- notes: See .orchestrator/bugs/closed_loop_divergence.md for full analysis. Dynamics and control verified stable independently. Root cause is EKF noise amplified by PID derivative at high rate. Multiple mitigations applied (50 Hz outer loop, direct ultrasonic altitude feedback, x/y error zeroing) but altitude still oscillates and drone eventually crashes. Needs EKF tuning or control gain redesign for closed-loop stability.
