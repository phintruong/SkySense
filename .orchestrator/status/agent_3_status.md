## Task 2.3a: IMU Simulator
- status: DONE
- blocker: none
- notes: Added `IMUSim` with NED specific-force conversion (`a_ned - g_ned` then `ned_to_body`), gyro bias random walk, and failure modes (`off`, `noisy`, `recover`).

## Task 2.3b: Ultrasonic Simulator
- status: DONE
- blocker: none
- notes: Added `UltrasonicSim` converting NED z to altitude (`-z`), HC-SR04 range gating (0.02m to 4.0m), Gaussian noise, and failure handling (`off`, `recover`).

## Task 2.3c: Module Init
- status: DONE
- blocker: none
- notes: Updated `src/sensors/__init__.py` to export `IMUSim` and `UltrasonicSim`; wrapped hardware imports in `try/except` so package import remains safe without `pyrplidar`/`gpiozero`.
