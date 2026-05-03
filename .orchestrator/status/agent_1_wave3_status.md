## Task 3.1a: Extended Kalman Filter
- status: DONE
- blocker: none
- notes: Implemented 13-state EKF with IMU prediction, numerical transition and accelerometer Jacobians, quaternion normalization, altitude and accelerometer updates, covariance propagation, and innovation gating with a startup-tolerant floor for plausible measurements.

## Task 3.1b: State Estimator Wrapper
- status: DONE
- blocker: none
- notes: Added StateEstimator wrapper for EKF prediction and sensor updates. Unhealthy IMU readings fall back to hover assumptions and set emergency; invalid ultrasonic readings are skipped.

## Task 3.1c: Module Init + Tests
- status: DONE
- blocker: none
- notes: Exported EKF and StateEstimator from src.navigation. Added EKF and StateEstimator tests covering initialization, prediction, altitude update, quaternion normalization, covariance growth, innovation gating, and sensor health handling.
