# Bug: Closed-Loop Altitude Divergence

## Status
Resolved.

## Summary
The full closed-loop simulation with EKF feedback was diverging during hover. The drone could not reliably hold `target_position = [0, 0, -2]`; it would oscillate, tilt, lose ultrasonic observability, and eventually crash or drift far from the target.

## Root Cause
The final failure was not the altitude PID alone. The closed-loop instability came from attitude feedback coupling:

1. Quaternion integration used the wrong multiplication order for the repo's convention.
   - The project stores quaternions as body-to-NED orientation.
   - Angular velocity is in body frame.
   - For this convention, propagation must use `q_dot = 0.5 * quat_multiply(q, omega_quat)`.
   - Both the simulation model and EKF had used `quat_multiply(omega_quat, q)`, which made attitude propagation inconsistent during rotation.

2. The attitude controller's derivative action was based on noisy estimated Euler-angle error.
   - EKF attitude estimates stay near level because accelerometer updates assume low translational acceleration.
   - In closed-loop flight, thrust and small disturbances make that assumption imperfect.
   - The derivative term amplified estimator noise and lag, causing real motor torque commands even when desired roll/pitch were zero.

Together, these caused a positive feedback loop: small attitude estimate errors produced motor torque, real tilt caused horizontal drift and altitude loss, and the estimator/controller disagreement grew.

## Fix Applied
Implemented these changes:

- `src/simulation/drone_model.py`
  - Changed quaternion integration to `quat_multiply(q, omega_quat)`.

- `src/navigation/ekf.py`
  - Changed EKF quaternion prediction to the same body-rate convention.

- `src/control/attitude_controller.py`
  - Added optional `current_rates_body` input.
  - When body rates are provided, derivative damping uses measured angular rate: `-kd * body_rate`.
  - This preserves the existing PID API for tests and callers that do not pass body rates.

- `main.py`
  - Passes bias-corrected IMU gyro rates into the attitude controller:
    `imu_data["gyro"] - est["gyro_bias"]`.

- `tests/test_closed_loop.py`
  - Added a deterministic regression test that runs 10 seconds of closed-loop hover with EKF feedback and requires final target error `< 0.5 m`.

## Verification
Commands run:

```bash
pytest tests -q
python main.py
```

Results:

- Full test suite: `47 passed`
- `python main.py` final target error: `0.3887 m`
- Additional seeded 10-run check: max target error `0.438 m`, mean target error `0.325 m`

## Notes
The earlier mitigations are still useful and remain in place:

- Altitude PID sign is corrected for NED.
- Outer position loop runs at 50 Hz.
- Z control uses direct ultrasonic altitude.
- X/Y position hold is disabled for v1 because there is no GPS or external position sensor.
- The simulation initializes at hover altitude with motors pre-warmed to hover thrust.

The bug is considered fixed as of this note because the documented verification target is satisfied: after 10 seconds, the true final position stays within `0.5 m` of `[0, 0, -2]`.
