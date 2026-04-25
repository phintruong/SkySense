## Task 2.1a: Drone Dynamics Model
- status: DONE
- blocker: none
- notes: Implemented DroneState and DroneModel with NED gravity, body-to-NED thrust, drag, external force injection, ground constraint, quad-X torque computation, Euler rigid-body angular dynamics, and quaternion integration. Hover sanity check with all motors at hover_thrust_per_motor keeps velocity at zero.

## Task 2.1b: Motor Model
- status: DONE
- blocker: none
- notes: Implemented first-order motor lag with thrust clamping to DroneParams min/max. Step response converges to commanded thrust.

## Task 2.1c: Environment
- status: DONE
- blocker: none
- notes: Implemented constant wind, temporary disturbance injection, combined external force output, gust timer expiration, and reset.

## Task 2.1d: Module Init
- status: DONE
- blocker: none
- notes: Exported DroneModel, DroneState, MotorModel, and Environment from src.simulation.
