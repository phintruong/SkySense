## Task 1.1a: Directory Tree
- status: DONE
- blocker: none
- notes: Created `src/` package directories (`config`, `simulation`, `sensors`, `navigation`, `control`, `telemetry`, `server`, `math_utils`), plus `tests/`, `legacy/`, and `demos/output/`. Added `__init__.py` files for owned `src/` subpackages and `tests/`.

## Task 1.1b: Move Existing Files
- status: DONE
- blocker: none
- notes: Copied `Logic/core/logic.py` to `src/navigation/obstacle_detector.py`, `Logic/hardware/rplidar_reader.py` to `src/sensors/lidar.py`, and `Logic/hardware/hc_sr04_distance.py` to `src/sensors/ultrasonic.py`. Moved `Logic/rplidar_ros2/` to `legacy/rplidar_ros2/`. Deleted duplicate `Logic/rplidar_reader.py`. Root `nul` file not present.

## Task 1.1c: Config System
- status: DONE
- blocker: none
- notes: Added `DroneParams` and `SimParams` dataclasses with requested defaults/comments and exports via `src/config/__init__.py`.

## Task 1.1d: Requirements + Test Scaffold
- status: DONE
- blocker: none
- notes: Added root `requirements.txt` with required dependencies and `tests/conftest.py` fixtures for default `DroneParams()` and `SimParams()`.
