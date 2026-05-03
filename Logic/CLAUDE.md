# Logic — LEGACY Python Backend

> **This is the original LiDAR obstacle detection system.** It is being superseded by the new `src/` GNC stack. See the root `CLAUDE.md` and `.orchestrator/plan.md` for the current project direction.

## Status
- `core/logic.py` has been copied to `src/navigation/obstacle_detector.py`
- `hardware/rplidar_reader.py` has been copied to `src/sensors/lidar.py`
- `hardware/hc_sr04_distance.py` has been copied to `src/sensors/ultrasonic.py`
- `rplidar_ros2/` has been moved to `legacy/rplidar_ros2/`
- Duplicate `rplidar_reader.py` at root has been deleted
- `server.py` and `main.py` remain here as reference for the new `src/server/app.py` (Wave 3)

## Still Active
- `server.py` — the FastAPI WebSocket server still works and the React frontend connects to it. Will be replaced by `src/server/app.py` in Wave 3.

## Do Not Modify
These files are kept for reference only. All new development happens in `src/`.
