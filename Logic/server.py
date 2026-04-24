"""
SkySense WebSocket Server
Streams live LiDAR scan data and navigation decisions to the frontend.
Falls back to a demo sweep if no RPLIDAR hardware is connected.
"""

import asyncio
import json
import math
import time
import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware

from core import process_scan, DANGER_RADIUS, FORWARD_CONE_HALF_ANGLE

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger(__name__)

# --- Hardware detection ---
_hardware_available = False
_lidar = None

try:
    from hardware import RPLidarReader

    _lidar = RPLidarReader()
    if _lidar.connect():
        _lidar.start()
        _hardware_available = True
        logger.info("RPLIDAR hardware connected — streaming real data")
    else:
        _lidar = None
        logger.warning("RPLIDAR not found — using demo mode")
except Exception as e:
    logger.warning(f"Hardware unavailable ({e}) — using demo mode")


def _shutdown_lidar():
    global _lidar
    if _lidar:
        _lidar.disconnect()
        _lidar = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    yield
    _shutdown_lidar()


app = FastAPI(title="SkySense", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# --- Demo data generator (rotating sweep, no real hardware) ---
def _demo_scan(tick: int):
    """Generate a fake 360° sweep with a couple of close obstacles that rotate."""
    points = []
    obstacle_angle = (tick * 15) % 360
    for deg in range(0, 360, 2):
        # Base distance ~2 m with some noise
        dist = 2.0 + 0.3 * math.sin(math.radians(deg * 3))
        # Place a close obstacle cluster
        if abs((deg - obstacle_angle + 180) % 360 - 180) < 20:
            dist = 0.25 + 0.1 * math.sin(math.radians(deg * 7))
        points.append((float(deg), round(dist, 3)))
    return points


# --- REST health ---
@app.get("/api/status")
async def status():
    return {
        "ok": True,
        "hardware": _hardware_available,
        "dangerRadius": DANGER_RADIUS,
        "forwardCone": FORWARD_CONE_HALF_ANGLE,
    }


# --- WebSocket: stream LiDAR data ---
@app.websocket("/ws/lidar")
async def lidar_ws(ws: WebSocket):
    await ws.accept()
    logger.info("Client connected to /ws/lidar")
    tick = 0

    try:
        while True:
            # Get scan data (real or demo)
            if _hardware_available and _lidar:
                scan_data = _lidar.get_scan()
                if scan_data is None:
                    await asyncio.sleep(0.05)
                    continue
            else:
                scan_data = _demo_scan(tick)
                tick += 1

            # TODO: replace with real IMU tilt
            tilt_angle = 0

            action, obstacles = process_scan(scan_data, tilt_angle=tilt_angle)

            payload = {
                "scan": scan_data,
                "action": action,
                "obstacles": obstacles,
                "tiltAngle": tilt_angle,
                "timestamp": time.time(),
            }
            await ws.send_text(json.dumps(payload))

            # ~10 Hz for real hardware, slower for demo so it's watchable
            await asyncio.sleep(0.1 if _hardware_available else 0.25)

    except WebSocketDisconnect:
        logger.info("Client disconnected")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)
