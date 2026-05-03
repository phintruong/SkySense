"""FastAPI application for SkySense GNC telemetry."""

import asyncio
import logging
from contextlib import asynccontextmanager

import numpy as np
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

from .protocol import TelemetryMessage, _convert_numpy, parse_command

logger = logging.getLogger(__name__)

simulation = None
sim_task: asyncio.Task | None = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize simulation on startup and stop the background task on shutdown."""
    global sim_task, simulation

    try:
        from main import Simulation
    except ModuleNotFoundError as exc:
        if exc.name != "main":
            raise
        logger.warning("Simulation entrypoint main.Simulation is not available")
        simulation = None
    else:
        simulation = Simulation()
    try:
        yield
    finally:
        if sim_task is not None:
            sim_task.cancel()
            try:
                await sim_task
            except asyncio.CancelledError:
                pass
            sim_task = None


app = FastAPI(title="SkySense GNC Server", lifespan=lifespan)


async def run_simulation():
    """Run the simulation loop in the background."""
    global simulation

    while True:
        if simulation is not None:
            simulation.step()
        await asyncio.sleep(0)


@app.get("/api/status")
async def get_status():
    """Health check and current simulation state summary."""
    if simulation is None:
        return {"status": "not_initialized"}

    snapshot = simulation.get_telemetry_snapshot()
    return {
        "status": "running",
        "time": snapshot["timestamp"],
        "emergency": snapshot["emergency"],
        "position": _convert_numpy(snapshot["true_state"]["position"]),
    }


@app.websocket("/ws/telemetry")
async def telemetry_websocket(websocket: WebSocket):
    """
    Bidirectional WebSocket for GNC telemetry.

    Server -> Client: TelemetryMessage at about 20 Hz.
    Client -> Server: disturbance, sensor failure, reset, and target commands.
    """
    global sim_task

    await websocket.accept()
    if simulation is None:
        await websocket.close(code=1011, reason="Simulation not initialized")
        return

    if sim_task is None or sim_task.done():
        sim_task = asyncio.create_task(run_simulation())

    try:
        while True:
            snapshot = simulation.get_telemetry_snapshot()
            message = TelemetryMessage(
                timestamp=snapshot["timestamp"],
                true_state=snapshot["true_state"],
                estimated_state=snapshot["estimated_state"],
                control=snapshot["control"],
                sensors=snapshot["sensors"],
                ekf=snapshot["ekf"],
                emergency=snapshot["emergency"],
            )
            await websocket.send_json(message.to_json())

            try:
                data = await asyncio.wait_for(websocket.receive_json(), timeout=0.05)
                handle_command(data)
            except asyncio.TimeoutError:
                pass
    except WebSocketDisconnect:
        logger.info("Telemetry WebSocket disconnected")


def handle_command(data: dict):
    """Dispatch an incoming WebSocket command to the simulation."""
    global simulation

    if simulation is None:
        logger.warning("Ignoring command before simulation initialization")
        return

    try:
        command = parse_command(data)
        command_type = command["type"]

        if command_type == "inject_disturbance":
            simulation.inject_disturbance(
                np.array(command["force"]),
                duration=command["duration"],
            )
        elif command_type == "sensor_failure":
            simulation.inject_sensor_failure(command["sensor"], command["mode"])
        elif command_type == "reset_sim":
            simulation.reset()
        elif command_type == "set_target":
            simulation.set_target(np.array(command["position"]))
    except (ValueError, KeyError, TypeError) as exc:
        logger.warning("Invalid command: %s", exc)
