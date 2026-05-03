import numpy as np
import pytest

from main import Simulation
from src.config import SimParams


def test_closed_loop_holds_hover_with_ekf_feedback():
    np.random.seed(0)
    sim = Simulation()

    sim.run(duration=10.0)

    target_error = np.linalg.norm(sim.drone.state.position - sim.target_position)
    assert target_error < 0.5


def test_simulation_rejects_invalid_loop_rates():
    params = SimParams(ultrasonic_rate_hz=0.0)
    with pytest.raises(ValueError, match="ultrasonic rates must be positive"):
        Simulation(sim_params=params)


def test_simulation_rejects_non_dividing_loop_rates():
    params = SimParams(ultrasonic_rate_hz=30.0)
    with pytest.raises(ValueError, match="ultrasonic rate must divide"):
        Simulation(sim_params=params)
