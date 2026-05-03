import numpy as np

from main import Simulation


def test_closed_loop_holds_hover_with_ekf_feedback():
    np.random.seed(0)
    sim = Simulation()

    sim.run(duration=10.0)

    target_error = np.linalg.norm(sim.drone.state.position - sim.target_position)
    assert target_error < 0.5
