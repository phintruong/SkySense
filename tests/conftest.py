import pytest

from src.config import DroneParams, SimParams


@pytest.fixture
def drone_params():
    return DroneParams()


@pytest.fixture
def sim_params():
    return SimParams()
