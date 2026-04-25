import numpy as np
import pytest

from src.config import DroneParams
from src.control import MotorMixer


@pytest.fixture
def mixer():
    return MotorMixer(DroneParams())


def test_pure_thrust(mixer):
    result = mixer.mix(thrust=17.64, torque_roll=0.0, torque_pitch=0.0, torque_yaw=0.0)
    np.testing.assert_allclose(result, [4.41, 4.41, 4.41, 4.41], atol=0.01)


def test_zero_input(mixer):
    result = mixer.mix(thrust=0.0, torque_roll=0.0, torque_pitch=0.0, torque_yaw=0.0)
    np.testing.assert_allclose(result, [0.0, 0.0, 0.0, 0.0], atol=1e-10)


def test_positive_roll(mixer):
    result = mixer.mix(thrust=17.64, torque_roll=1.0, torque_pitch=0.0, torque_yaw=0.0)
    assert result[2] > result[0]
    assert result[3] > result[1]


def test_positive_pitch(mixer):
    result = mixer.mix(thrust=17.64, torque_roll=0.0, torque_pitch=1.0, torque_yaw=0.0)
    assert result[1] > result[0]
    assert result[2] > result[3]


def test_positive_yaw(mixer):
    result = mixer.mix(thrust=17.64, torque_roll=0.0, torque_pitch=0.0, torque_yaw=0.1)
    assert result[1] > result[0]
    assert result[3] > result[2]


def test_clamping(mixer):
    result = mixer.mix(thrust=100.0, torque_roll=0.0, torque_pitch=0.0, torque_yaw=0.0)
    assert np.all(result <= DroneParams().motor_max_thrust + 0.01)
    assert np.all(result >= 0.0)
