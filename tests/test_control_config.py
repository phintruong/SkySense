import pytest

from src.config import SimParams
from src.control import AttitudeController, PositionController


def test_attitude_controller_uses_sim_params():
    params = SimParams(attitude_kp=1.5, attitude_ki=0.2, attitude_kd=0.7)
    controller = AttitudeController.from_sim_params(params)

    assert controller.roll_pid.kp == pytest.approx(1.5)
    assert controller.roll_pid.ki == pytest.approx(0.2)
    assert controller.roll_pid.kd == pytest.approx(0.7)


def test_position_controller_uses_sim_params():
    params = SimParams(
        position_kp=1.2,
        position_ki=0.3,
        position_kd=0.9,
        altitude_kp=4.0,
        altitude_ki=0.8,
        altitude_kd=2.5,
        max_tilt_angle=0.4,
    )
    controller = PositionController.from_sim_params(params, hover_thrust=17.0)

    assert controller.x_pid.kp == pytest.approx(1.2)
    assert controller.z_pid.kp == pytest.approx(4.0)
    assert controller.max_tilt == pytest.approx(0.4)
    assert controller.hover_thrust == pytest.approx(17.0)
