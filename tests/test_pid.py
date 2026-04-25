import pytest

from src.control import PIDController


def test_proportional_only():
    pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
    output = pid.update(error=1.0, dt=0.01)
    assert output == pytest.approx(2.0)


def test_integral_accumulates():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
    pid.update(error=1.0, dt=0.01)
    output = pid.update(error=1.0, dt=0.01)
    assert output == pytest.approx(0.02)


def test_integral_windup():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, integral_max=0.1)
    for _ in range(1000):
        pid.update(error=1.0, dt=0.01)
    output = pid.update(error=1.0, dt=0.01)
    assert output == pytest.approx(0.1)


def test_output_clamping():
    pid = PIDController(kp=100.0, ki=0.0, kd=0.0, output_min=-1.0, output_max=1.0)
    output = pid.update(error=1.0, dt=0.01)
    assert output == pytest.approx(1.0)


def test_zero_error():
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
    output = pid.update(error=0.0, dt=0.01)
    assert output == pytest.approx(0.0)


def test_reset():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0)
    pid.update(error=1.0, dt=0.01)
    pid.reset()
    output = pid.update(error=1.0, dt=0.01)
    assert output == pytest.approx(0.01)


def test_derivative_first_call_zero():
    pid = PIDController(kp=0.0, ki=0.0, kd=1.0)
    output = pid.update(error=5.0, dt=0.01)
    assert output == pytest.approx(0.0)
