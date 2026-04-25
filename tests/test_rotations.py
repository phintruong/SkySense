import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.math_utils import (
    body_to_ned,
    euler_to_quat,
    ned_to_body,
    quat_from_axis_angle,
    quat_identity,
    quat_to_euler,
    quat_to_rotation_matrix,
    rotation_matrix_to_quat,
)


def test_identity_euler():
    euler = quat_to_euler(quat_identity())
    np.testing.assert_allclose(euler, [0.0, 0.0, 0.0], atol=1e-10)


def test_euler_round_trip():
    for roll, pitch, yaw in [(0.3, 0.2, 0.1), (-0.5, 0.4, -1.0), (0.0, 0.0, np.pi)]:
        q = euler_to_quat(roll, pitch, yaw)
        recovered = quat_to_euler(q)
        np.testing.assert_allclose(recovered, [roll, pitch, yaw], atol=1e-10)


def test_euler_to_quat_matches_axis_angle_for_yaw():
    q = euler_to_quat(0.0, 0.0, np.pi / 2.0)
    expected = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), np.pi / 2.0)
    np.testing.assert_allclose(q, expected, atol=1e-10)


def test_90_pitch():
    q = euler_to_quat(0.0, np.pi / 2.0, 0.0)
    euler = quat_to_euler(q)
    np.testing.assert_allclose(euler[1], np.pi / 2.0, atol=1e-10)


def test_gimbal_lock():
    q = euler_to_quat(0.1, np.pi / 2.0, 0.2)
    euler = quat_to_euler(q)
    assert not np.any(np.isnan(euler))


def test_body_to_ned_identity():
    v = np.array([1.0, 2.0, 3.0])
    result = body_to_ned(quat_identity(), v)
    np.testing.assert_allclose(result, v, atol=1e-10)


def test_body_ned_round_trip():
    q = euler_to_quat(0.3, 0.2, 0.1)
    v = np.array([1.0, -0.5, 2.0])
    v_ned = body_to_ned(q, v)
    v_back = ned_to_body(q, v_ned)
    np.testing.assert_allclose(v_back, v, atol=1e-10)


def test_rotation_matrix_round_trip():
    q = euler_to_quat(0.5, -0.3, 1.2)
    R = quat_to_rotation_matrix(q)
    q_back = rotation_matrix_to_quat(R)
    if q_back[0] * q[0] < 0.0:
        q_back = -q_back
    np.testing.assert_allclose(q_back, q, atol=1e-10)


def test_rotation_matrix_is_orthogonal():
    q = euler_to_quat(0.5, -0.3, 1.2)
    R = quat_to_rotation_matrix(q)
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)


def test_rotation_matrix_matches_vector_rotation():
    q = euler_to_quat(0.5, -0.3, 1.2)
    v = np.array([1.0, -2.0, 0.5])
    R = quat_to_rotation_matrix(q)
    np.testing.assert_allclose(R @ v, body_to_ned(q, v), atol=1e-10)


def test_ned_gravity_in_body():
    g_ned = np.array([0.0, 0.0, 9.81])
    g_body = ned_to_body(quat_identity(), g_ned)
    np.testing.assert_allclose(g_body, g_ned, atol=1e-10)

    q = euler_to_quat(0.0, np.radians(30.0), 0.0)
    g_body = ned_to_body(q, g_ned)
    assert g_body[0] < 0.0
    assert g_body[2] > 0.0


def test_rotation_matrix_to_quat_180_degree_rotation():
    q = quat_from_axis_angle(np.array([1.0, 0.0, 0.0]), np.pi)
    R = quat_to_rotation_matrix(q)
    q_back = rotation_matrix_to_quat(R)
    np.testing.assert_allclose(abs(q_back[1]), 1.0, atol=1e-10)
