import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.math_utils import (
    quat_conjugate,
    quat_from_axis_angle,
    quat_identity,
    quat_inverse,
    quat_multiply,
    quat_normalize,
    quat_rotate_vector,
)


def test_identity_rotation():
    v = np.array([1.0, 2.0, 3.0])
    result = quat_rotate_vector(quat_identity(), v)
    np.testing.assert_allclose(result, v, atol=1e-10)


def test_rotate_90_about_z():
    q = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), np.pi / 2.0)
    result = quat_rotate_vector(q, np.array([1.0, 0.0, 0.0]))
    np.testing.assert_allclose(result, [0.0, 1.0, 0.0], atol=1e-10)


def test_rotate_90_about_x():
    q = quat_from_axis_angle(np.array([1.0, 0.0, 0.0]), np.pi / 2.0)
    result = quat_rotate_vector(q, np.array([0.0, 1.0, 0.0]))
    np.testing.assert_allclose(result, [0.0, 0.0, 1.0], atol=1e-10)


def test_rotate_90_about_y():
    q = quat_from_axis_angle(np.array([0.0, 1.0, 0.0]), np.pi / 2.0)
    result = quat_rotate_vector(q, np.array([0.0, 0.0, 1.0]))
    np.testing.assert_allclose(result, [1.0, 0.0, 0.0], atol=1e-10)


def test_multiply_associative():
    q1 = quat_from_axis_angle(np.array([1.0, 0.0, 0.0]), 0.3)
    q2 = quat_from_axis_angle(np.array([0.0, 1.0, 0.0]), 0.5)
    q3 = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), 0.7)
    left = quat_multiply(quat_multiply(q1, q2), q3)
    right = quat_multiply(q1, quat_multiply(q2, q3))
    np.testing.assert_allclose(left, right, atol=1e-10)


def test_normalize_produces_unit():
    q = np.array([2.0, 1.0, 0.5, 0.3])
    result = quat_normalize(q)
    assert abs(np.linalg.norm(result) - 1.0) < 1e-10


def test_normalize_zero_returns_identity():
    result = quat_normalize(np.zeros(4))
    np.testing.assert_allclose(result, quat_identity(), atol=1e-10)


def test_inverse_non_unit_quaternion():
    q = 2.0 * quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), 0.5)
    product = quat_multiply(q, quat_inverse(q))
    np.testing.assert_allclose(product, quat_identity(), atol=1e-10)


def test_rotate_then_inverse():
    q = quat_from_axis_angle(np.array([1.0, 1.0, 1.0]) / np.sqrt(3.0), 1.23)
    v = np.array([3.0, -1.0, 2.5])
    rotated = quat_rotate_vector(q, v)
    back = quat_rotate_vector(quat_inverse(q), rotated)
    np.testing.assert_allclose(back, v, atol=1e-10)


def test_identity_multiply():
    q = quat_from_axis_angle(np.array([0.0, 0.0, 1.0]), 0.5)
    result = quat_multiply(quat_identity(), q)
    np.testing.assert_allclose(result, q, atol=1e-10)


def test_zero_angle_identity():
    q = quat_from_axis_angle(np.array([1.0, 0.0, 0.0]), 0.0)
    np.testing.assert_allclose(q, quat_identity(), atol=1e-10)


def test_zero_axis_identity():
    q = quat_from_axis_angle(np.zeros(3), 1.0)
    np.testing.assert_allclose(q, quat_identity(), atol=1e-10)


def test_conjugate_negates_vector_part():
    q = np.array([1.0, 2.0, -3.0, 4.0])
    result = quat_conjugate(q)
    np.testing.assert_allclose(result, [1.0, -2.0, 3.0, -4.0], atol=1e-10)
