"""Quaternion operations using scalar-first Hamilton convention."""

import numpy as np


def quat_identity() -> np.ndarray:
    """Return identity quaternion [1, 0, 0, 0]."""
    return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton quaternion product: q1 * q2."""
    q1 = np.asarray(q1, dtype=float)
    q2 = np.asarray(q2, dtype=float)
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Return [qw, -qx, -qy, -qz]."""
    q = np.asarray(q, dtype=float)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize to unit quaternion. Return identity for zero-norm inputs."""
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if norm == 0.0 or not np.isfinite(norm):
        return quat_identity()
    return q / norm


def quat_inverse(q: np.ndarray) -> np.ndarray:
    """Return inverse quaternion."""
    q = np.asarray(q, dtype=float)
    norm_sq = float(np.dot(q, q))
    if norm_sq == 0.0 or not np.isfinite(norm_sq):
        return quat_identity()
    return quat_conjugate(q) / norm_sq


def quat_rotate_vector(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate 3D vector v by quaternion q."""
    q = quat_normalize(q)
    v = np.asarray(v, dtype=float)
    qw = q[0]
    q_xyz = q[1:]

    t = 2.0 * np.cross(q_xyz, v)
    return v + qw * t + np.cross(q_xyz, t)


def quat_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    """Create quaternion from axis-angle representation."""
    axis = np.asarray(axis, dtype=float)
    axis_norm = np.linalg.norm(axis)
    if angle == 0.0 or axis_norm == 0.0 or not np.isfinite(axis_norm):
        return quat_identity()

    half_angle = 0.5 * angle
    return quat_normalize(
        np.concatenate(
            (
                np.array([np.cos(half_angle)], dtype=float),
                np.sin(half_angle) * axis / axis_norm,
            )
        )
    )
