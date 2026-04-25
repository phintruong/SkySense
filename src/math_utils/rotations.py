"""Rotation conversion utilities for NED 3-2-1 Euler convention."""

import numpy as np

from .quaternion import quat_conjugate, quat_normalize, quat_rotate_vector


def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """Convert quaternion to Euler angles [roll, pitch, yaw] in radians."""
    qw, qx, qy, qz = quat_normalize(q)

    roll = np.arctan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
    pitch_arg = 2.0 * (qw * qy - qz * qx)
    pitch = np.arcsin(np.clip(pitch_arg, -1.0, 1.0))
    yaw = np.arctan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )

    return np.array([roll, pitch, yaw], dtype=float)


def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert Euler angles to quaternion."""
    cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
    cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
    cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)

    return quat_normalize(
        np.array(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ],
            dtype=float,
        )
    )


def quat_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """Convert quaternion to a DCM that maps body-frame vectors to NED."""
    qw, qx, qy, qz = quat_normalize(q)

    return np.array(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qw * qz),
                2.0 * (qx * qz + qw * qy),
            ],
            [
                2.0 * (qx * qy + qw * qz),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qw * qx),
            ],
            [
                2.0 * (qx * qz - qw * qy),
                2.0 * (qy * qz + qw * qx),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=float,
    )


def rotation_matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to canonical quaternion."""
    R = np.asarray(R, dtype=float)
    trace = float(np.trace(R))

    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    q = quat_normalize(np.array([qw, qx, qy, qz], dtype=float))
    if q[0] < 0.0:
        q = -q
    return q


def body_to_ned(q: np.ndarray, v_body: np.ndarray) -> np.ndarray:
    """Rotate body-frame vector to NED frame using quaternion."""
    return quat_rotate_vector(q, v_body)


def ned_to_body(q: np.ndarray, v_ned: np.ndarray) -> np.ndarray:
    """Rotate NED-frame vector to body frame using quaternion."""
    return quat_rotate_vector(quat_conjugate(q), v_ned)
