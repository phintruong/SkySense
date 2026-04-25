# Agent 2 Instructions

## Assignment
- **Assigned Tool:** Codex
- **Wave:** 1
- **Role:** Quaternion math library, rotation utilities, and comprehensive unit tests

## Objective
Implement the core math utilities that the entire GNC stack depends on: quaternion operations and rotation conversions. These must be correct — a sign error here propagates to every downstream module (EKF, drone model, attitude controller). Every function must have thorough unit tests.

## Context

This is a drone GNC project using:
- **NED coordinate system**: X=North, Y=East, Z=Down. Gravity = [0, 0, +9.81].
- **Quaternion convention**: [qw, qx, qy, qz] (scalar-first, Hamilton convention)
- **Euler convention**: 3-2-1 rotation order (ZYX). Roll=phi (about X), Pitch=theta (about Y), Yaw=psi (about Z).
- **numpy** arrays throughout. Quaternions are shape (4,), vectors are shape (3,), matrices are (3,3).

Critical correctness requirements:
- Hamilton product (NOT JPL convention): `q1 * q2` where `i*j=k` (not `i*j=-k`)
- NED Euler: `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)` transforms body to NED
- Specific force in hover: IMU reads `[0, 0, -g]` in body frame (accelerometer measures specific force = acceleration - gravity, in NED gravity is +Z, so body-frame specific force is -g on Z when hovering)

## Tasks

### Task 1.2a: Quaternion Operations
- **Description:** Create `src/math_utils/quaternion.py` with the following functions. All operate on numpy arrays. All handle edge cases (zero vectors, non-unit quaternions).

```python
import numpy as np

def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """
    Hamilton quaternion product: q1 * q2.
    q = [qw, qx, qy, qz]
    
    Formula:
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    """

def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Return [qw, -qx, -qy, -qz]."""

def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize to unit quaternion. Handle zero-norm gracefully (return identity)."""

def quat_inverse(q: np.ndarray) -> np.ndarray:
    """Inverse quaternion. For unit quaternions, same as conjugate."""

def quat_rotate_vector(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """
    Rotate 3D vector v by quaternion q.
    v' = q * [0, v] * q_conj
    
    Efficient formula (avoids full quaternion multiply):
    t = 2 * cross(q_xyz, v)
    v' = v + qw * t + cross(q_xyz, t)
    """

def quat_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    """
    Create quaternion from axis-angle representation.
    q = [cos(angle/2), sin(angle/2) * axis_normalized]
    Handle zero angle (return identity) and zero axis.
    """

def quat_identity() -> np.ndarray:
    """Return identity quaternion [1, 0, 0, 0]."""
```

- **Acceptance criteria:**
  - [ ] All 7 functions implemented
  - [ ] Type hints on all functions
  - [ ] Edge cases handled (zero vectors, non-unit inputs)
  - [ ] No comments except where the math is non-obvious
- **Produces:** Quaternion math used by rotations.py, drone_model.py, ekf.py

### Task 1.2b: Rotation Conversions
- **Description:** Create `src/math_utils/rotations.py` with rotation conversion functions.

```python
import numpy as np
from .quaternion import quat_multiply, quat_conjugate, quat_normalize, quat_rotate_vector

def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """
    Convert quaternion to Euler angles [roll, pitch, yaw].
    NED convention, 3-2-1 (ZYX) rotation order.
    
    roll  = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2))
    pitch = asin(clip(2*(qw*qy - qz*qx), -1, 1))
    yaw   = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
    
    Returns angles in radians. Pitch clamped to [-pi/2, pi/2].
    """

def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Convert Euler angles to quaternion.
    NED convention, 3-2-1 (ZYX) rotation order.
    
    cr, sr = cos(roll/2), sin(roll/2)
    cp, sp = cos(pitch/2), sin(pitch/2)
    cy, sy = cos(yaw/2), sin(yaw/2)
    
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    """

def quat_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """
    Convert quaternion to 3x3 rotation matrix (DCM).
    R transforms body-frame vectors to NED-frame: v_ned = R @ v_body
    
    Standard formula using quaternion components.
    """

def rotation_matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """
    Convert 3x3 rotation matrix to quaternion.
    Uses Shepperd's method for numerical stability.
    Always returns quaternion with positive qw (canonical form).
    """

def body_to_ned(q: np.ndarray, v_body: np.ndarray) -> np.ndarray:
    """Rotate body-frame vector to NED frame using quaternion."""
    return quat_rotate_vector(q, v_body)

def ned_to_body(q: np.ndarray, v_ned: np.ndarray) -> np.ndarray:
    """Rotate NED-frame vector to body frame using quaternion."""
    return quat_rotate_vector(quat_conjugate(q), v_ned)
```

- **Acceptance criteria:**
  - [ ] All 6 functions implemented
  - [ ] body_to_ned and ned_to_body are inverses of each other
  - [ ] Euler convention matches NED 3-2-1 exactly
  - [ ] Gimbal lock (pitch = +-90 deg) doesn't crash (asin clamped)
- **Produces:** Rotation conversions used everywhere

### Task 1.2c: Module Init + Exports
- **Description:** Create `src/math_utils/__init__.py` that exports all public functions.

```python
from .quaternion import (
    quat_multiply,
    quat_conjugate,
    quat_normalize,
    quat_inverse,
    quat_rotate_vector,
    quat_from_axis_angle,
    quat_identity,
)
from .rotations import (
    quat_to_euler,
    euler_to_quat,
    quat_to_rotation_matrix,
    rotation_matrix_to_quat,
    body_to_ned,
    ned_to_body,
)
```

- **Acceptance criteria:**
  - [ ] `from src.math_utils import quat_multiply, quat_to_euler, body_to_ned` works

### Task 1.2d: Quaternion Tests
- **Description:** Create `tests/test_quaternion.py` with comprehensive tests.

```python
import numpy as np
import pytest
from src.math_utils import (
    quat_multiply, quat_conjugate, quat_normalize, quat_inverse,
    quat_rotate_vector, quat_from_axis_angle, quat_identity,
)

# Test identity
def test_identity_rotation():
    """Identity quaternion does not change a vector."""
    v = np.array([1.0, 2.0, 3.0])
    result = quat_rotate_vector(quat_identity(), v)
    np.testing.assert_allclose(result, v, atol=1e-10)

# Test 90-degree rotations about each axis
def test_rotate_90_about_z():
    """90 deg rotation about Z: [1,0,0] -> [0,1,0]."""
    q = quat_from_axis_angle(np.array([0,0,1]), np.pi/2)
    result = quat_rotate_vector(q, np.array([1,0,0]))
    np.testing.assert_allclose(result, [0, 1, 0], atol=1e-10)

def test_rotate_90_about_x():
    """90 deg rotation about X: [0,1,0] -> [0,0,1]."""
    q = quat_from_axis_angle(np.array([1,0,0]), np.pi/2)
    result = quat_rotate_vector(q, np.array([0,1,0]))
    np.testing.assert_allclose(result, [0, 0, 1], atol=1e-10)

def test_rotate_90_about_y():
    """90 deg rotation about Y: [0,0,1] -> [1,0,0]."""
    # CAREFUL: right-hand rule. Y rotation: Z -> X
    q = quat_from_axis_angle(np.array([0,1,0]), np.pi/2)
    result = quat_rotate_vector(q, np.array([0,0,1]))
    np.testing.assert_allclose(result, [1, 0, 0], atol=1e-10)

# Test multiply is associative
def test_multiply_associative():
    q1 = quat_from_axis_angle(np.array([1,0,0]), 0.3)
    q2 = quat_from_axis_angle(np.array([0,1,0]), 0.5)
    q3 = quat_from_axis_angle(np.array([0,0,1]), 0.7)
    left = quat_multiply(quat_multiply(q1, q2), q3)
    right = quat_multiply(q1, quat_multiply(q2, q3))
    np.testing.assert_allclose(left, right, atol=1e-10)

# Test normalize
def test_normalize_produces_unit():
    q = np.array([2.0, 1.0, 0.5, 0.3])
    result = quat_normalize(q)
    assert abs(np.linalg.norm(result) - 1.0) < 1e-10

# Test inverse rotation round-trip
def test_rotate_then_inverse():
    q = quat_from_axis_angle(np.array([1, 1, 1]) / np.sqrt(3), 1.23)
    v = np.array([3.0, -1.0, 2.5])
    rotated = quat_rotate_vector(q, v)
    back = quat_rotate_vector(quat_inverse(q), rotated)
    np.testing.assert_allclose(back, v, atol=1e-10)

# Test identity multiply
def test_identity_multiply():
    q = quat_from_axis_angle(np.array([0,0,1]), 0.5)
    result = quat_multiply(quat_identity(), q)
    np.testing.assert_allclose(result, q, atol=1e-10)

# Test zero angle gives identity
def test_zero_angle_identity():
    q = quat_from_axis_angle(np.array([1,0,0]), 0.0)
    np.testing.assert_allclose(q, quat_identity(), atol=1e-10)

# Add more tests for edge cases as you see fit
```

- **Acceptance criteria:**
  - [ ] All tests pass with `pytest tests/test_quaternion.py -v`
  - [ ] At least 8 tests covering: identity, 90-deg rotations (3 axes), associativity, normalize, round-trip, zero angle

### Task 1.2e: Rotation Tests
- **Description:** Create `tests/test_rotations.py` with comprehensive tests.

```python
import numpy as np
import pytest
from src.math_utils import (
    quat_to_euler, euler_to_quat, quat_to_rotation_matrix,
    rotation_matrix_to_quat, body_to_ned, ned_to_body,
    quat_identity, quat_from_axis_angle,
)

def test_identity_euler():
    """Identity quaternion -> zero Euler angles."""
    euler = quat_to_euler(quat_identity())
    np.testing.assert_allclose(euler, [0, 0, 0], atol=1e-10)

def test_euler_round_trip():
    """euler_to_quat -> quat_to_euler recovers original angles."""
    for roll, pitch, yaw in [(0.3, 0.2, 0.1), (-0.5, 0.4, -1.0), (0, 0, np.pi)]:
        q = euler_to_quat(roll, pitch, yaw)
        recovered = quat_to_euler(q)
        np.testing.assert_allclose(recovered, [roll, pitch, yaw], atol=1e-10)

def test_90_pitch():
    """90 degree pitch -> known quaternion."""
    q = euler_to_quat(0, np.pi/2, 0)
    euler = quat_to_euler(q)
    np.testing.assert_allclose(euler[1], np.pi/2, atol=1e-10)

def test_gimbal_lock():
    """Pitch at +-90 degrees doesn't crash."""
    q = euler_to_quat(0.1, np.pi/2, 0.2)
    euler = quat_to_euler(q)
    assert not np.any(np.isnan(euler))

def test_body_to_ned_identity():
    """Identity quaternion: body_to_ned is passthrough."""
    v = np.array([1.0, 2.0, 3.0])
    result = body_to_ned(quat_identity(), v)
    np.testing.assert_allclose(result, v, atol=1e-10)

def test_body_ned_round_trip():
    """body_to_ned then ned_to_body returns original vector."""
    q = euler_to_quat(0.3, 0.2, 0.1)
    v = np.array([1.0, -0.5, 2.0])
    v_ned = body_to_ned(q, v)
    v_back = ned_to_body(q, v_ned)
    np.testing.assert_allclose(v_back, v, atol=1e-10)

def test_rotation_matrix_round_trip():
    """quat -> matrix -> quat round-trip."""
    q = euler_to_quat(0.5, -0.3, 1.2)
    R = quat_to_rotation_matrix(q)
    q_back = rotation_matrix_to_quat(R)
    # Quaternions q and -q represent the same rotation
    if q_back[0] * q[0] < 0:
        q_back = -q_back
    np.testing.assert_allclose(q_back, q, atol=1e-10)

def test_rotation_matrix_is_orthogonal():
    """Rotation matrix should be orthogonal: R @ R.T = I."""
    q = euler_to_quat(0.5, -0.3, 1.2)
    R = quat_to_rotation_matrix(q)
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)

def test_ned_gravity_in_body():
    """Hovering drone: gravity [0,0,9.81] in NED rotated to body should match expected."""
    # No rotation: body gravity same as NED gravity
    g_ned = np.array([0, 0, 9.81])
    g_body = ned_to_body(quat_identity(), g_ned)
    np.testing.assert_allclose(g_body, g_ned, atol=1e-10)
    
    # 30 degree pitch forward: gravity should have -X component in body
    q = euler_to_quat(0, np.radians(30), 0)
    g_body = ned_to_body(q, g_ned)
    assert g_body[0] < 0  # gravity tilts into -X body when pitched forward
    assert g_body[2] > 0  # Z component still positive (down)
```

- **Acceptance criteria:**
  - [ ] All tests pass with `pytest tests/test_rotations.py -v`
  - [ ] At least 9 tests covering: identity, round-trips, gimbal lock, matrix orthogonality, NED gravity

## Scope

### Files you OWN (you may create, modify, delete):
- `src/math_utils/__init__.py`
- `src/math_utils/quaternion.py`
- `src/math_utils/rotations.py`
- `tests/test_quaternion.py`
- `tests/test_rotations.py`

### Files you must AVOID (owned by other agents):
- `src/config/` (Agent 1)
- `src/simulation/` (Agent 1)
- `src/sensors/` (Agent 1)
- `src/navigation/` (Agent 1)
- `src/control/` (Agent 1)
- `src/telemetry/` (Agent 1)
- `src/server/` (Agent 1)
- `tests/conftest.py` (Agent 1)
- `requirements.txt` (Agent 1)
- `legacy/` (Agent 1)

### Files you may READ but not modify:
- `Goal.md`
- `.orchestrator/plan.md`

## Dependencies

### Before you start:
- Agent 1 should have created the `src/math_utils/` directory. If it doesn't exist yet, create it yourself (mkdir is idempotent).
- numpy must be available. Install with: `pip install numpy`

### What depends on YOUR output:
- **Wave 2, Task 2.1 (Drone Dynamics)**: imports `body_to_ned`, `quat_multiply`, `quat_normalize`, `quat_rotate_vector`
- **Wave 2, Task 2.3 (Sensor Sim)**: imports `ned_to_body` for IMU gravity rotation
- **Wave 3, Task 3.1 (EKF)**: imports everything — quaternion integration, rotations, body/NED conversions
- Getting these functions wrong breaks the entire project. Test thoroughly.

## When Blocked
If you encounter a blocker:
1. Document it in your status file at `.orchestrator/status/agent_2_status.md`
2. Continue with any unblocked tasks in your assignment
3. Do NOT modify files outside your scope to work around the blocker

## Status Updates
When you complete a task or hit a blocker, write to `.orchestrator/status/agent_2_status.md` using this format:

```
## Task 1.2a: Quaternion Operations
- status: DONE | IN_PROGRESS | BLOCKED
- blocker: <description, or "none">
- notes: <decisions made, things the next wave should know>
```

## Definition of Done
This agent's work is complete when:
- [ ] `src/math_utils/quaternion.py` has all 7 functions implemented
- [ ] `src/math_utils/rotations.py` has all 6 functions implemented
- [ ] `src/math_utils/__init__.py` exports all public functions
- [ ] `pytest tests/test_quaternion.py -v` passes (8+ tests)
- [ ] `pytest tests/test_rotations.py -v` passes (9+ tests)
- [ ] All tasks are marked DONE in the status file
- [ ] No files outside the owned scope were modified
