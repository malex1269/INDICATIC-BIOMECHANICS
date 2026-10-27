"""
kinematics/quaternion.py — small quaternion utilities (scalar-first [w, x, y, z]).

The whole pipeline speaks VQF's convention: **sensor -> earth, scalar-first**. These are the
only quaternion operations the kinematic layer needs; orientation *estimation* itself is done
by VQF (``core.fusion_vqf``), never here.
"""
from __future__ import annotations

import numpy as np


def q_conj(q: np.ndarray) -> np.ndarray:
    """Conjugate (= inverse for unit quaternions) of (N,4) scalar-first quaternions."""
    q = np.asarray(q, float)
    out = q.copy()
    out[..., 1:] *= -1.0
    return out


def q_mult(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a ⊗ b for (N,4) scalar-first quaternion arrays."""
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    aw, ax, ay, az = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    bw, bx, by, bz = b[..., 0], b[..., 1], b[..., 2], b[..., 3]
    return np.stack([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], axis=-1)


def q_norm(q: np.ndarray) -> np.ndarray:
    """Renormalise (N,4) quaternions to unit length."""
    q = np.asarray(q, float)
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    return q / np.clip(n, 1e-12, None)


def gravity_in_sensor(q: np.ndarray) -> np.ndarray:
    """Earth 'up' [0,0,1] expressed in each sensor frame for q (N,4) -> (N,3).

    For a sensor->earth quaternion q, ``grav_sensor = R(q)^T @ [0,0,1]`` is the third ROW of
    R(q). This is the fused, drift-corrected tilt direction — the basis of the yaw-immune
    sagittal joint angle.
    """
    q = q_norm(q)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.column_stack([2 * (x * z - w * y),
                            2 * (y * z + w * x),
                            1 - 2 * (x * x + y * y)])


def rotate_to_world(q: np.ndarray, v_sensor: np.ndarray) -> np.ndarray:
    """Rotate per-sample sensor-frame vectors (N,3) into the earth frame by q (N,4).

    ``v_earth = R(q) @ v_sensor`` — used to express foot acceleration in the world frame for
    the (optional) ZUPT stride-length estimate.
    """
    q = q_norm(q)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    vx, vy, vz = v_sensor[:, 0], v_sensor[:, 1], v_sensor[:, 2]
    # v + 2*q_w*(q_vec x v) + 2*q_vec x (q_vec x v)
    tx = 2 * (y * vz - z * vy)
    ty = 2 * (z * vx - x * vz)
    tz = 2 * (x * vy - y * vx)
    return np.column_stack([vx + w * tx + (y * tz - z * ty),
                            vy + w * ty + (z * tx - x * tz),
                            vz + w * tz + (x * ty - y * tx)])


def mean_quat(q: np.ndarray) -> np.ndarray:
    """Average of a (M,4) quaternion block: hemisphere-align to the first, mean, renormalise."""
    q = np.asarray(q, float)
    aligned = np.where((q @ q[0])[:, None] < 0, -q, q)
    m = aligned.mean(axis=0)
    return m / (np.linalg.norm(m) + 1e-12)
