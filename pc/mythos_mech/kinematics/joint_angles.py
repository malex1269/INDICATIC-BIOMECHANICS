"""
kinematics/joint_angles.py — sagittal joint flexion from per-segment VQF orientations.

This is the heart of the library's *kinematic* output. Given two adjacent segments' VQF
orientation quaternions (sensor->earth), it returns the **sagittal flexion** of the joint
between them, over time, in degrees.

METHOD (and why it is defensible without optical reference or sensor-to-segment calibration)
--------------------------------------------------------------------------------------------
* **Functional joint axis.** The mediolateral (flexion) axis of each segment is the
  largest-variance direction of its gyroscope during walking — the axis the limb actually
  rotates about. This is the gyroscope functional-axis identification of *Seel, Raisch & Schauer
  (Sensors 2014)*, which validated IMU sagittal joint angles to within a few degrees of optical
  motion capture. No physical alignment of the sensor to the bone is required.
* **Gravity-projection (yaw-immune, drift-free) angle.** Each segment's tilt is taken from the
  fused gravity direction (``gravity_in_sensor`` of the VQF quaternion). The sagittal rotation is
  the signed angle this gravity vector has swept *about the joint axis*, relative to a neutral
  (standing) reference. Because it uses only gravity-referenced tilt, it is immune to heading
  (yaw) drift — so a magnetometer is **not** needed and an uncalibrated/disturbed one cannot
  corrupt flexion. This is why VQF **6D** is the default upstream.
* **Joint flexion = distal tilt − proximal tilt** about the (sign-aligned) joint axes.
* **Complementary refinement (optional).** The drift-free gravity angle slightly lags very fast
  motion; blending in the gyro-derived joint rate at high frequency (a complementary filter)
  sharpens swing without re-introducing drift. The gravity-only trace is always kept alongside.

SIGN / OFFSET
-------------
Flexion is reported **relative to the neutral standing pose** (≈0° at quiet stance) with the
sign chosen so the dominant gait deflection is positive (flexion). An *absolute* anatomical
offset would require a calibration pose — which is exactly what OpenSim OpenSense's IMU Placer
provides; that absolute path is kept separate (``opensim_export``).
"""
from __future__ import annotations

import numpy as np

from .quaternion import gravity_in_sensor


# --------------------------------------------------------------------------- #
def neutral_gravity(acc: np.ndarray, gyr: np.ndarray, fs: float, win_s: float = 0.5) -> np.ndarray:
    """Sensor-frame 'up' at the neutral pose = mean accel over the quietest ``win_s`` window.

    No dedicated static trial is assumed: the lowest-gyro-energy stretch is taken as the
    neutral reference that sets where flexion reads zero.
    """
    acc = np.asarray(acc, float)
    gmag = np.linalg.norm(np.asarray(gyr, float), axis=1)
    w = max(1, int(win_s * fs))
    if len(gmag) <= w:
        a0 = acc.mean(0)
    else:
        energy = np.convolve(gmag, np.ones(w), "valid")
        i = int(np.argmin(energy))
        a0 = acc[i:i + w].mean(0)
    return a0 / (np.linalg.norm(a0) + 1e-12)


def functional_axis(gyr: np.ndarray) -> np.ndarray:
    """Mediolateral (flexion) axis of a segment = largest-variance gyroscope direction.

    Seel et al. (2014) functional axis: the principal eigenvector of the gyro covariance is the
    axis the segment predominantly rotates about during gait (the sagittal/flexion axis).
    """
    gyr = np.asarray(gyr, float)
    C = gyr.T @ gyr
    _w, V = np.linalg.eigh(C)
    return V[:, -1]


def _sagittal_rotation(grav: np.ndarray, grav0: np.ndarray, axis: np.ndarray) -> np.ndarray:
    """Signed angle (rad) gravity has rotated about ``axis`` from neutral ``grav0``.

    Yaw-immune and integration-free: projects gravity onto the plane perpendicular to the joint
    axis and measures its rotation there.
    """
    j = axis / (np.linalg.norm(axis) + 1e-12)
    g0 = grav0 - (grav0 @ j) * j
    g0 = g0 / (np.linalg.norm(g0) + 1e-12)
    gp = grav - (grav @ j)[:, None] * j
    gp = gp / (np.linalg.norm(gp, axis=1, keepdims=True) + 1e-12)
    sin = np.cross(np.broadcast_to(g0, gp.shape), gp) @ j
    cos = gp @ g0
    return np.arctan2(sin, cos)


def _complementary(theta_lf: np.ndarray, rate: np.ndarray, fs: float, tau: float) -> np.ndarray:
    """Complementary filter: high-pass gyro integral + low-pass gravity-based angle."""
    dt = 1.0 / fs
    a = tau / (tau + dt)
    out = np.empty_like(theta_lf)
    out[0] = theta_lf[0]
    for i in range(1, len(theta_lf)):
        out[i] = a * (out[i - 1] + rate[i] * dt) + (1 - a) * theta_lf[i]
    return out


# --------------------------------------------------------------------------- #
def joint_flexion(q_dist, q_prox, gyr_dist, gyr_prox, grav0_dist, grav0_prox,
                  *, axis_mask=None, fs: float = 256.0, tau: float = 0.0) -> dict:
    """Sagittal joint flexion (deg) over time — yaw-immune, drift-free.

    ``q_*`` are sensor->earth VQF quaternions (N,4); ``gyr_*`` the matching gyro (N,3, rad/s);
    ``grav0_*`` each segment's neutral 'up' (3,). ``axis_mask`` restricts functional-axis
    estimation to steady straight-walking samples (turns excluded) so the proximal axis is the
    mediolateral one, not a turn (yaw) axis.

    The primary ``flexion`` is the **gravity-projection** angle: drift-free and purely sagittal,
    which VQF's bias-corrected orientation makes accurate without any gyro integration. A
    complementary refinement is available (``tau`` > 0) for sharpening very fast swing, but it is
    OFF by default because integrating the gyro can leak out-of-plane motion into the angle.

    Returns ``flexion`` (primary), ``flexion_gravity_only`` (always the drift-free trace),
    ``joint_rate_dps`` and the two joint axes.
    """
    gd_axis = gyr_dist if axis_mask is None else gyr_dist[axis_mask]
    gp_axis = gyr_prox if axis_mask is None else gyr_prox[axis_mask]
    jd = functional_axis(gd_axis)
    jp = functional_axis(gp_axis)

    # Sign-align the two joint axes so distal and proximal sagittal rates co-vary positively.
    rd, rp = gyr_dist @ jd, gyr_prox @ jp
    if np.corrcoef(rd, rp)[0, 1] < 0:
        jp = -jp

    gd = gravity_in_sensor(q_dist)
    gp = gravity_in_sensor(q_prox)
    th_d = _sagittal_rotation(gd, grav0_dist / np.linalg.norm(grav0_dist), jd)
    th_p = _sagittal_rotation(gp, grav0_prox / np.linalg.norm(grav0_prox), jp)

    flex_grav = np.unwrap(th_d - th_p)              # drift-free, purely sagittal
    joint_rate = gyr_dist @ jd - gyr_prox @ jp      # rad/s
    flex = _complementary(flex_grav, joint_rate, fs, tau) if tau and tau > 0 else flex_grav

    # Orient sign so the dominant gait deflection (flexion) is positive (clinical convention).
    if np.mean((flex_grav - np.median(flex_grav)) ** 3) < 0:
        flex, flex_grav, joint_rate = -flex, -flex_grav, -joint_rate

    return {
        "flexion": np.degrees(flex),
        "flexion_gravity_only": np.degrees(flex_grav),
        "joint_rate_dps": np.degrees(joint_rate),
        "joint_axis_distal": jd,
        "joint_axis_proximal": jp,
    }


def derivative(x: np.ndarray, fs: float) -> np.ndarray:
    """Central finite difference (per second), same length."""
    return np.gradient(np.asarray(x, float), 1.0 / fs)


def rom(x: np.ndarray) -> float:
    """Range of motion = max − min (ignoring NaNs)."""
    return float(np.nanmax(x) - np.nanmin(x))
