"""
core/fusion_vqf.py — Orientation fusion with VQF (Laidig & Seel 2023).

This module replaces the project's former home-grown Madgwick filter with **VQF**, a
peer-reviewed, validated orientation estimator:

    D. Laidig and T. Seel. "VQF: Highly Accurate IMU Orientation Estimation with Bias
    Estimation and Magnetic Disturbance Rejection." Information Fusion 2023, 91, 187-204.
    doi:10.1016/j.inffus.2022.10.014

Each segment's raw accel/gyro(/mag) is fused **independently** into a world-frame orientation
quaternion. VQF performs rest detection, gyroscope-bias estimation, and (in 9D) magnetic
disturbance rejection internally — none of which we re-implement.

WHAT VQF OUTPUTS (verified empirically, see docs/method.md)
----------------------------------------------------------
``quat6D`` / ``quat9D`` are **sensor -> earth, scalar-first [w, x, y, z]**: rotating the
measured gravity vector by the quaternion yields earth +Z (up). That is exactly the
convention OpenSim OpenSense consumes (the IMU-axes-to-model-axes rotation is handled inside
OpenSim via ``sensor_to_opensim_rotations``, not here). So VQF's native output is passed
through unchanged.

6D vs 9D
--------
* **6D** (``gyr+acc``, magnetometer-free): tilt (roll/pitch) is gravity-referenced and
  absolute; heading is gyro-integrated with bias correction — robust, needs NO magnetometer
  calibration. This is the **default**, because it cannot be corrupted by an uncalibrated or
  locally disturbed magnetometer.
* **9D** (``gyr+acc+mag``): heading is additionally referenced to magnetic north, giving a
  consistent absolute heading shared by every segment. Use it when the magnetometers are
  calibrated and the environment is magnetically clean.

OFFLINE vs MULTI-RATE
---------------------
* When a node's channels share one rate (the usual case — the hub timebase makes accel, gyro
  and magnetometer one synchronous stream), we use :func:`vqf.offlineVQF`, the acausal
  forward-backward variant — the most accurate offline estimate and free of startup
  transients (which matters for the static calibration window).
* When the magnetometer is delivered on its OWN clock (a different length / rate), we use the
  real-time :class:`vqf.VQF` with the true per-channel sample times (``gyrTs``/``accTs``/
  ``magTs``) and feed each channel at its own cadence. VQF handles the rate difference
  natively — there is **no custom resampling**.
"""
from __future__ import annotations

import numpy as np
from vqf import VQF, offlineVQF

from .config import MountingConfig
from .rawdata import infer_rate, common_timebase


# --------------------------------------------------------------------------- #
def _resolve_mode(mode: str, mag_present: bool) -> str:
    """Resolve 'auto' to '9D' (usable magnetometer present) or '6D'."""
    if mode == "auto":
        return "9D" if mag_present else "6D"
    return mode


def _fuse_offline(gyr, acc, mag, Ts, *, want9d: bool) -> np.ndarray:
    """Acausal offline VQF for one synchronous stream -> (N,4) quaternion."""
    gyr = np.ascontiguousarray(gyr, float)
    acc = np.ascontiguousarray(acc, float)
    mag = np.ascontiguousarray(mag, float) if want9d else None
    out = offlineVQF(gyr, acc, mag, Ts)
    return out["quat9D"] if want9d else out["quat6D"]


def _fuse_multirate(gyr, acc, mag, *, gyr_ts, acc_ts, mag_ts, want9d: bool) -> np.ndarray:
    """Real-time VQF with per-channel sample times (magnetometer on its own clock).

    Feeds gyro+accel at the IMU cadence and each magnetometer sample when its own clock
    reaches the current IMU instant — VQF's native multi-rate handling, no resampling.
    Returns one quaternion per IMU sample, (N,4).
    """
    gyr = np.ascontiguousarray(gyr, float)
    acc = np.ascontiguousarray(acc, float)
    mag = np.ascontiguousarray(mag, float)
    n, m = len(gyr), len(mag)
    vqf = VQF(gyr_ts, acc_ts, mag_ts)
    t_imu = np.arange(n) * gyr_ts
    t_mag = np.arange(m) * mag_ts
    quats = np.empty((n, 4))
    j = 0
    for i in range(n):
        vqf.updateGyr(gyr[i])
        vqf.updateAcc(acc[i])
        while want9d and j < m and t_mag[j] <= t_imu[i] + 1e-9:
            vqf.updateMag(mag[j])
            j += 1
        quats[i] = vqf.getQuat9D() if want9d else vqf.getQuat6D()
    return quats


def fuse_segment(stream: dict, *, mode: str = "6D", imu_hz: float = 100.0,
                 mag_hz: float | None = None, n: int | None = None) -> tuple[np.ndarray, str]:
    """Fuse one node's stream into (N,4) sensor->earth quaternions.

    ``stream`` is ``{"t","acc","gyr","mag","mag_present"[, "t_mag"]}``. Returns
    ``(quat (N,4), effective_mode)``. ``n`` trims to a shared length (the common timebase).
    The IMU sample time is taken from the timestamps when available, falling back to
    ``imu_hz``; a magnetometer on its own clock uses ``mag_hz`` (or its ``t_mag``).
    """
    mag_hz = imu_hz if mag_hz is None else mag_hz
    eff = _resolve_mode(mode, bool(stream.get("mag_present")))
    # 9D needs a usable magnetometer; degrade to 6D when a segment has none so we never feed
    # VQF an all-zero magnetic vector (which would normalise to NaN).
    if eff == "9D" and not stream.get("mag_present"):
        eff = "6D"
    want9d = eff == "9D"

    t = np.asarray(stream["t"], float)
    acc = np.asarray(stream["acc"], float)
    gyr = np.asarray(stream["gyr"], float)
    mag = np.asarray(stream.get("mag"), float) if stream.get("mag") is not None else None
    n = len(t) if n is None else min(n, len(t))

    acc, gyr, t = acc[:n], gyr[:n], t[:n]
    imu_ts = (1.0 / infer_rate(t)) if infer_rate(t) > 0 else (1.0 / imu_hz)

    # 6D, or 9D with the magnetometer on the same clock -> acausal offline VQF.
    t_mag = stream.get("t_mag")
    synchronous = mag is not None and t_mag is None and len(mag) >= n
    if not want9d:
        return _fuse_offline(gyr, acc, None, imu_ts, want9d=False), eff
    if synchronous:
        return _fuse_offline(gyr, acc, mag[:n], imu_ts, want9d=True), eff

    # 9D with a magnetometer on its own clock -> real-time multi-rate VQF.
    mag_ts = (1.0 / infer_rate(t_mag)) if (t_mag is not None and infer_rate(t_mag) > 0) \
        else (1.0 / mag_hz)
    return _fuse_multirate(gyr, acc, mag, gyr_ts=imu_ts, acc_ts=imu_ts, mag_ts=mag_ts,
                           want9d=True), eff


# --------------------------------------------------------------------------- #
def fuse_session(streams: dict, config: MountingConfig) -> dict:
    """Fuse every measured segment in a session onto one shared time grid.

    Only nodes declared in ``config.sensors`` are fused. Returns a dict::

        {"t": (N,) time grid (s), "fs": float sample rate (Hz),
         "orientations": {node_id: (N,4) quaternion}, "modes": {node_id: "6D"|"9D"}}

    Each quaternion is sensor->earth, scalar-first, on the common time grid.
    """
    declared = [n for n in streams if n in config.sensors]
    if not declared:
        raise ValueError(f"no declared sensor produced data (declared "
                         f"{sorted(config.sensors)}, got {sorted(streams)})")
    sel = {k: streams[k] for k in declared}
    t, n = common_timebase(sel)
    fs = infer_rate(t) or config.imu_hz

    orientations, modes = {}, {}
    for node in declared:
        q, eff = fuse_segment(sel[node], mode=config.mode, imu_hz=config.imu_hz,
                              mag_hz=config.mag_hz, n=n)
        orientations[node] = np.asarray(q, float)[:n]
        modes[node] = eff
    return {"t": t, "fs": float(fs), "orientations": orientations, "modes": modes}
