"""
kinematics/parameters.py — the gait parameters clinicians and researchers report.

Turns the per-joint flexion traces and the foot gait events into standard parameters. Every
quantity here is a literature-grounded measure obtainable from a 4-IMU single-leg rig
(pelvis + thigh + shank + foot); none needs optical/marker reference.

PER JOINT (hip / knee / ankle)
  * range of motion (ROM), peak flexion / extension, peak & mean angular velocity, cycle count.

TEMPORAL (steady straight walking)
  * cadence (steps/min), stride time & step time (mean ± SD),
  * stance / swing as % of the gait cycle (Perry & Burnfield norms: ~60 / 40),
  * stride-time variability (coefficient of variation) — a validated fall-risk / gait-quality
    marker (Hausdorff, J. NeuroEng. Rehabil. 2005).

SPATIAL (estimate)
  * stride length & walking speed via foot-IMU zero-velocity-update (ZUPT) double integration
    (Mariani et al., J. Biomech. 2010). Reported as an ESTIMATE: it is the least robust output
    of an IMU-only rig and depends on a clean foot-flat phase.
"""
from __future__ import annotations

import numpy as np
from scipy.signal import find_peaks

from .joint_angles import rom
from .quaternion import rotate_to_world

GRAVITY = 9.81


# --------------------------------------------------------------------------- #
def joint_parameters(flexion, ang_vel, mask, fs, *, min_cycle_s=0.6, active_vel_dps=10.0) -> dict:
    """Per-joint parameters from a flexion trace (deg) and its angular velocity (deg/s).

    ``mask`` is the steady-state boolean mask; steady quantities use it, full-bout quantities
    use the whole trace.
    """
    flexion = np.asarray(flexion, float)
    ang_vel = np.asarray(ang_vel, float)
    steady = flexion[mask] if mask.any() else flexion
    vel_steady = ang_vel[mask] if mask.any() else ang_vel

    dist = max(1, int(min_cycle_s * fs))
    peaks, _ = find_peaks(steady, distance=dist) if len(steady) > dist else (np.array([]), {})
    active_s = float(np.count_nonzero(np.abs(vel_steady) > active_vel_dps) / fs) if fs else 0.0

    return {
        "rom_deg": float(rom(steady)) if len(steady) else float("nan"),
        "rom_deg_full": float(rom(flexion)),
        "peak_flexion_deg": float(np.nanmax(flexion)),
        "peak_extension_deg": float(np.nanmin(flexion)),
        "peak_abs_vel_dps": float(np.nanmax(np.abs(ang_vel))) if len(ang_vel) else float("nan"),
        "mean_abs_vel_dps": float(np.nanmean(np.abs(vel_steady))) if len(vel_steady) else float("nan"),
        "cycle_count": int(len(peaks)),
        "active_duration_s": active_s,
    }


# --------------------------------------------------------------------------- #
def temporal_parameters(ev, mask, fs, *, max_stride_s=2.5) -> dict:
    """Bout-level temporal parameters from foot events restricted to steady strides."""
    strikes = np.asarray(ev.get("foot_strike", []), int)
    strikes = strikes[strikes < len(mask)]
    steady_strikes = strikes[mask[strikes]] if len(strikes) else strikes

    st = np.diff(steady_strikes) / fs if len(steady_strikes) > 1 else np.array([])
    st = st[st < max_stride_s]                          # drop turn gaps
    if len(st):
        stride_mean, stride_std = float(np.mean(st)), float(np.std(st))
        cadence = 60.0 / stride_mean * 2.0              # both feet ≈ 2 × single-foot stride rate
        cv = 100.0 * stride_std / stride_mean
    else:
        stride_mean = stride_std = cadence = cv = float("nan")

    stance, swing = _stance_swing(ev, mask, fs, max_stride_s)
    return {
        "cadence_steps_per_min": cadence,
        "stride_time_mean_s": stride_mean,
        "stride_time_std_s": stride_std,
        "stride_time_cv_pct": cv,
        "step_time_s": (stride_mean / 2.0) if stride_mean == stride_mean else float("nan"),
        "stance_pct": stance,
        "swing_pct": swing,
        "n_steady_strides": int(len(st)),
        "n_foot_strikes_total": int(len(strikes)),
    }


def _stance_swing(ev, mask, fs, max_stride_s):
    """Stance/swing %: stance = strike→toe-off, swing = toe-off→next strike (steady strides)."""
    strikes = np.asarray(ev.get("foot_strike", []), int)
    toe = np.asarray(ev.get("toe_off", []), int)
    if len(strikes) < 2 or len(toe) < 1:
        return float("nan"), float("nan")
    swing_fracs, n = [], len(mask)
    for a, b in zip(strikes[:-1], strikes[1:]):
        if b - a > int(max_stride_s * fs) or b >= n or not mask[a:b].all():
            continue
        t = toe[(toe > a) & (toe < b)]
        if len(t) == 0:
            continue
        swing = (b - t[-1]) / (b - a)
        if 0.2 < swing < 0.6:                           # physiologic swing fraction
            swing_fracs.append(swing)
    if not swing_fracs:
        return float("nan"), float("nan")
    sw = 100.0 * float(np.mean(swing_fracs))
    return 100.0 - sw, sw


# --------------------------------------------------------------------------- #
def stride_length_zupt(q_foot, acc_foot, gyr_foot, ev, mask, fs, *, max_stride_s=2.5) -> dict:
    """Estimate stride length & walking speed by foot-IMU ZUPT double integration.

    The world-frame linear acceleration (gravity removed via the VQF quaternion) is integrated
    twice. The zero-velocity updates are placed at **mid-stance** (the foot-flat instant of
    least angular velocity in each stride), where the foot is genuinely stationary; integrating
    between consecutive mid-stance anchors and forcing the velocity to zero at both ends
    de-drifts each stride. The horizontal displacement magnitude is the stride length (Mariani
    et al., J. Biomech. 2010). Reported as an ESTIMATE — the least robust IMU-only output.
    """
    strikes = np.asarray(ev.get("foot_strike", []), int)
    strikes = strikes[strikes < len(mask)]
    a_world = rotate_to_world(np.asarray(q_foot, float), np.asarray(acc_foot, float))
    a_lin = a_world - np.array([0.0, 0.0, GRAVITY])     # remove gravity in earth frame
    gm = np.linalg.norm(np.asarray(gyr_foot, float), axis=1)
    dt = 1.0 / fs

    # Mid-stance anchor per stride = least-rotation (foot-flat) sample in its first 60 %.
    anchors = []
    for a, b in zip(strikes[:-1], strikes[1:]):
        if b - a < 5 or b - a > int(max_stride_s * fs):
            continue
        anchors.append(a + int(np.argmin(gm[a:a + int(0.6 * (b - a))])))

    lengths = []
    for a, b in zip(anchors[:-1], anchors[1:]):
        if b - a > int(max_stride_s * fs) or b >= len(mask) or not mask[a:b].all():
            continue
        v = np.cumsum(a_lin[a:b], axis=0) * dt
        v -= v[0]                                          # ZUPT: v(start)=0
        v -= np.linspace(0, 1, len(v))[:, None] * v[-1]    # de-drift so v(end)=0
        p = np.cumsum(v, axis=0) * dt
        lengths.append(float(np.hypot(p[-1, 0], p[-1, 1])))  # horizontal displacement
    if not lengths:
        return {"stride_length_m_est": float("nan"), "walking_speed_mps_est": float("nan"),
                "n_strides_spatial": 0}
    lengths = np.array(lengths)
    stride_mean = float(np.mean(lengths))
    steady_strikes = strikes[mask[strikes]]
    st = np.diff(steady_strikes) / fs
    st = st[(st > 0) & (st < max_stride_s)]
    speed = stride_mean / float(np.mean(st)) if len(st) else float("nan")
    return {
        "stride_length_m_est": stride_mean,
        "stride_length_std_m_est": float(np.std(lengths)),
        "walking_speed_mps_est": speed,
        "n_strides_spatial": int(len(lengths)),
    }


# --------------------------------------------------------------------------- #
def overlay_cycles(flexion, strikes, steady, fs, *, n_points=101, max_cycles=80):
    """Resample each steady stride of a flexion trace to 0–100 % of the gait cycle.

    Returns ``(grid, cycles, mean, std)`` for the cycle-overlay plot and cycle-averaged ROM.
    """
    flexion = np.asarray(flexion, float)
    strikes = np.asarray(strikes, int)
    grid = np.linspace(0, 100, n_points)
    cycles = []
    for a, b in zip(strikes[:-1], strikes[1:]):
        if b - a < 5 or b - a > int(2.5 * fs) or b > len(flexion) or not steady[a:b].all():
            continue
        seg = flexion[a:b]
        cycles.append(np.interp(grid, np.linspace(0, 100, len(seg)), seg))
        if len(cycles) >= max_cycles:
            break
    if not cycles:
        return grid, [], None, None
    arr = np.array(cycles)
    return grid, cycles, arr.mean(0), arr.std(0)
