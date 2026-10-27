"""
kinematics/gait_events.py — gait-event detection and turnaround / steady-state masking.

GAIT EVENTS (foot or shank gyroscope)
-------------------------------------
Each stride shows a large **mid-swing** peak in the foot/shank sagittal angular velocity.
Initial contact (foot strike) is the sharp angular-rate reversal just **after** mid-swing;
toe-off (terminal contact) is the reversal just **before** it. This gyroscope mid-swing method
is the established ambulatory approach of *Aminian et al. (J. Biomech. 2002)* and *Salarian et
al. (IEEE TBME 2004)*, both developed on the very Physilog hardware that recorded the bundled
Geneva dataset; the foot-worn variant is used by *Mariani et al. (J. Biomech. 2010)*.

TURNAROUNDS / STEADY STATE
--------------------------
A 2-minute walk is a back-and-forth: straight passes separated by ~180° turns. Turns are
detected from the pelvis angular rate about the (gravity) vertical axis and **excluded** from
steady-state gait statistics (cadence, stride time, stance/swing, ROM), which should describe
straight walking — not the turns.
"""
from __future__ import annotations

import numpy as np
from scipy.signal import find_peaks


# --------------------------------------------------------------------------- #
def principal_axis(gyr: np.ndarray) -> np.ndarray:
    """Mediolateral (sagittal) axis = largest-variance gyroscope direction."""
    gyr = np.asarray(gyr, float)
    C = gyr.T @ gyr
    _w, V = np.linalg.eigh(C)
    return V[:, -1]


def sagittal_rate(gyr: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Foot/shank angular velocity projected on its sagittal axis, oriented so swing is +."""
    ax = principal_axis(gyr)
    s = np.asarray(gyr, float) @ ax
    if np.mean(s ** 3) < 0:                  # orient so the big mid-swing peaks are positive
        s = -s
        ax = -ax
    return s, ax


def detect_events(foot_gyr: np.ndarray, fs: float, *, min_stride_s: float = 0.6) -> dict:
    """Detect mid-swing, initial-contact (foot strike) and toe-off from a foot/shank gyro.

    Returns sample indices: ``mid_swing``, ``foot_strike``, ``toe_off`` plus the smoothed
    ``sagittal_rate`` signal the detection ran on (for plotting).
    """
    s, _ = sagittal_rate(foot_gyr)
    k = max(1, int(0.03 * fs))               # ~30 ms smoothing
    s_s = np.convolve(s, np.ones(k) / k, "same")
    dist = int(min_stride_s * fs)
    thr = 0.5 * np.percentile(np.abs(s_s), 95)
    mids, _ = find_peaks(s_s, distance=dist, height=thr)

    strikes, toeoffs = [], []
    for m in mids:
        a, b = m, min(len(s_s), m + int(0.5 * fs))      # foot strike: min within 0.5 s AFTER
        if b > a + 1:
            strikes.append(a + int(np.argmin(s_s[a:b])))
        a2, b2 = max(0, m - int(0.4 * fs)), m            # toe-off: min within 0.4 s BEFORE
        if b2 > a2 + 1:
            toeoffs.append(a2 + int(np.argmin(s_s[a2:b2])))
    return {
        "mid_swing": np.asarray(mids, int),
        "foot_strike": np.asarray(sorted(set(strikes)), int),
        "toe_off": np.asarray(sorted(set(toeoffs)), int),
        "sagittal_rate": s_s,
    }


# --------------------------------------------------------------------------- #
def _lowpass_grav(acc: np.ndarray, fs: float, fc: float = 0.4) -> np.ndarray:
    w = max(1, int(fs / fc))
    k = np.ones(w) / w
    return np.column_stack([np.convolve(acc[:, i], k, "same") for i in range(3)])


def vertical_yaw_rate(acc: np.ndarray, gyr: np.ndarray, fs: float) -> np.ndarray:
    """Angular rate about the gravity (vertical) axis in the sensor frame (rad/s)."""
    g = _lowpass_grav(np.asarray(acc, float), fs)
    gu = g / (np.linalg.norm(g, axis=1, keepdims=True) + 1e-12)
    return np.einsum("ij,ij->i", np.asarray(gyr, float), gu)


def detect_turnarounds(acc, gyr, fs, *, min_turn_deg=120.0, rate_thresh_dps=50.0,
                       smooth_s=0.5) -> tuple[list, np.ndarray]:
    """Detect ~180° turns from the pelvis vertical-axis angular rate.

    Returns ``(turns, yaw_rate_smoothed)`` with ``turns`` a list of ``(start, end, deg)``.
    """
    yr = vertical_yaw_rate(acc, gyr, fs)
    w = max(1, int(smooth_s * fs))
    yr_s = np.convolve(yr, np.ones(w) / w, "same")
    active = np.abs(yr_s) > np.radians(rate_thresh_dps)
    turns, i, n = [], 0, len(yr_s)
    while i < n:
        if active[i]:
            s, sign = i, np.sign(yr_s[i])
            while i < n and active[i] and np.sign(yr_s[i]) == sign:
                i += 1
            ang = np.degrees(np.trapezoid(yr_s[s:i], dx=1.0 / fs))
            if abs(ang) >= min_turn_deg:
                turns.append((s, i, float(ang)))
        else:
            i += 1
    return turns, yr_s


def steady_state_mask(n: int, turns: list, fs: float, *, pad_s: float = 0.7) -> np.ndarray:
    """Boolean mask of steady straight-walking samples (turns + a pad removed)."""
    m = np.ones(n, bool)
    pad = int(pad_s * fs)
    for s, e, _ in turns:
        m[max(0, s - pad):min(n, e + pad)] = False
    return m
