"""
kinematics — raw IMU -> gait kinematics (joint angles + gait parameters), visualised in Python.

The library's primary product. Built on validated components: VQF orientation
(``core.fusion_vqf``) feeds gravity-referenced, yaw-immune sagittal joint angles (Seel et al.
2014) and gyroscope gait-event detection (Aminian 2002 / Salarian 2004 / Mariani 2010). The
OpenSim/OpenSense export (``opensim_export``) is a separate downstream option.

    from kinematics import analyze_session
    res = analyze_session("data/P04_S01_2minWalk")
    print(res.summary())
"""
from __future__ import annotations

from .pipeline import analyze_session
from .results import KinematicResults

__all__ = ["analyze_session", "KinematicResults"]
