"""
kinematics/results.py — the object returned by :func:`kinematics.pipeline.analyze_session`.

``KinematicResults`` bundles everything the kinematic pipeline produced — the common time grid,
per-joint flexion/velocity traces and parameters, the temporal & (estimated) spatial gait
parameters, the detected gait events, the steady-state mask and turnarounds — plus small
exporters that write the intermediate CSV/JSON artefacts the viewer (or any other consumer)
loads. No optical/marker ground truth is ever required.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json

import numpy as np


@dataclass
class KinematicResults:
    session_id: str
    fs: float                                   # common sample rate (Hz)
    t: np.ndarray                               # (N,) time grid (s), 0 at bout start
    side: str                                   # measured leg ("right"/"left")
    joints: dict                                # name -> {flexion, flexion_gravity_only, ang_vel, params}
    events: dict                                # foot_strike / mid_swing / toe_off (sample idx) + sagittal_rate
    temporal: dict                              # cadence, stride/step time, stance/swing, CV ...
    spatial: dict                               # stride length & speed (estimate)
    steady_state: np.ndarray                    # (N,) bool mask (turns excluded)
    turnarounds: list = field(default_factory=list)     # (start_idx, end_idx, deg)
    modes: dict = field(default_factory=dict)           # node -> "6D"/"9D" effective fusion mode
    orientations: dict = field(default_factory=dict)    # node -> (N,4) VQF quaternion
    foot_node: str | None = None
    warnings: list = field(default_factory=list)

    # -- queries ------------------------------------------------------------- #
    @property
    def joint_names(self) -> list:
        return list(self.joints.keys())

    @property
    def duration_s(self) -> float:
        return float(self.t[-1]) if len(self.t) else 0.0

    def angle(self, joint: str) -> np.ndarray:
        return self.joints[joint]["flexion"]

    # -- summary ------------------------------------------------------------- #
    def summary(self) -> dict:
        """JSON-serialisable dict of all parameters (no time-series arrays)."""
        return {
            "session_id": self.session_id,
            "fs_hz": self.fs,
            "duration_s": self.duration_s,
            "side": self.side,
            "fusion_modes": self.modes,
            "joints": {j: d["params"] for j, d in self.joints.items()},
            "temporal": self.temporal,
            "spatial": self.spatial,
            "n_turnarounds": len(self.turnarounds),
            "turnarounds": [{"t_start_s": float(self.t[s]), "t_end_s": float(self.t[e]),
                             "deg": float(a)} for s, e, a in self.turnarounds],
            "warnings": self.warnings,
        }

    def save_summary_json(self, path: str) -> str:
        with open(path, "w") as f:
            json.dump(self.summary(), f, indent=2)
        return path

    # -- time-series export -------------------------------------------------- #
    def timeseries_columns(self) -> dict:
        """Ordered column dict (name -> array) of the full joint-angle time-series output."""
        cols = {"t_s": self.t}
        for j, d in self.joints.items():
            cols[f"{j}_deg"] = d["flexion"]
            cols[f"{j}_vel_dps"] = d["ang_vel"]
        n = len(self.t)
        for ev_name in ("foot_strike", "mid_swing", "toe_off"):
            idx = np.asarray(self.events.get(ev_name, []), int)
            idx = idx[idx < n]
            col = np.zeros(n, int)
            col[idx] = 1
            cols[ev_name] = col
        cols["steady_state"] = self.steady_state.astype(int)
        return cols

    def save_timeseries_csv(self, path: str) -> str:
        cols = self.timeseries_columns()
        header = ",".join(cols.keys())
        arr = np.column_stack([np.asarray(v, float) for v in cols.values()])
        np.savetxt(path, arr, delimiter=",", header=header, comments="", fmt="%.6g")
        return path

    def save_events_csv(self, path: str) -> str:
        """One row per gait event: time (s), sample index, event type."""
        rows = []
        for name in ("foot_strike", "toe_off", "mid_swing"):
            for i in np.asarray(self.events.get(name, []), int):
                if i < len(self.t):
                    rows.append((float(self.t[i]), int(i), name))
        rows.sort()
        with open(path, "w", newline="\n") as f:
            f.write("t_s,sample,event\n")
            for ts, i, name in rows:
                f.write(f"{ts:.6f},{i},{name}\n")
        return path

    def __repr__(self) -> str:
        cad = self.temporal.get("cadence_steps_per_min", float("nan"))
        return (f"KinematicResults({self.session_id}, fs={self.fs:.1f}Hz, "
                f"dur={self.duration_s:.1f}s, joints={self.joint_names}, "
                f"cadence={cad:.1f}/min)")
