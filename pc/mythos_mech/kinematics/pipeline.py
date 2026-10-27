"""
kinematics/pipeline.py — the library's main entry point: raw IMU CSVs -> gait kinematics.

``analyze_session(session_dir)`` is the single public function and the spine of the library.
It is **pure**: a session folder in, a :class:`KinematicResults` out. Internally it chains the
validated stages, in order:

    load        per-node 9-DOF CSVs on the shared hub clock        core.rawdata
    fuse        VQF orientation per segment (sensor->earth)         core.fusion_vqf
    segment     pelvis turnarounds -> steady straight-walking mask  kinematics.gait_events
    angles      yaw-immune sagittal flexion per joint               kinematics.joint_angles
    events      foot strike / toe-off / mid-swing, on the foot      kinematics.gait_events
    parameters  per-joint ROM/peaks + temporal + spatial estimate   kinematics.parameters

The kinematic OUTPUT (joint angles + gait parameters, visualised in Python) is the goal. The
OpenSim/OpenSense path (``opensim_export``) is a *separate* downstream option, not part of this.

CLI
---
    python -m kinematics.pipeline <session_dir> [--mode 6D|9D|auto] [--side right|left] [--csv]

``<session_dir>`` holds per-node CSVs (RF.csv/RS.csv/RT.csv/SA.csv) or a combined raw/data.csv.
``--csv`` writes the intermediate artefacts into ``<session>/results/``.
"""
from __future__ import annotations

import argparse
import os

import numpy as np

from core.rawdata import load_session, infer_rate
from core.config import config_for_nodes
from core.fusion_vqf import fuse_session
from . import gait_events as G
from . import joint_angles as A
from . import parameters as P
from .results import KinematicResults

# Joint topology by anatomical kind: name -> (distal_kind, proximal_kind), proximal->distal.
_JOINTS_BY_KIND = {
    "hip": ("thigh", "pelvis"),
    "knee": ("shank", "thigh"),
    "ankle": ("foot", "shank"),
}


def _resolve_joints(sensors: dict) -> tuple[dict, list]:
    """From ``{node: kind}`` build computable ``{joint: (distal_node, proximal_node)}``.

    A joint is computable when both its segment kinds are present. Returns the joint map plus a
    list of human-readable warnings for the joints skipped for a missing segment.
    """
    kind_to_node = {kind: node for node, kind in sensors.items()}
    joints, warnings = {}, []
    for jname, (dk, pk) in _JOINTS_BY_KIND.items():
        if dk in kind_to_node and pk in kind_to_node:
            joints[jname] = (kind_to_node[dk], kind_to_node[pk])
        else:
            missing = [k for k in (dk, pk) if k not in kind_to_node]
            warnings.append(f"joint {jname!r} skipped: missing segment(s) {missing}")
    return joints, warnings


# --------------------------------------------------------------------------- #
def analyze_session(session_dir, *, mode: str = "6D", side: str | None = None,
                    tau: float = 0.0) -> KinematicResults:
    """Raw IMU session folder -> :class:`KinematicResults` (joint angles + gait parameters).

    ``mode`` is the VQF fusion mode ("6D" default, magnetometer-free); ``side`` forces the
    measured leg (else inferred from node ids); ``tau`` (s) optionally enables the joint-angle
    complementary refinement (0 = drift-free gravity-projection angle, the default).
    """
    streams, meta = load_session(session_dir)
    config = config_for_nodes(meta["nodes"], side=side, mode=mode)
    fused = fuse_session(streams, config)

    t = np.asarray(fused["t"], float)
    fs = float(fused["fs"]) or infer_rate(t)
    n = len(t)
    quats = {node: np.asarray(q, float)[:n] for node, q in fused["orientations"].items()}
    gyr = {node: np.asarray(streams[node]["gyr"], float)[:n] for node in quats}
    acc = {node: np.asarray(streams[node]["acc"], float)[:n] for node in quats}

    joints, warnings = _resolve_joints(config.sensors)
    kind_to_node = {kind: node for node, kind in config.sensors.items()}
    foot_node = kind_to_node.get("foot")
    shank_node = kind_to_node.get("shank")
    pelvis_node = kind_to_node.get("pelvis")
    # Gait events: prefer the shank (Salarian 2004 — cleanest toe-off), fall back to the foot.
    event_node = shank_node if (shank_node and shank_node in gyr) else foot_node

    # Steady straight-walking mask from pelvis turnarounds (whole bout steady if no pelvis).
    if pelvis_node and pelvis_node in acc:
        turns, _ = G.detect_turnarounds(acc[pelvis_node], gyr[pelvis_node], fs)
    else:
        turns = []
        warnings.append("no pelvis node: turnarounds not detected; whole bout treated as steady")
    mask = G.steady_state_mask(n, turns, fs)

    # Per-segment neutral 'up' (quietest window) for the gravity-projection joint angle.
    grav0 = {node: A.neutral_gravity(acc[node], gyr[node], fs) for node in quats}

    # Joint angles (gait events first, so per-joint params can use cycle-averaged ROM).
    if event_node and event_node in gyr:
        ev = G.detect_events(gyr[event_node], fs)
    else:
        ev = {"foot_strike": np.array([], int), "mid_swing": np.array([], int),
              "toe_off": np.array([], int), "sagittal_rate": np.zeros(n)}
        warnings.append("no shank/foot node: gait events not detected")
    ev["event_node"] = event_node
    strikes = ev.get("foot_strike", np.array([], int))

    joint_out = {}
    for jname, (dist, prox) in joints.items():
        ja = A.joint_flexion(quats[dist], quats[prox], gyr[dist], gyr[prox],
                             grav0[dist], grav0[prox], axis_mask=mask, fs=fs, tau=tau)
        flex = ja["flexion"]
        ang_vel = A.derivative(flex, fs)
        params = P.joint_parameters(flex, ang_vel, mask, fs)
        # Cycle-averaged ROM/peaks (robust to occasional bad strides near turns).
        _grid, _cyc, cyc_mean, _std = P.overlay_cycles(flex, strikes, mask, fs)
        if cyc_mean is not None:
            params["rom_cycle_deg"] = float(np.nanmax(cyc_mean) - np.nanmin(cyc_mean))
            params["peak_flexion_cycle_deg"] = float(np.nanmax(cyc_mean))
            params["peak_extension_cycle_deg"] = float(np.nanmin(cyc_mean))
        joint_out[jname] = {
            "flexion": flex,
            "flexion_gravity_only": ja["flexion_gravity_only"],
            "ang_vel": ang_vel,
            "params": params,
        }

    temporal = P.temporal_parameters(ev, mask, fs)

    spatial = (P.stride_length_zupt(quats[foot_node], acc[foot_node], gyr[foot_node], ev, mask, fs)
               if foot_node and foot_node in quats
               else {"stride_length_m_est": float("nan"), "walking_speed_mps_est": float("nan")})

    return KinematicResults(
        session_id=meta["session_id"], fs=fs, t=t, side=config.side,
        joints=joint_out, events=ev, temporal=temporal, spatial=spatial,
        steady_state=mask, turnarounds=turns, modes=fused["modes"],
        orientations=quats, foot_node=foot_node, warnings=warnings,
    )


# --------------------------------------------------------------------------- #
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Compute gait kinematics (joint angles + gait parameters) from a raw IMU "
                    "session. The OpenSim export is separate (opensim_export).")
    ap.add_argument("session", help="session folder (RF.csv/RS.csv/RT.csv/SA.csv or raw/data.csv)")
    ap.add_argument("--mode", choices=("6D", "9D", "auto"), default="6D",
                    help="VQF fusion mode (default: 6D, magnetometer-free)")
    ap.add_argument("--side", choices=("right", "left"), default=None,
                    help="force the measured leg (default: infer from node ids)")
    ap.add_argument("--csv", action="store_true",
                    help="write intermediate CSV/JSON artefacts into <session>/results/")
    args = ap.parse_args(argv)
    if not os.path.isdir(args.session):
        ap.error(f"not a directory: {args.session}")

    res = analyze_session(args.session, mode=args.mode, side=args.side)
    print(res)
    print(f"  side={res.side}  duration={res.duration_s:.1f}s  turns={len(res.turnarounds)}")
    for j in res.joint_names:
        p = res.joints[j]["params"]
        rom_c = p.get("rom_cycle_deg", p["rom_deg"])
        print(f"  {j:5s}  ROM(cycle)={rom_c:5.1f} deg  peak_flex(cycle)="
              f"{p.get('peak_flexion_cycle_deg', p['peak_flexion_deg']):6.1f} deg  "
              f"cycles={p['cycle_count']}")
    tp = res.temporal
    print(f"  cadence={tp['cadence_steps_per_min']:.1f}/min  "
          f"stride={tp['stride_time_mean_s']:.3f}+/-{tp['stride_time_std_s']:.3f}s  "
          f"stance={tp['stance_pct']:.1f}%  swing={tp['swing_pct']:.1f}%  "
          f"CV={tp['stride_time_cv_pct']:.1f}%")
    sp = res.spatial
    print(f"  stride_length~{sp['stride_length_m_est']:.2f}m  "
          f"speed~{sp['walking_speed_mps_est']:.2f}m/s  (estimate)")

    if args.csv:
        out = os.path.join(args.session, "results")
        os.makedirs(out, exist_ok=True)
        sid = res.session_id
        res.save_timeseries_csv(os.path.join(out, f"{sid}_joint_angles.csv"))
        res.save_events_csv(os.path.join(out, f"{sid}_gait_events.csv"))
        res.save_summary_json(os.path.join(out, f"{sid}_gait_parameters.json"))
        print(f"  wrote {sid}_joint_angles.csv, {sid}_gait_events.csv, {sid}_gait_parameters.json "
              f"-> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
