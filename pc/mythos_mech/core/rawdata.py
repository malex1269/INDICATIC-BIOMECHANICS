"""
core/rawdata.py — Per-node 9-DOF CSV loader + raw-data contract.

This is the I/O layer, kept deliberately close to the original gait-kinematics loader because
it was already sound: a **per-node CSV format on a shared (hub) timebase**. Nothing here does
any orientation math — it only reads raw inertial/magnetic samples into per-node arrays.

RAW-DATA CONTRACT
-----------------
Each sample of a node carries:

    t           seconds, monotonic, 0 at bout start, SHARED across nodes (the hub/common
                timebase)                                              (float)
    ax, ay, az  linear acceleration, sensor frame                     (m/s^2)
    gx, gy, gz  angular velocity, sensor frame                        (rad/s)
    mx, my, mz  magnetometer, sensor frame                            (any consistent units;
                                                                       VQF normalises internally)

ON-DISK LAYOUTS (both accepted by :func:`load_session`)
-------------------------------------------------------
  1. **Per-node files** sitting directly in the session dir, named after the node id:
     ``RF.csv``, ``RS.csv``, ``RT.csv``, ``SA.csv`` (the Geneva slice / real-hardware
     format). Columns ``t_native_s,t_opt_s,ax..mz``; ``t_opt_s`` is the shared hub clock.
  2. **Combined file** ``raw/data.csv`` in long format with a ``node`` column
     (``node,t_s,ax..mz``).

THE COMMON-TIMEBASE (HUB) CONTRACT
----------------------------------
All nodes are expected on ONE shared clock (``t_opt_s`` for the slices), 0 at bout start, so
sample *i* of every node is the same instant. The loader does NOT resample across nodes — it
trusts the hub clock and only trims to the common length. (Per-channel rate handling for a
magnetometer on its OWN clock is a VQF concern, see :mod:`core.fusion_vqf`; it is NOT done by
custom resampling here.)
"""
from __future__ import annotations

import csv
import os

import numpy as np

# Time-column aliases, in priority order. ``t_opt_s`` is the shared hub clock of the slices;
# ``t_native_s`` is each sensor's own clock and is intentionally NOT preferred.
_T_ALIASES = ("t_s", "t_opt_s", "t", "time", "time_s", "timestamp_s")
_COL_ALIASES = {
    "ax": ("ax", "acc_x", "a_x"), "ay": ("ay", "acc_y", "a_y"), "az": ("az", "acc_z", "a_z"),
    "gx": ("gx", "gyr_x", "g_x"), "gy": ("gy", "gyr_y", "g_y"), "gz": ("gz", "gyr_z", "g_z"),
    "mx": ("mx", "mag_x", "m_x"), "my": ("my", "mag_y", "m_y"), "mz": ("mz", "mag_z", "m_z"),
}


# --------------------------------------------------------------------------- #
def _resolve_columns(header):
    low = [h.strip().lower() for h in header]
    t_idx = next((low.index(a) for a in _T_ALIASES if a in low), None)
    node_idx = low.index("node") if "node" in low else None
    idx = {c: next((low.index(a) for a in al if a in low), None)
           for c, al in _COL_ALIASES.items()}
    return idx, t_idx, node_idx


def parse_imu_csv(path, node_hint=None) -> dict:
    """Parse one CSV into ``{node: {"t","acc","gyr","mag","mag_present"}}``.

    A combined file (with a ``node`` column) splits into nodes; otherwise the node id comes
    from ``node_hint`` or the filename stem. Missing magnetometer columns become zeros and
    ``mag_present`` is False. Rows are sorted by time.
    """
    with open(path, newline="") as f:
        rdr = csv.reader(f)
        header = next(rdr)
        idx, t_idx, node_idx = _resolve_columns(header)
        if t_idx is None:
            raise ValueError(f"{os.path.basename(path)}: no time column "
                             f"(expected one of {_T_ALIASES})")
        rows = {}
        for r in rdr:
            if not r:
                continue
            node = (r[node_idx].strip() if node_idx is not None
                    else (node_hint or os.path.splitext(os.path.basename(path))[0]))
            rows.setdefault(node, []).append(r)

    out = {}
    for node, rr in rows.items():
        arr = np.array(rr, dtype=object)
        get = lambda j: arr[:, j].astype(float) if j is not None else None
        t = get(t_idx)
        acc = np.column_stack([get(idx["ax"]) if get(idx["ax"]) is not None else np.zeros(len(arr)),
                               get(idx["ay"]) if get(idx["ay"]) is not None else np.zeros(len(arr)),
                               get(idx["az"]) if get(idx["az"]) is not None else np.zeros(len(arr))])
        gyr = np.column_stack([get(idx["gx"]) if get(idx["gx"]) is not None else np.zeros(len(arr)),
                               get(idx["gy"]) if get(idx["gy"]) is not None else np.zeros(len(arr)),
                               get(idx["gz"]) if get(idx["gz"]) is not None else np.zeros(len(arr))])
        has_mag = all(idx[c] is not None for c in ("mx", "my", "mz"))
        if has_mag:
            mag = np.column_stack([get(idx["mx"]), get(idx["my"]), get(idx["mz"])])
        else:
            mag = np.zeros((len(arr), 3))
        order = np.argsort(t)
        mag_present = bool(has_mag and np.any(mag[order] != 0.0))
        out[node] = {"t": t[order], "acc": acc[order], "gyr": gyr[order],
                     "mag": mag[order], "mag_present": mag_present}
    return out


# --------------------------------------------------------------------------- #
def load_session(session_dir) -> tuple[dict, dict]:
    """Load a session's raw IMU into ``{node: stream}`` plus inferred metadata.

    Accepts both on-disk layouts (per-node files, or a combined ``raw/data.csv``). Returns
    ``(streams, meta)`` where ``meta`` has ``session_id`` (folder name), ``nodes`` and the
    inferred measured ``side`` ("right"/"left", from the node ids).
    """
    streams = _load_streams(session_dir)
    if not streams:
        raise FileNotFoundError(
            f"no IMU data in {session_dir!r}: expected per-node CSVs named after their node "
            f"ids (RF.csv, RS.csv, RT.csv, SA.csv) or a combined raw/data.csv")
    nodes = list(streams.keys())
    meta = {
        "session_id": os.path.basename(os.path.normpath(session_dir)),
        "nodes": nodes,
        "side": _infer_side(nodes),
        "path": session_dir,
    }
    return streams, meta


def _load_streams(session_dir) -> dict:
    """Combined raw/data.csv if present, else every per-node CSV in the directory."""
    combined = os.path.join(session_dir, "raw", "data.csv")
    if os.path.exists(combined):
        return parse_imu_csv(combined)
    if not os.path.isdir(session_dir):
        return {}
    streams = {}
    for fn in sorted(os.listdir(session_dir)):
        if not fn.lower().endswith(".csv"):
            continue
        p = os.path.join(session_dir, fn)
        if not os.path.isfile(p):
            continue
        node = os.path.splitext(fn)[0]
        try:
            streams.update(parse_imu_csv(p, node_hint=node))
        except Exception:
            continue  # not an IMU CSV (e.g. a stray export) — skip it
    return streams


def _infer_side(nodes) -> str:
    """Measured leg from the node ids: any 'L*' leg sensor -> left, else right."""
    for n in nodes:
        u = str(n).upper()
        if u in ("LF", "LS", "LT"):
            return "left"
        if u in ("RF", "RS", "RT"):
            return "right"
    return "right"


def infer_rate(t) -> float:
    """Sample rate (Hz) from a timestamp vector (median dt). 0 if undeterminable."""
    t = np.asarray(t, float)
    dt = np.median(np.diff(t)) if len(t) > 1 else 0.0
    return float(1.0 / dt) if dt > 0 else 0.0


def common_timebase(streams) -> tuple[np.ndarray, int]:
    """Shared time grid for a set of node streams under the hub contract.

    Returns ``(t, n)`` where ``n`` is the common (shortest) length and ``t`` the reference
    node's first ``n`` timestamps. Nodes are trusted to share one clock; this only trims to a
    common length so every exported column is the same height.
    """
    nodes = list(streams)
    n = min(len(streams[k]["t"]) for k in nodes)
    # Prefer a foot node as the reference clock, else the first node.
    ref = next((k for k in nodes if str(k).upper() in ("RF", "LF")), nodes[0])
    return np.asarray(streams[ref]["t"][:n], float), n
