"""
core/config.py — Mounting configuration (sensor-count agnostic).

The mounting config is the single place that knows *which* sensors are present and *what*
body segment each one is on. Everything downstream (fusion, OpenSim column naming) is driven
by it, so the pipeline is agnostic to the number of sensors — you change the rig by changing
the config, not the code.

It declares:

  * ``sensors`` : ``node_id -> segment kind``. The kind is one of
                  :data:`SEGMENT_KINDS` = ``("pelvis", "thigh", "shank", "foot")`` — the
                  generic anatomical part, NOT a side-specific label. The measured side is a
                  separate field so the same four kinds describe either leg.
  * ``side``    : the measured leg, ``"right"`` or ``"left"``. Only ONE leg is instrumented
                  in the default rig; the OpenSim column names get the matching ``r``/``l``
                  suffix and the other leg is never fabricated.
  * ``rates``   : declared per-channel sample rates (Hz). ``imu_hz`` covers accel+gyro;
                  ``mag_hz`` may differ. These are *nominal* — fusion reads the true rate
                  from the timestamps when it can — but they are what feeds VQF in the
                  genuine multi-rate case (magnetometer on its own clock).
  * ``mode``    : orientation-fusion mode handed to VQF — ``"6D"`` (magnetometer-free),
                  ``"9D"`` (gyr+acc+mag), or ``"auto"`` (9D when a usable magnetometer is
                  present, else 6D). See :mod:`core.fusion_vqf` for the trade-off.

DEFAULT rig: **4 sensors, ONE leg, WITH pelvis** -> ``{pelvis, thigh, shank, foot}``, using
the Geneva node ids ``SA/RT/RS/RF`` (right) or ``SA/LT/LS/LF`` (left).
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict

# The four anatomical segment kinds this pipeline understands. Order is proximal -> distal;
# OpenSim columns are emitted in this order.
SEGMENT_KINDS = ("pelvis", "thigh", "shank", "foot")

# Canonical Geneva-style node id -> segment kind. Drives the default rig and the
# auto-detection in :func:`config_for_nodes`. Side is read from the R/L prefix.
NODE_KIND = {
    "SA": "pelvis",
    "RT": "thigh", "RS": "shank", "RF": "foot",
    "LT": "thigh", "LS": "shank", "LF": "foot",
}

# Nominal default sample rates (Hz). Fusion prefers the rate measured from the timestamps;
# these are the fallback and the rate VQF uses for a magnetometer on its own clock.
DEFAULT_RATES = {"imu_hz": 100.0, "mag_hz": 100.0}

_VALID_MODES = ("6D", "9D", "auto")


@dataclass
class MountingConfig:
    """Which sensors are present, the segment each is on, the measured side, and rates.

    Construct directly, via :func:`default_config` / :func:`config_for_nodes`, or from a
    plain dict / YAML mapping with :meth:`coerce`.
    """
    sensors: dict                                  # node_id -> segment kind (SEGMENT_KINDS)
    side: str = "right"                            # measured leg: "right" | "left"
    rates: dict = field(default_factory=lambda: dict(DEFAULT_RATES))
    mode: str = "6D"                               # "6D" | "9D" | "auto"

    # -- construction -------------------------------------------------------- #
    def __post_init__(self):
        self.sensors = {str(k): str(v).lower() for k, v in self.sensors.items()}
        bad = {n: k for n, k in self.sensors.items() if k not in SEGMENT_KINDS}
        if bad:
            raise ValueError(f"unknown segment kind(s) {bad}; valid kinds are {SEGMENT_KINDS}")
        s = str(self.side).lower()
        self.side = "left" if s.startswith("l") else "right"
        self.rates = {**DEFAULT_RATES, **dict(self.rates or {})}
        self.rates.setdefault("mag_hz", self.rates["imu_hz"])
        if self.mode not in _VALID_MODES:
            raise ValueError(f"mode must be one of {_VALID_MODES}; got {self.mode!r}")

    @classmethod
    def coerce(cls, v) -> "MountingConfig":
        """Accept a MountingConfig, a dict, or None (-> 4-sensor right-leg default)."""
        if v is None:
            return default_config()
        if isinstance(v, cls):
            return v
        v = dict(v)
        return cls(sensors=dict(v["sensors"]), side=v.get("side", "right"),
                   rates=dict(v.get("rates", {})), mode=v.get("mode", "6D"))

    def to_dict(self) -> dict:
        return asdict(self)

    # -- queries ------------------------------------------------------------- #
    @property
    def imu_hz(self) -> float:
        return float(self.rates.get("imu_hz", DEFAULT_RATES["imu_hz"]))

    @property
    def mag_hz(self) -> float:
        return float(self.rates.get("mag_hz", self.imu_hz))

    @property
    def side_char(self) -> str:
        """'r' or 'l' — the suffix used in OpenSim IMU column names."""
        return "l" if self.side == "left" else "r"

    @property
    def nodes(self) -> list:
        return list(self.sensors.keys())


# --------------------------------------------------------------------------- #
def default_config(side: str = "right", *, imu_hz: float | None = None,
                   mag_hz: float | None = None, mode: str = "6D") -> MountingConfig:
    """DEFAULT rig: 4 sensors on ONE leg -> {pelvis, thigh, shank, foot}.

    ``side`` selects the leg (right: SA/RT/RS/RF, left: SA/LT/LS/LF). Rates default to
    :data:`DEFAULT_RATES`; the true rate is still measured from the data at fusion time.
    """
    s = "L" if str(side).lower().startswith("l") else "R"
    sensors = {"SA": "pelvis", f"{s}T": "thigh", f"{s}S": "shank", f"{s}F": "foot"}
    rates = {"imu_hz": imu_hz if imu_hz is not None else DEFAULT_RATES["imu_hz"]}
    rates["mag_hz"] = mag_hz if mag_hz is not None else rates["imu_hz"]
    return MountingConfig(sensors=sensors, side=("left" if s == "L" else "right"),
                          rates=rates, mode=mode)


def config_for_nodes(nodes, *, side: str | None = None, imu_hz: float | None = None,
                     mag_hz: float | None = None, mode: str = "6D") -> MountingConfig:
    """Build a ONE-leg config from the node ids actually present (sensor-count agnostic).

    Each known node id (see :data:`NODE_KIND`) maps to its segment kind. This is a single-leg
    rig, so exactly one leg is kept (plus the pelvis): the measured ``side`` is taken from the
    argument when given, otherwise inferred from the present leg sensors (defaulting to right
    when both legs happen to be present). Only that leg's sensors are retained — the other leg
    is dropped, never mapped — so OpenSim columns are always unique. Adapts to 2, 3 or 4
    sensors while the shipped default remains the 4-sensor one-leg rig.
    """
    kinds = {}
    present_sides = []
    for n in nodes:
        kind = NODE_KIND.get(str(n))
        if kind is None:
            continue
        kinds[str(n)] = kind
        if kind != "pelvis":
            present_sides.append("left" if str(n).upper().startswith("L") else "right")
    if not kinds:
        raise ValueError(f"no recognised sensor node ids in {list(nodes)}; "
                         f"expected some of {sorted(NODE_KIND)}")

    if side is not None:
        chosen = "left" if str(side).lower().startswith("l") else "right"
    elif present_sides:
        chosen = "right" if "right" in present_sides else "left"
    else:
        chosen = "right"
    pref = "L" if chosen == "left" else "R"
    # Keep the pelvis plus only the chosen leg's sensors (single-leg rig).
    sensors = {n: k for n, k in kinds.items()
               if k == "pelvis" or str(n).upper().startswith(pref)}

    rates = {"imu_hz": imu_hz if imu_hz is not None else DEFAULT_RATES["imu_hz"]}
    rates["mag_hz"] = mag_hz if mag_hz is not None else rates["imu_hz"]
    return MountingConfig(sensors=sensors, side=chosen, rates=rates, mode=mode)
