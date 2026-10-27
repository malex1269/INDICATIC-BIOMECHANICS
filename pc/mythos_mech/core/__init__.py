"""
core — raw-IMU I/O, mounting config, and VQF orientation fusion for the OpenSim pipeline.

Pipeline: :func:`core.rawdata.load_session` -> :func:`core.fusion_vqf.fuse_session`, driven by
a :class:`core.config.MountingConfig`. No orientation math beyond the validated VQF filter.
"""
from .config import (MountingConfig, SEGMENT_KINDS, NODE_KIND,
                     default_config, config_for_nodes)
from .rawdata import load_session, parse_imu_csv, infer_rate, common_timebase
from .fusion_vqf import fuse_session, fuse_segment

__all__ = [
    "MountingConfig", "SEGMENT_KINDS", "NODE_KIND", "default_config", "config_for_nodes",
    "load_session", "parse_imu_csv", "infer_rate", "common_timebase",
    "fuse_session", "fuse_segment",
]
