"""Runtime discovery and frame conversions for the local water backend."""
import os
import sys
from pathlib import Path
import numpy as np


def load_newton():
    root = Path(os.environ.get('WATER_ISAAC_ROOT', 'Y:/isaacsim'))
    for path in (root/'exts/isaacsim.pip.newton/pip_prebundle',
                 root/'extscache/omni.warp.core-1.13.0+wx64',
                 root/'exts/omni.pip.compute/pip_prebundle'):
        sys.path.insert(0, str(path))
    import newton
    import warp as wp
    return newton, wp


def rotate(q, v):
    q = np.asarray(q)
    return v + 2*np.cross(q[:3], np.cross(q[:3], v)+q[3]*v)


def boundary_state(q, qd, com):
    """Newton q uses body origin, qd linear velocity uses COM."""
    offset = rotate(q[3:], com)
    return np.r_[q, qd[:3]-np.cross(qd[3:], offset), qd[3:]].astype(np.float32)


def com_wrench(wrench, q, com):
    result = np.array(wrench, dtype=np.float32, copy=True)
    result[3:] -= np.cross(rotate(q[3:], com), result[:3])
    return result
