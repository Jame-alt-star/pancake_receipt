"""Genauigkeit der Beinodometrie im synchronen Kreis (ohne ROS2), mit realistischem Sensorrauschen.

Gemessen 14.10.2026: Geradeaus 14,5 m -> 11 cm, Gehen+Drehen 24,5 m -> 8 cm (jeweils < 1 % der Strecke).
"""
import os
import sys

import mujoco
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "roboter_steuerung"))
from sim_helpers import BRINGUP, load_config, load_controller29  # noqa: E402
from roboter_steuerung.zustandsschaetzung.beinodometrie import Beinodometrie  # noqa: E402


def _run(cmd_fn, dur, seed=0):
    cfg = load_config()
    xml = BRINGUP + cfg["xml_path"]
    c = load_controller29(cfg)
    m = mujoco.MjModel.from_xml_path(xml)
    d = mujoco.MjData(m)
    m.opt.timestep = 0.002
    kps = np.array(cfg["kps"], dtype=np.float32)
    kds = np.array(cfg["kds"], dtype=np.float32)
    tgt = np.array(cfg["default_angles"], dtype=np.float32)
    d.qpos[7:] = tgt
    mujoco.mj_forward(m, d)
    est = Beinodometrie(xml)
    est.reset(d.qpos[:3])
    rng = np.random.default_rng(seed)
    weg, last = 0.0, d.qpos[:2].copy()
    for i in range(int(dur / 0.002)):
        d.ctrl[:] = (tgt - d.qpos[7:]) * kps - d.qvel[6:] * kds
        mujoco.mj_step(m, d)
        if (i + 1) % 5 == 0:                          # 100 Hz Sensordaten mit Rauschen
            q = d.qpos[7:] + rng.normal(0, 0.001, 29)
            est.update(d.time, q, d.qpos[3:7].copy())
            weg += float(np.linalg.norm(d.qpos[:2] - last))
            last = d.qpos[:2].copy()
        if (i + 1) % 10 == 0:
            tgt = c.compute_joint_targets(d.qpos[7:].copy(), d.qvel[6:].copy(), d.qpos[3:7].copy(),
                                          d.qvel[3:6].copy(), np.array(cmd_fn(d.time), dtype=np.float32))
    return float(np.linalg.norm(est.pos[:2] - d.qpos[:2])), weg, abs(est.pos[2] - d.qpos[2])


@pytest.mark.parametrize("name,fn,dur", [
    ("geradeaus", lambda t: [min(0.5, t), 0, 0], 20),
    ("gehen_drehen", lambda t: [0.5, 0, 0] if (t % 12) < 8 else [0, 0, 1.0], 40),
])
def test_beinodometrie_unter_2_prozent(name, fn, dur):
    err, weg, ez = _run(fn, dur)
    assert weg > 5.0
    assert err / weg < 0.02, f"{name}: Fehler {err:.2f} m auf {weg:.1f} m"
    assert ez < 0.03
