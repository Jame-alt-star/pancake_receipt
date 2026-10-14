"""Gemeinsame Helfer: synchroner Regelkreis (echter Controller + echtes Modell, ohne ROS2)."""
import importlib.util
import os
import sys
from dataclasses import dataclass, field

import mujoco
import numpy as np
import yaml

WS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src") + "/"
BRINGUP = WS + "roboter_simulation/"
STEUER = WS + "roboter_steuerung/"


def load_config(name="g1_policy_29dof.yaml"):
    cfg = yaml.safe_load(open(BRINGUP + "config/" + name))
    cfg["policy_path"] = BRINGUP + cfg["policy_path"]
    return cfg


def load_controller29(cfg):
    if STEUER not in sys.path:
        sys.path.insert(0, STEUER)
    path = STEUER + "roboter_steuerung/control/rl_locomotion_controller_29dof.py"
    spec = importlib.util.spec_from_file_location("ctl29", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.RLLocomotionController29Dof(cfg)


def yaw_deg(q):
    return float(np.degrees(np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))))


@dataclass
class Result:
    fell_at: float = None
    t_end: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    leg_dq: float = 0.0          # mittlere |dq| der Beingelenke in den letzten 2 s
    trace: list = field(default_factory=list)   # (t, x, y, z, yaw)


def run_loop(cmd_fn, dur, delay_ms=0.0, decimation=None, jitter=0, seed=0, noise=None):
    """cmd_fn(t, yaw_deg) -> [vx, vy, wz]. Regelt mit Policy-Takt 50 Hz (Sim-Zeit).
    noise: dict mit Standardabweichungen fuer die Beobachtungen, z.B.
      {"q": 0.001, "dq": 0.02, "gyro": 0.01, "gyro_bias": 0.005, "ori": 0.005}  (rad, rad/s)"""
    cfg = load_config()
    c = load_controller29(cfg)
    m = mujoco.MjModel.from_xml_path(BRINGUP + cfg["xml_path"])
    d = mujoco.MjData(m)
    m.opt.timestep = float(cfg["simulation_dt"])
    kps = np.array(cfg["kps"], dtype=np.float32)
    kds = np.array(cfg["kds"], dtype=np.float32)
    tgt = np.array(cfg["default_angles"], dtype=np.float32)
    d.qpos[7:] = tgt
    mujoco.mj_forward(m, d)
    dec = decimation or int(cfg["control_decimation"])
    delay_steps = int(round(delay_ms / 1000.0 / m.opt.timestep))
    snaps = []
    rng = np.random.default_rng(seed)
    nz = noise or {}
    gyro_bias = rng.normal(0.0, nz.get("gyro_bias", 0.0), 3)
    next_ctrl = dec
    res = Result()
    legs = []
    for i in range(int(dur / m.opt.timestep)):
        d.ctrl[:] = (tgt - d.qpos[7:]) * kps - d.qvel[6:] * kds
        mujoco.mj_step(m, d)
        snaps.append((d.qpos[7:].copy(), d.qvel[6:].copy(), d.qpos[3:7].copy(), d.qvel[3:6].copy()))
        if len(snaps) > delay_steps + 1:
            snaps.pop(0)
        if i + 1 >= next_ctrl:
            q, dq, quat, om = snaps[0]
            if nz:
                q = q + rng.normal(0.0, nz.get("q", 0.0), q.shape)
                dq = dq + rng.normal(0.0, nz.get("dq", 0.0), dq.shape)
                om = om + gyro_bias + rng.normal(0.0, nz.get("gyro", 0.0), 3)
                if nz.get("ori", 0.0) > 0:          # kleine Drehung um zufaellige Achse
                    v = rng.normal(0.0, nz["ori"], 3)
                    dqt = np.concatenate([[1.0], 0.5 * v]); dqt /= np.linalg.norm(dqt)
                    w1, x1, y1, z1 = quat; w2, x2, y2, z2 = dqt
                    quat = np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2,
                                     w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2])
            cmd = np.asarray(cmd_fn(d.time, yaw_deg(quat)), dtype=np.float32)
            tgt = c.compute_joint_targets(q, dq, quat, om, cmd)
            next_ctrl = i + 1 + dec + (int(rng.integers(-jitter, jitter + 1)) if jitter else 0)
        if (i + 1) % 250 == 0:
            res.trace.append((d.time, d.qpos[0], d.qpos[1], d.qpos[2], yaw_deg(d.qpos[3:7])))
        if d.time > dur - 2.0:
            legs.append(float(np.abs(d.qvel[6:18]).mean()))
        if d.qpos[2] < 0.4:
            res.fell_at = d.time
            break
    res.t_end, res.x, res.y, res.z = d.time, d.qpos[0], d.qpos[1], d.qpos[2]
    res.yaw = yaw_deg(d.qpos[3:7])
    res.leg_dq = float(np.mean(legs)) if legs else 0.0
    return res
