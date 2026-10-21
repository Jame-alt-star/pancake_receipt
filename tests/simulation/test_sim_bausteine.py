"""Bausteine der Simulation ohne ROS: Lidar (Raycast), Sensormodell, Posenaufnahme."""
import math
import sys

import mujoco
import numpy as np

from sim_helpers import BRINGUP, WS, load_config

sys.path.insert(0, BRINGUP)
from roboter_simulation.aufnahme import Posenaufnahme  # noqa: E402
from roboter_simulation.lidar import Lidar  # noqa: E402
from roboter_simulation.sensormodell import Sensormodell  # noqa: E402
from roboter_simulation.szene import lade_modell  # noqa: E402


MONTAGE = {"parent": "pelvis", "xyz": [0.0, 0.0, 0.45], "rpy": [0.0, 0.0, 0.0]}


def _testwelt():
    cfg = load_config()
    m = lade_modell(cfg["xml_path"], WS + "roboter_welten/welten/testwelt/szene.xml")
    d = mujoco.MjData(m)
    d.qpos[7:] = cfg["default_angles"]
    d.qpos[2] = 0.793
    mujoco.mj_forward(m, d)
    return m, d


def test_lidar_trifft_hindernis_vor_dem_roboter():
    """Testwelt: Hindernis-Vorderkante bei x = 1.25 (Quader 0.5 m tief um x = 1.5), Roboter bei (0, 0) mit Blick +x."""
    m, d = _testwelt()
    lid = Lidar(m, MONTAGE, 360, 10.0, ringe=24)
    ranges = lid.scan(d)
    assert abs(ranges[180] - 1.25) < 0.02                 # Strahl 180 = Winkel 0 (nach vorn)
    assert np.isfinite(ranges).all()                      # geschlossener Raum: jeder Strahl trifft eine Wand
    pts = lid.wolke(d)
    vorn = pts[(np.abs(pts[:, 1]) < 0.05) & (pts[:, 0] > 0)]
    assert len(vorn) and abs(vorn[:, 0].min() - 1.25) < 0.05
    assert not np.any(np.linalg.norm(pts, axis=1) <= 0.15)     # Blindbereich, kein eigener Koerper


def test_lidar_ausfaelle_und_rauschen():
    m, d = _testwelt()
    lid = Lidar(m, MONTAGE, 360, 10.0)
    ranges = lid.scan(d, np.random.default_rng(1), 0.02, 0.5)
    anteil_inf = float(np.mean(np.isinf(ranges)))
    assert 0.35 < anteil_inf < 0.65


def test_sensormodell_latenz_und_drift():
    rng = np.random.default_rng(0)
    sm = Sensormodell(rng, 0.0, 0.0, 0.0, 0.0, 0.0, latenz_s=0.01, yaw_drift_rad_s=0.0)
    q0 = np.array([1.0, 0, 0, 0])
    for i in range(20):                                   # 2-ms-Schritte, Gelenkwinkel = Zeit
        t = i * 0.002
        q, _, _, _ = sm.anwenden(t, np.array([t]), np.zeros(1), q0, np.zeros(3))
    assert 0.026 - 1e-9 <= q[0] <= 0.028 + 1e-9           # Wert von vor ~10 ms (Raster 2 ms)
    drift = Sensormodell(np.random.default_rng(0), 0, 0, 0, 0, 0, 0.0, yaw_drift_rad_s=math.radians(0.5) / 60)
    _, _, quat, _ = drift.anwenden(600.0, np.zeros(1), np.zeros(1), q0, np.zeros(3))
    yaw = math.degrees(2 * math.atan2(quat[3], quat[0]))
    assert abs(yaw - 5.0) < 1e-6                          # 0.5 Grad/min x 10 min


def test_posenaufnahme(tmp_path):
    pfad = str(tmp_path / "pose.npz")
    a = Posenaufnahme(pfad)
    for i in range(500):
        a.schritt(i * 0.002, np.full(3, i))               # 1 s Simzeit im Physiktakt -> ~25 Bilder (25 Hz)
    a.schreiben()
    z = np.load(pfad)
    assert 24 <= len(z["t"]) <= 25 and z["qpos"].shape == (len(z["t"]), 3)
    assert np.all(np.diff(z["t"]) >= 0.04 - 1e-9)
    Posenaufnahme("").schritt(0.0, np.zeros(3))           # ohne Pfad: nichts tun


def test_lidar_montage_folgt_dem_elternglied():
    """Am torso_link montiert dreht der Sensor mit der Taille mit (am Becken nicht)."""
    m, d = _testwelt()
    torso = Lidar(m, {"parent": "torso_link", "xyz": [0.0, 0.0, 0.4], "rpy": [0.0, 0.0, 0.0]}, 36, 10.0)
    becken = Lidar(m, MONTAGE, 36, 10.0)
    _, R_t0 = torso.pose(d); _, R_b0 = becken.pose(d)
    gier = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "waist_yaw_joint")]
    d.qpos[gier] += 0.5                                                   # Taille 0.5 rad drehen
    mujoco.mj_kinematics(m, d)
    _, R_t1 = torso.pose(d); _, R_b1 = becken.pose(d)
    winkel = lambda A, B: np.degrees(np.arccos(np.clip((np.trace(A.T @ B) - 1) / 2, -1, 1)))   # noqa: E731
    assert abs(winkel(R_t0, R_t1) - np.degrees(0.5)) < 0.5 and winkel(R_b0, R_b1) < 1e-6


def test_rpy_konvention_wie_static_transform_publisher():
    """R = Rz(yaw) Ry(pitch) Rx(roll), wie tf2 (static_transform_publisher --roll --pitch --yaw)."""
    import math
    from roboter_beschreibung.geometrie import quat_mul, quat_zu_rotmat, rotmat_aus_rpy
    r, p, y = 0.3, -0.2, 1.1
    qx = (math.cos(r / 2), math.sin(r / 2), 0, 0)
    qy = (math.cos(p / 2), 0, math.sin(p / 2), 0)
    qz = (math.cos(y / 2), 0, 0, math.sin(y / 2))
    assert np.allclose(rotmat_aus_rpy(r, p, y), quat_zu_rotmat(quat_mul(qz, quat_mul(qy, qx))))
