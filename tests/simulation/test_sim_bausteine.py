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
    lid = Lidar(m, 0.45, 360, 10.0, ringe=24)
    ranges = lid.scan(d, d.qpos[3:7])
    assert abs(ranges[180] - 1.25) < 0.02                 # Strahl 180 = Winkel 0 (nach vorn)
    assert np.isfinite(ranges).all()                      # geschlossener Raum: jeder Strahl trifft eine Wand
    pts = lid.wolke(d, d.qpos[3:7])
    vorn = pts[(np.abs(pts[:, 1]) < 0.05) & (pts[:, 0] > 0)]
    assert len(vorn) and abs(vorn[:, 0].min() - 1.25) < 0.05
    assert not np.any(np.linalg.norm(pts, axis=1) <= 0.15)     # Blindbereich, kein eigener Koerper


def test_lidar_ausfaelle_und_rauschen():
    m, d = _testwelt()
    lid = Lidar(m, 0.45, 360, 10.0)
    ranges = lid.scan(d, d.qpos[3:7], np.random.default_rng(1), 0.02, 0.5)
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
