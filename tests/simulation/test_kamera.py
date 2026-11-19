"""RGBD-Kamera der Simulation (kamera.py, szene.py): Achsen, Intrinsik, Tiefe, Bildausrichtung.

Ohne OpenGL laufen nur die Rechentests; die Rendertests ueberspringen sich selbst (z. B. in der CI ohne Bildschirm).
Kern der Rendertests: Pixel mit Intrinsik und gemessener Tiefe in den optical frame zurueckrechnen und in die Welt
uebertragen - die Punkte muessen auf den bekannten Flaechen der Szene liegen. Das prueft Intrinsik, Tiefe (entlang der
Blickachse), Bildausrichtung (nicht gespiegelt, nicht kopfueber) und die Achsen des optical frame zusammen.
"""
import ast
import math
import os
import threading

import mujoco
import numpy as np
import pytest

from roboter_beschreibung.geometrie import rotmat_aus_rpy
from roboter_simulation.kamera import (KAMERA_NAME, R_LINK_ZU_MUJOCO_KAMERA, Kamera, KameraThread, intrinsik,
                                       kamera_quat, tiefe_aufbereiten)
from roboter_simulation.szene import lade_modell
from sim_helpers import BESCHREIBUNG, load_config

# Wie roboter.yaml, aber waagerecht (Nickwinkel 0), damit die erwarteten Werte einfach bleiben.
MONTAGE = {"parent": "torso_link", "xyz": [0.09, 0.0, 0.42], "rpy": [0.0, 0.0, 0.0],
           "fovy_grad": 58.0, "breite": 320, "hoehe": 240}

# Testszene: Wand bei x = 2.0 (Vorderseite 1.95) quer vor dem Roboter, roter Quader LINKS davor (y = +0.5,
# Vorderseite x = 1.4). Links/rechts unterscheidbar, damit ein gespiegeltes Bild auffaellt.
SZENE = """<mujoco><worldbody>
  <geom name="wand" type="box" pos="2.0 0 1.0" size="0.05 3 1.0" group="2" rgba="0.7 0.7 0.7 1"/>
  <geom name="rot"  type="box" pos="1.5 0.5 1.2" size="0.1 0.1 0.2" group="2" rgba="1 0 0 1"/>
</worldbody></mujoco>"""


def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


# ---------------------------------------------------------------- ohne OpenGL
def test_kamera_blickt_entlang_x_des_kamera_links():
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, kamera_quat([0.0, 0.0, 0.0]))
    R = R.reshape(3, 3)
    assert np.allclose(R @ [0, 0, -1], [1, 0, 0])     # Blickrichtung (-z der MuJoCo-Kamera) = vorn
    assert np.allclose(R @ [0, 1, 0], [0, 0, 1])      # Bild oben = oben
    assert np.allclose(R @ [1, 0, 0], [0, -1, 0])     # Bild rechts = rechts (-y)
    mujoco.mju_quat2Mat(R.reshape(-1), kamera_quat([0.0, 0.3, 0.0]))
    blick = R.reshape(3, 3) @ [0, 0, -1]
    assert blick[2] < 0 and abs(math.atan2(-blick[2], blick[0]) - 0.3) < 1e-9   # positiver Nickwinkel = nach unten


def test_optical_frame_im_tf_baum_passt_zur_mujoco_kamera():
    """beschreibung.launch.py (OPTICAL_RPY) und kamera.py muessen dieselben Achsen meinen: der optical frame ist die
    MuJoCo-Kamera, um 180 Grad um x gedreht (y und z umgekehrt)."""
    with open(os.path.join(BESCHREIBUNG, "launch", "beschreibung.launch.py")) as f:
        baum = ast.parse(f.read())
    ausdruck = next(k.value for k in ast.walk(baum) if isinstance(k, ast.Assign)
                    and any(getattr(z, "id", None) == "OPTICAL_RPY" for z in k.targets))
    rpy = eval(compile(ast.Expression(ausdruck), "OPTICAL_RPY", "eval"), {"math": math})
    assert np.allclose(rotmat_aus_rpy(*rpy), R_LINK_ZU_MUJOCO_KAMERA @ _rot_x(math.pi))


def test_intrinsik():
    fx, fy, cx, cy = intrinsik(320, 240, 58.0)
    assert fx == fy and abs(fy - 120.0 / math.tan(math.radians(29.0))) < 1e-9
    assert (cx, cy) == (159.5, 119.5)


def test_tiefe_aufbereiten_bereich_und_rauschen():
    t = np.array([[0.1, 0.5, 5.0, 40.0]], dtype=np.float32)
    a = tiefe_aufbereiten(t, 0.2, 10.0)
    assert np.isnan(a[0, 0]) and np.isnan(a[0, 3]) and a[0, 1] == 0.5 and a[0, 2] == 5.0
    rng = np.random.default_rng(0)
    b = tiefe_aufbereiten(np.full((200, 200), 2.0, dtype=np.float32), 0.2, 10.0, rng, 0.01)
    assert abs(np.nanstd(b) - 0.01 * 2.0 ** 2) < 0.002 and abs(np.nanmean(b) - 2.0) < 0.002


def test_modell_bekommt_die_kamera_aus_roboter_yaml():
    m = lade_modell(load_config()["xml_path"], kamera=MONTAGE)
    i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, KAMERA_NAME)
    assert i >= 0
    assert m.cam_bodyid[i] == mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
    assert tuple(m.cam_resolution[i]) == (320, 240) and abs(m.cam_fovy[i] - 58.0) < 1e-9
    assert mujoco.MjModel.from_xml_path(load_config()["xml_path"]).ncam == 0   # Unitree-Datei bleibt unveraendert


# ---------------------------------------------------------------- mit OpenGL
@pytest.fixture(scope="module")
def szene(tmp_path_factory):
    if not (os.environ.get("MUJOCO_GL") or os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        pytest.skip("kein OpenGL (kein Bildschirm, MUJOCO_GL nicht gesetzt)")
    pfad = tmp_path_factory.mktemp("kamera") / "szene.xml"
    pfad.write_text(SZENE)
    cfg = load_config()
    m = lade_modell(cfg["xml_path"], str(pfad), MONTAGE)
    d = mujoco.MjData(m)
    d.qpos[7:] = cfg["default_angles"]
    d.qpos[2] = 0.793
    mujoco.mj_forward(m, d)
    try:
        kamera = Kamera(m)
    except Exception as e:
        pytest.skip(f"Rendern nicht moeglich: {type(e).__name__}: {e}")
    rgb, tiefe = kamera.bilder(d)
    kamera.close()
    return m, d, rgb, tiefe


def _in_welt(m, d, tiefe, maske):
    """Pixel (Maske) mit Intrinsik und Tiefe in Weltpunkte: optical frame = MuJoCo-Kamera um 180 Grad um x gedreht."""
    i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, KAMERA_NAME)
    fx, fy, cx, cy = intrinsik(*m.cam_resolution[i], m.cam_fovy[i])
    v, u = np.nonzero(maske)
    z = tiefe[v, u].astype(float)
    p_opt = np.stack([(u - cx) / fx * z, (v - cy) / fy * z, z], axis=1)
    R_welt_opt = d.cam_xmat[i].reshape(3, 3) @ _rot_x(math.pi)
    return p_opt @ R_welt_opt.T + d.cam_xpos[i]


def test_tiefe_entlang_der_blickachse_auf_der_wand(szene):
    m, d, rgb, tiefe = szene
    h, w = tiefe.shape
    cam_x = d.cam_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, KAMERA_NAME)][0]
    assert abs(tiefe[h // 2, w // 2] - (1.95 - cam_x)) < 0.01          # Bildmitte: Wand, waagerechte Blickachse
    zeile = np.zeros_like(tiefe, dtype=bool)
    zeile[h // 2, w // 2:] = True                                       # rechte Haelfte der Mittelzeile: nur Wand
    p = _in_welt(m, d, tiefe, zeile)
    assert np.allclose(p[:, 0], 1.95, atol=0.01)                        # alle Punkte auf der Wandflaeche


def test_bild_nicht_gespiegelt_roter_quader_links_und_an_seinem_ort(szene):
    m, d, rgb, tiefe = szene
    r, g, b = (rgb[..., k].astype(int) for k in range(3))
    rot = (r > 100) & (r > 2 * g) & (r > 2 * b)
    assert rot.sum() > 100
    assert np.nonzero(rot)[1].mean() < rgb.shape[1] / 2                 # links im Bild (Quader bei y = +0.5)
    p = _in_welt(m, d, tiefe, rot)
    assert abs(np.median(p[:, 0]) - 1.4) < 0.02                         # Vorderseite des Quaders
    assert 0.38 < np.median(p[:, 1]) < 0.62 and 0.98 < np.median(p[:, 2]) < 1.42


def test_kamerathread_liefert_bild_mit_stempel(szene):
    m, d, _, _ = szene
    fertig, ergebnis = threading.Event(), {}

    def bei_bild(rgb, tiefe, stempel):
        ergebnis.update(rgb=rgb, tiefe=tiefe, stempel=stempel)
        fertig.set()
    k = KameraThread(m, bei_bild, 0.2, 10.0)
    try:
        k.aufnehmen(d.qpos, "stempel-1")
        assert fertig.wait(10.0), "kein Bild vom Kamerathread"
    finally:
        k.stoppen()
    assert ergebnis["stempel"] == "stempel-1"
    assert ergebnis["rgb"].shape == (240, 320, 3) and ergebnis["tiefe"].dtype == np.float32
    assert np.isfinite(ergebnis["tiefe"][120, 160])                    # Bildmitte: Wand (gueltige Tiefe)
