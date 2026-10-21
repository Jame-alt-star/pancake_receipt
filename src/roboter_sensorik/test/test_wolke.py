"""Sensorik-Rechenfunktionen (roboter_sensorik/wolke.py) ohne ROS: Scan aus Punktwolke, Formate, Montage."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from roboter_sensorik.wolke import scan_aus_punkten  # noqa: E402

H0 = 1.24


def quat_rpy(roll=0.0, pitch=0.0, yaw=0.0):
    cr, sr, cp, sp, cy, sy = (math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2),
                              math.sin(pitch / 2), math.cos(yaw / 2), math.sin(yaw / 2))
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def run(pts, quat):
    return scan_aus_punkten(np.array(pts, float), quat, H0, 0.12, 1.9, 360, 0.2, 10.0)


def bin_of(deg):
    return int((math.radians(deg) + math.pi) / (2 * math.pi) * 360) % 360


def test_boden_ausgeblendet_niedriges_hindernis_behalten():
    pts = [[3.0, 0.0, -H0],            # Boden vorne
           [2.0, 0.0, 0.30 - H0],      # niedrige Palette (0.30 m hoch) vorne
           [0.0, 2.5, 0.0]]            # Wand links auf Sensorhoehe
    r = run(pts, quat_rpy())
    assert r[bin_of(0)] == np.float64(2.0)       # Palette sichtbar, Boden dahinter ignoriert
    assert abs(r[bin_of(90)] - 2.5) < 1e-6
    assert np.isinf(r[bin_of(180)])


def test_nur_boden_ergibt_keinen_treffer():
    r = run([[x, y, -H0] for x in (1, 2, 4) for y in (-1, 0, 1)], quat_rpy())
    assert np.all(np.isinf(r))


def test_neigung_des_roboters_wird_ausgeglichen():
    # Roboter (und Sensor) 4 Grad nach vorn geneigt: ferner Boden liegt im Sensorframe "hoeher".
    pitch = math.radians(4.0)
    q = quat_rpy(pitch=pitch)
    w, x, y, z = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    boden_welt = np.array([[8.0, 0.0, -H0]])
    boden_sensor = boden_welt @ R          # Welt -> Sensorframe (R^T v)
    assert boden_sensor[0, 2] + H0 > 0.12  # ohne Ausgleich waere der Boden ein "Hindernis"
    assert np.all(np.isinf(run(boden_sensor, q)))


def test_gierwinkel_bleibt_im_sensorframe():
    r = run([[2.0, 0.0, 0.0]], quat_rpy(yaw=math.radians(90)))
    assert abs(r[bin_of(0)] - 2.0) < 1e-6    # Scan ist im Sensorframe, nicht in Weltkoordinaten


def test_ausduennen_und_hoehenband():
    from roboter_sensorik.wolke import ausduennen, hoehenband_maske
    pts = np.array([[2.0, 0.0, -H0], [2.0, 0.0, 0.3 - H0], [2.01, 0.01, 0.3 - H0], [3.0, 1.0, 0.0]])
    mask, _ = hoehenband_maske(pts, quat_rpy(), H0, 0.12, 1.9, 0.2, 10.0)
    assert mask.tolist() == [False, True, True, True]          # Boden raus
    assert len(ausduennen(pts[mask], 0.05)) == 2                 # zwei Nachbarpunkte -> einer


def test_wolke_mit_weiteren_feldern_wie_livox():
    """Echte Treiber senden mehr Felder als x/y/z (Livox: + Intensitaet, Tag, Linie, Zeit; 26 Byte je Punkt).
    Regression: frueher wurde jede Wolke mit point_step != 12 still verworfen."""
    from types import SimpleNamespace as F
    from roboter_sensorik.wolke import xyz_aus_wolke
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("tag", "u1"), ("line", "u1"),
                   ("timestamp", "<f8")])
    roh = np.zeros(3, dtype=dt)
    roh["x"], roh["y"], roh["z"], roh["intensity"] = [1.0, 2.0, np.nan], [0.5, -1.0, 0.0], [0.1, 0.2, 0.3], 7.0
    fields = [F(name=n, offset=dt.fields[n][1], datatype=t) for n, t in
              (("x", 7), ("y", 7), ("z", 7), ("intensity", 7), ("tag", 2), ("line", 2), ("timestamp", 8))]
    pts = xyz_aus_wolke(roh.tobytes(), fields, dt.itemsize, 3)
    assert dt.itemsize == 26 and pts.shape == (2, 3)                  # NaN-Punkt entfaellt
    assert np.allclose(pts, [[1.0, 0.5, 0.1], [2.0, -1.0, 0.2]], atol=1e-6)
    # klassische 12-Byte-Wolke weiterhin
    xyz = np.array([[1, 2, 3], [4, 5, 6]], dtype="<f4")
    f12 = [F(name=n, offset=4 * i, datatype=7) for i, n in enumerate("xyz")]
    assert np.allclose(xyz_aus_wolke(xyz.tobytes(), f12, 12, 2), xyz)
    assert xyz_aus_wolke(b"", f12, 12, 0).shape == (0, 3)


def test_geneigte_montage_ueber_tf():
    """Lidar 0.45 m ueber dem Becken, um 30 Grad nach vorn geneigt (wie am Kopf des G1): ein Punkt, der im
    Sensorframe schraeg nach unten liegt, landet nach der Uebertragung in base_link an der richtigen Stelle."""
    from types import SimpleNamespace as N
    from roboter_sensorik.wolke import in_basis, scan_aus_punkten
    nick = math.radians(30)
    q = N(w=math.cos(nick / 2), x=0.0, y=math.sin(nick / 2), z=0.0)            # Drehung um y (nach vorn geneigt)
    tf = N(rotation=q, translation=N(x=0.0, y=0.0, z=0.45))
    # Hindernis 2 m vor dem Becken auf 0.3 m Hoehe -> im Basisframe (2, 0, 0.3 - 0.793)
    ziel_basis = np.array([2.0, 0.0, 0.3 - 0.793])
    R = np.array([[math.cos(nick), 0, math.sin(nick)], [0, 1, 0], [-math.sin(nick), 0, math.cos(nick)]])
    p_sensor = (ziel_basis - np.array([0, 0, 0.45])) @ R                       # Umkehrung: R^T (p - t)
    pts = in_basis(np.array([p_sensor]), tf)
    assert np.allclose(pts[0], ziel_basis, atol=1e-9)
    ranges = scan_aus_punkten(pts, np.array([1.0, 0, 0, 0]), 0.793, 0.12, 1.9, 360, 0.2, 10.0)
    assert abs(ranges[bin_of(0)] - 2.0) < 1e-6                                 # Hindernis im Scan, 2 m voraus
