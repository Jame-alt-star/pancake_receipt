"""Reine Rechenfunktionen der Sensorik (ohne ROS): Punktwolke lesen, in die Basis uebertragen, waagerecht
ausrichten, Hoehenband, 2D-Scan, Ausduennen. Getrennt vom ROS-Knoten (punktwolke_zu_scan.py), damit sie ohne ROS
getestet werden koennen (auch in der CI)."""
import math

import numpy as np

from roboter_beschreibung.geometrie import quat_zu_rotmat


def in_basis(pts, transform):
    """Punkte (N x 3, Sensorframe) mit einem TF-Transform (geometry_msgs/Transform: Basis <- Sensor) uebertragen."""
    q, t = transform.rotation, transform.translation
    return pts @ quat_zu_rotmat((q.w, q.x, q.y, q.z)).T + np.array([t.x, t.y, t.z])


def waagerecht(pts, quat):
    """Punkte (Sensorframe) mit der IMU-Orientierung waagerecht ausrichten: Roll/Nick entfernen,
    Gier bleibt (Blickrichtung wie der Sensor)."""
    R = quat_zu_rotmat(quat)
    yaw = math.atan2(R[1, 0], R[0, 0])
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz_t = np.array([[cy, sy, 0.0], [-sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return pts @ (rz_t @ R).T


def maske_aus_level(level, sensor_hoehe, min_hoehe, max_hoehe, r_min, r_max):
    h = level[:, 2] + sensor_hoehe
    r = np.hypot(level[:, 0], level[:, 1])
    return (h >= min_hoehe) & (h <= max_hoehe) & (r >= r_min) & (r <= r_max)


def scan_aus_level(level, mask, n_bins):
    ang = np.arctan2(level[mask, 1], level[mask, 0])
    idx = ((ang + math.pi) / (2 * math.pi) * n_bins).astype(int) % n_bins
    ranges = np.full(n_bins, np.inf)
    np.minimum.at(ranges, idx, np.hypot(level[mask, 0], level[mask, 1]))
    return ranges


def hoehenband_maske(pts, quat, sensor_hoehe, min_hoehe, max_hoehe, r_min, r_max):
    """Maske der Punkte im Hoehenband (waagerecht ausgerichtet) + waagerechte Koordinaten."""
    level = waagerecht(pts, quat)
    return maske_aus_level(level, sensor_hoehe, min_hoehe, max_hoehe, r_min, r_max), level


_NUMPY_TYP = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}   # PointField.datatype


def xyz_aus_wolke(data: bytes, fields, point_step: int, n: int, bigendian: bool = False) -> np.ndarray:
    """x, y, z aus einer beliebigen PointCloud2 (N x 3, float64), unabhaengig von weiteren Feldern.

    Echte Lidar-Treiber senden mehr als x/y/z (Livox MID-360 im G1: Intensitaet, Tag, Linie, Zeit; 18-32 Byte je
    Punkt). Frueher wurde nur genau 12 Byte (x, y, z als float32) akzeptiert und alles andere still als leere Wolke
    behandelt - auf echter Hardware haette es weder Scan noch Hindernisse gegeben. Punkte mit NaN/inf entfallen."""
    feld = {f.name: f for f in fields}
    if n == 0 or not all(k in feld for k in "xyz"):
        return np.zeros((0, 3))
    ordnung = ">" if bigendian else "<"
    dtype = np.dtype({"names": list("xyz"),
                      "formats": [ordnung + _NUMPY_TYP[feld[k].datatype] for k in "xyz"],
                      "offsets": [feld[k].offset for k in "xyz"], "itemsize": point_step})
    roh = np.frombuffer(bytes(data), dtype=dtype, count=n)
    pts = np.stack([roh["x"], roh["y"], roh["z"]], axis=1).astype(np.float64)
    return pts[np.isfinite(pts).all(axis=1)]


def ausduennen(pts, voxel=0.05):
    """Ein Punkt je Wuerfel der Kantenlaenge voxel."""
    if len(pts) == 0:
        return pts
    _, idx = np.unique(np.floor(pts / voxel).astype(np.int64), axis=0, return_index=True)
    return pts[np.sort(idx)]


def scan_aus_punkten(pts, quat, sensor_hoehe, min_hoehe, max_hoehe, n_bins, r_min, r_max):
    """pts: (N,3) im Sensorframe; quat [w,x,y,z] der Basis (Sensor fest montiert, nicht gedreht).
    Gibt ranges (n_bins,) zurueck, inf = kein Treffer."""
    level = waagerecht(pts, quat)
    return scan_aus_level(level, maske_aus_level(level, sensor_hoehe, min_hoehe, max_hoehe, r_min, r_max), n_bins)
