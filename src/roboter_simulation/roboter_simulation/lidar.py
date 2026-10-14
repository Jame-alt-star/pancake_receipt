"""Simuliertes Lidar: Raycast (mj_multiRay) auf die Welt-Geoms einer Gruppe (Standard 2 = Hindernisse).

Montage: lidar_link an base_link (0, 0, hoehe), kippt mit dem Roboter wie ein am Torso montierter Sensor.
- 2D: eine Scanebene in der Koerper-xy-Ebene (n Strahlen ueber 360 Grad), kein Treffer = inf.
- 3D: Ringe zwischen elev_min und elev_max (24 x 360 Strahlen), Treffer als Punkte im lidar_link-Frame.
Mit Rauschen (sigma) und Ausfaellen (p_ausfall), wenn ein Zufallsgenerator uebergeben wird.
"""
import mujoco
import numpy as np

from roboter_simulation.geometrie import quat_zu_rotmat

R_MIN = 0.15   # naeher liefert der Sensor nichts (eigener Koerper, Blindbereich)


class Lidar:

    def __init__(self, model, hoehe, n_strahlen, reichweite, geomgroup=2, ringe=0, elev_min_deg=-40.0,
                 elev_max_deg=10.0):
        self.model, self.hoehe, self.n, self.rmax = model, hoehe, n_strahlen, reichweite
        self.maske = np.zeros(6, dtype=np.uint8)
        self.maske[geomgroup] = 1
        a = np.linspace(-np.pi, np.pi, n_strahlen, endpoint=False)
        self.richtungen_2d = np.stack([np.cos(a), np.sin(a), np.zeros_like(a)], axis=1)
        self.richtungen_3d = None
        if ringe:
            el = np.radians(np.linspace(elev_min_deg, elev_max_deg, ringe))
            E, A = np.meshgrid(el, a, indexing="ij")
            self.richtungen_3d = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)],
                                          axis=-1).reshape(-1, 3)

    def _strahlen(self, data, quat, richtungen_lokal):
        R = quat_zu_rotmat(quat)
        ursprung = data.qpos[0:3] + R @ np.array([0.0, 0.0, self.hoehe])
        dirs = richtungen_lokal @ R.T
        n = len(dirs)
        dist = np.zeros(n, dtype=np.float64)
        geomid = np.zeros(n, dtype=np.int32)
        mujoco.mj_multiRay(self.model, data, ursprung, dirs.reshape(-1), self.maske, True, -1,
                           geomid, dist, None, n, self.rmax)
        return dist, geomid

    def scan(self, data, quat, rng=None, sigma=0.0, p_ausfall=0.0):
        """2D-Scan: Entfernungen je Strahl (inf = kein Treffer bzw. Ausfall)."""
        dist, geomid = self._strahlen(data, quat, self.richtungen_2d)
        ranges = np.where((geomid >= 0) & (dist >= 0) & (dist < self.rmax), dist, np.inf)
        if rng is not None:
            ranges = ranges + rng.normal(0.0, sigma, self.n)
            ranges[rng.random(self.n) < p_ausfall] = np.inf
        return ranges

    def wolke(self, data, quat, rng=None, sigma=0.0, p_ausfall=0.0):
        """3D-Punktwolke im lidar_link-Frame (float32, N x 3)."""
        dist, geomid = self._strahlen(data, quat, self.richtungen_3d)
        ok = (geomid >= 0) & (dist > R_MIN) & (dist < self.rmax)
        if rng is not None:
            ok &= rng.random(len(dist)) >= p_ausfall          # Ausfaelle wie beim 2D-Scan (Review U6)
        d_ok = dist[ok]
        if rng is not None and len(d_ok):
            d_ok = d_ok + rng.normal(0.0, sigma, len(d_ok))
        return (self.richtungen_3d[ok] * d_ok[:, None]).astype(np.float32)
