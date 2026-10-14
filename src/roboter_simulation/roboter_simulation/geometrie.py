"""Quaternion-Hilfen der Simulation (MuJoCo-Konvention q = [w, x, y, z])."""
import numpy as np


def quat_zu_rotmat(q):
    """Rotationsmatrix (Koerper -> Welt)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotate_inverse(q, v):
    """v (Weltkoordinaten) in das Koerper-Koordinatensystem mit Orientierung q."""
    return quat_zu_rotmat(q).T @ np.asarray(v, dtype=float)


def quat_mul(a, b):
    """Hamilton-Produkt a * b."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])
