"""Quaternion-Hilfen fuer alle Pakete (Konvention q = [w, x, y, z] wie MuJoCo und die Policy).

Eine Stelle statt vorher sieben Kopien (Simulation, Sensorik, Beinodometrie, Haltung, Balance-Dienst, Testdouble,
Demo/Tests): ein Vorzeichenfehler muss so nur einmal behoben werden.
"""
import math

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


def yaw_aus_quat(q) -> float:
    """Gierwinkel [rad]."""
    w, x, y, z = q
    return float(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def neigung_grad(q) -> float:
    """Winkel zwischen Koerper-z-Achse und Senkrechter [Grad] (0 = aufrecht)."""
    _, x, y, _ = q
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))


def winkel_normieren(winkel: float) -> float:
    """Auf [-pi, pi]: ein Fehler von 359 Grad ist -1 Grad, nicht fast eine volle Drehung."""
    return float((winkel + math.pi) % (2.0 * math.pi) - math.pi)


def rotmat_aus_rpy(roll: float, pitch: float, yaw: float):
    """Rotationsmatrix aus Roll/Nick/Gier (ROS-Konvention wie static_transform_publisher: R = Rz * Ry * Rx)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])
