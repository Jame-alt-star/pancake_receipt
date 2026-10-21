"""Kurs- und Positionshaltung als duenne Schicht ueber der Policy (Layer 3), ohne ROS.

Die Policy haelt weder eine Weltrichtung noch eine Position: /cmd_vel gibt ihr nur eine koerperbezogene
Soll-Geschwindigkeit. Beobachtet: der Roboter dreht beim Gehen und Stehen langsam weg (mal links, mal rechts)
und verschiebt sich im Stand. Beide Regler sind einfache P-Regler mit begrenzter Korrektur; die Korrektur wird
jeden Takt neu berechnet und NICHT aufsummiert (Regression 13.10.2026: aufsummiert wuchs cmd_wz im Stand auf
-572 rad/s an).
"""
from typing import Optional, Tuple

import numpy as np

from roboter_beschreibung.geometrie import winkel_normieren, yaw_aus_quat

# Unterhalb dieser angeforderten Drehrate gilt "keine Drehung gewuenscht" und die Kurshaltung greift. Bewusst
# fast 0 (Review L2, vorher 0.05): Nav2 gibt in sanften Kurven kleine wz aus, die Kurshaltung hielt sonst den
# alten Kurs gegen den Pfadfolger.
YAW_CMD_SCHWELLE = 0.001
# Unterhalb dieser Geschwindigkeit (angefordert UND gerampt) gilt der Roboter als stehend (Positionshaltung).
LIN_CMD_SCHWELLE = 0.05
# P-Verstaerkungen [1/s] und Begrenzung der Korrektur (klein gegenueber echten Befehlen, damit ein
# Messausreisser die Policy nicht destabilisiert). Vorsichtig gewaehlt, nicht fein abgestimmt.
HEADING_HOLD_KP = 1.0
HEADING_HOLD_MAX_YAW_RATE = 0.3
POSITION_HOLD_KP = 1.0
POSITION_HOLD_MAX_SPEED = 0.15


class Kurshaltung:
    """Haelt die Ausrichtung, solange keine Drehung angefordert ist. Bei angeforderter Drehung folgt das Ziel
    der aktuellen Ausrichtung, damit danach die NEUE Richtung gehalten wird."""

    def __init__(self):
        self.ziel_yaw: Optional[float] = None

    def zuruecksetzen(self):
        """Neues Ziel beim naechsten Takt (z. B. bei Freigabe: der Roboter dreht im Stand ~0.5 deg/s weg;
        mit dem alten Ziel drehte er beim ersten Befehl zurueck)."""
        self.ziel_yaw = None

    def korrektur(self, quat, wz_angefordert: float) -> float:
        yaw = yaw_aus_quat(quat)
        if self.ziel_yaw is None or abs(wz_angefordert) >= YAW_CMD_SCHWELLE:
            self.ziel_yaw = yaw
            return 0.0
        fehler = winkel_normieren(self.ziel_yaw - yaw)
        return float(np.clip(HEADING_HOLD_KP * fehler, -HEADING_HOLD_MAX_YAW_RATE, HEADING_HOLD_MAX_YAW_RATE))


class Positionshaltung:
    """Haelt im Stand die Position (nur Zustand Balancieren). Ziel wird erst erfasst, wenn kein Befehl mehr
    ansteht UND die Rampe abgeklungen ist. Gemessen 13.10.2026 (synchroner Kreis): ~1 cm in 60 s statt 32 cm
    Drift; seitlicher Stoss 0.3 m/s -> max. 4 cm."""

    def __init__(self):
        self.ziel: Optional[np.ndarray] = None

    def zuruecksetzen(self):
        self.ziel = None

    def korrektur(self, position, quat, cmd_ziel, cmd_rampe) -> Tuple[float, float]:
        """(vx, vy) im Koerper-Frame."""
        xy = np.asarray(position[:2], dtype=np.float32)
        if (np.any(np.abs(np.asarray(cmd_ziel)[:2]) >= LIN_CMD_SCHWELLE)
                or np.any(np.abs(np.asarray(cmd_rampe)[:2]) >= LIN_CMD_SCHWELLE)):
            self.ziel = None
            return 0.0, 0.0
        if self.ziel is None:
            self.ziel = xy.copy()
            return 0.0, 0.0
        dx, dy = self.ziel - xy
        yaw = yaw_aus_quat(quat)
        vor = np.cos(yaw) * dx + np.sin(yaw) * dy
        seit = -np.sin(yaw) * dx + np.cos(yaw) * dy
        g = POSITION_HOLD_MAX_SPEED
        return (float(np.clip(POSITION_HOLD_KP * vor, -g, g)), float(np.clip(POSITION_HOLD_KP * seit, -g, g)))
