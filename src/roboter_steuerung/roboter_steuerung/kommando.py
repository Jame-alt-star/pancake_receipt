"""Aufbereitung des Fahrbefehls [vx, vy, wz] fuer die Policy (Layer 3), ohne ROS.

Reihenfolge je Regeltakt (steuereinheit_node._control_loop):
  Watchdog -> Rampe (cmd_ramped) -> + Kurshaltung (haltung.py) -> Begrenzung -> an die Policy (cmd)
cmd_target ist der zuletzt per /cmd_vel angeforderte Wunsch; Zustandswechsel pruefen bewusst gegen
cmd_target, nicht gegen cmd (sonst wuerden Haltungs-Korrekturen als Bewegungswunsch gelten).
"""
from typing import Optional

import numpy as np

# Maximale Aenderungsrate des Kommandos [1/s] fuer alle drei Achsen: ein Wechsel von vx 0.5 auf 0 dauert
# ~0.5 s statt eines Takts. Ein harter Sprung reisst die Policy aus dem Bereich, in dem sie trainiert wurde.
CMD_RAMP_RATE = 1.0


def rampe(ramped: np.ndarray, ziel: np.ndarray, dt: float, rate: float = CMD_RAMP_RATE) -> np.ndarray:
    """Naehert ramped pro Takt um hoechstens rate*dt je Achse an ziel an (neues Array)."""
    schritt = rate * dt
    return (ramped + np.clip(ziel - ramped, -schritt, schritt)).astype(np.float32)


class Kommandobegrenzung:
    """Begrenzung auf den Trainingsbereich der Policy plus kombinierte Grenze bei hoher Geschwindigkeit.

    Kombigrenze (gemessen 13.10.2026): bei |vx| > vx_ref sinkt die erlaubte |wz| linear bis wz_at_vmax
    bei vx_max; vx 1.0 + wz 1.0 fiel in ~15-20 % der Laeufe, die Linie (0.7, 1.0) -> (1.0, 0.5) war in
    10 Seeds je Punkt stabil."""

    def __init__(self, cmd_min, cmd_max, wz_vx_ref: Optional[float] = None, wz_at_vmax: Optional[float] = None):
        self.cmd_min = None if cmd_min is None else np.asarray(cmd_min, dtype=np.float32)
        self.cmd_max = None if cmd_max is None else np.asarray(cmd_max, dtype=np.float32)
        self.wz_vx_ref = wz_vx_ref
        self.wz_at_vmax = wz_at_vmax

    @staticmethod
    def aus_policy(config: dict) -> "Kommandobegrenzung":
        rng = config.get("cmd_training_range")
        if not rng:
            return Kommandobegrenzung(None, None)
        lo = [rng["lin_vel_x"][0], rng["lin_vel_y"][0], rng["ang_vel_z"][0]]
        hi = [rng["lin_vel_x"][1], rng["lin_vel_y"][1], rng["ang_vel_z"][1]]
        hs = config.get("cmd_wz_limit_high_speed")
        return Kommandobegrenzung(lo, hi, float(hs["vx_ref"]) if hs else None,
                                  float(hs["wz_at_vmax"]) if hs else None)

    def ausserhalb(self, cmd) -> bool:
        if self.cmd_min is None:
            return False
        return bool(np.any((cmd < self.cmd_min) | (cmd > self.cmd_max)))

    def begrenze(self, cmd: np.ndarray) -> np.ndarray:
        if self.cmd_min is None:
            return cmd
        cmd = np.clip(cmd, self.cmd_min, self.cmd_max).astype(np.float32)
        if self.wz_vx_ref is not None:
            vmax = float(max(abs(self.cmd_min[0]), abs(self.cmd_max[0])))
            wz_hi = float(self.cmd_max[2])
            v = abs(float(cmd[0]))
            if v > self.wz_vx_ref and vmax > self.wz_vx_ref:
                a = min(1.0, (v - self.wz_vx_ref) / (vmax - self.wz_vx_ref))
                cap = wz_hi + a * (self.wz_at_vmax - wz_hi)
                cmd[2] = float(np.clip(cmd[2], -cap, cap))
        return cmd


class Watchdog:
    """/cmd_vel-Watchdog: bleibt der Befehlsstrom laenger als timeout_s aus (z. B. Nav2 faellt aus), soll das
    Ziel auf 0 gehen, statt mit dem letzten Befehl weiterzulaufen. timeout_s <= 0 = aus."""

    def __init__(self, timeout_s: float):
        self.timeout_s = float(timeout_s)
        self.letzte = None

    def empfangen(self, t: float):
        self.letzte = t

    def abgelaufen(self, t: float) -> bool:
        return self.timeout_s > 0.0 and self.letzte is not None and t - self.letzte > self.timeout_s
