"""Beinodometrie (ROS-frei): schaetzt die Basisposition aus Gelenkwinkeln und IMU-Orientierung.

Prinzip: Das Standbein (tiefster Fuss) steht am Boden still. Bewegt sich dieser Fuss relativ zur Basis
(in Welt-ausgerichteten Koordinaten), hat sich die Basis um genau den umgekehrten Betrag bewegt.
Kinematik ueber das MuJoCo-Modell (nur Vorwaertskinematik, keine Physik) - auf echter Hardware
gleich verwendbar, da nur Gelenkwinkel und IMU-Orientierung eingehen.

Grenzen: Fussrutschen und Doppelstandphasen erzeugen Drift; Gier aus der IMU (auf echter Hardware
driftend). Fuer Langzeitgenauigkeit braucht es eine Korrektur von aussen (SLAM: map->odom).
"""
import numpy as np
import mujoco


class Beinodometrie:
    def __init__(self, xml_path: str, update_period_s: float = 0.01, hysterese_m: float = 0.003,
                 foot_bodies=("left_ankle_roll_link", "right_ankle_roll_link")):
        self.m = mujoco.MjModel.from_xml_path(xml_path)
        self.d = mujoco.MjData(self.m)
        self.feet = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, n) for n in foot_bodies]
        self.period = update_period_s
        self.hyst = hysterese_m
        self.pos = np.zeros(3)          # geschaetzte Basisposition (odom)
        self.vel = np.zeros(3)          # Weltgeschwindigkeit
        self.quat = np.array([1.0, 0, 0, 0])
        self._stance = None
        self._prev_foot = None
        self._t_last = None
        self._h0 = None                 # Hoehe Basis ueber Standfuss-Referenz

    def reset(self, pos):
        self.pos = np.asarray(pos, dtype=float).copy()
        self._stance = None
        self._prev_foot = None
        self._t_last = None
        self._h0 = None

    def _feet_rel(self, q, quat):
        self.d.qpos[:3] = 0.0
        self.d.qpos[3:7] = quat
        self.d.qpos[7:7 + len(q)] = q
        mujoco.mj_kinematics(self.m, self.d)
        return [self.d.xpos[b].copy() for b in self.feet]   # relativ zur Basis, welt-ausgerichtet

    def update(self, t: float, q, quat):
        """t [s], q Gelenkwinkel (MuJoCo-Reihenfolge), quat [w,x,y,z]. Gibt (pos, vel, quat) zurueck."""
        self.quat = np.asarray(quat, dtype=float)
        if self._t_last is not None and t - self._t_last < self.period:
            return self.pos, self.vel, self.quat
        feet = self._feet_rel(np.asarray(q, dtype=float), self.quat)
        z = [f[2] for f in feet]
        if self._stance is None:
            self._stance = int(np.argmin(z))
        # Bewegung seit dem letzten Update immer mit dem BISHERIGEN Standfuss verbuchen (der stand
        # bis eben); erst danach ggf. auf den neuen Standfuss wechseln. Vorher ging die Bewegung im
        # Wechselintervall verloren (-16 % Strecke beim Geradeausgehen).
        foot = feet[self._stance]
        if self._h0 is None:
            self._h0 = self.pos[2] + foot[2]
        if self._prev_foot is not None and self._t_last is not None:
            delta = foot - self._prev_foot                   # Fuss "bewegt" sich relativ zur Basis
            dt = t - self._t_last
            self.pos[:2] -= delta[:2]
            self.vel[:2] = -delta[:2] / dt if dt > 0 else 0.0
        self.pos[2] = self._h0 - foot[2]
        if z[1 - self._stance] < z[self._stance] - self.hyst:
            self._stance = 1 - self._stance                  # Wechsel des Standbeins
        self._prev_foot = feet[self._stance]
        self._t_last = t
        return self.pos, self.vel, self.quat
