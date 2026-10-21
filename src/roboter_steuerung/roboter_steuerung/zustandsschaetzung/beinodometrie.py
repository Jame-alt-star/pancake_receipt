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


def kinematikmodell(xml_path: str) -> mujoco.MjModel:
    """Robotermodell ohne Netz-Geometrien: fuer die Vorwaertskinematik genuegen Glieder und Gelenke. Mit allen Netzen
    belegte das Kompilieren ~410 MB Arbeitsspeicher, ohne ~15 MB; Koerperposen, Massen und Traegheiten bleiben gleich
    (gemessen 21.10.2026, Test in tests/simulation/test_beinodometrie.py). Unter WSL2 mit 2,8 GB lagerte das System
    sonst bei Simulation mit Viewer aus."""
    spec = mujoco.MjSpec.from_file(xml_path)
    for g in [g for g in spec.geoms if g.type == mujoco.mjtGeom.mjGEOM_MESH]:
        spec.delete(g)
    for netz in list(spec.meshes):
        spec.delete(netz)
    return spec.compile()


class Beinodometrie:
    def __init__(self, xml_path: str, update_period_s: float = 0.009, hysterese_m: float = 0.003,
                 foot_bodies=("left_ankle_roll_link", "right_ankle_roll_link")):
        self.m = kinematikmodell(xml_path)
        self.d = mujoco.MjData(self.m)
        self.feet = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, n) for n in foot_bodies]
        # 0.009 statt 0.01 s (Review L7): /joint_states kommt nominal mit 100 Hz, aber mit Jitter;
        # bei exakt 0.01 fiel ein Teil der Nachrichten unter die Schwelle und wurde uebersprungen.
        self.period = update_period_s
        # Geschwindigkeit: Tiefpass (Zeitkonstante tau). Die Rohgeschwindigkeit aus der Fussbewegung
        # pro 10-ms-Schritt ist stark verrauscht (Ausreisser beim Standbeinwechsel; gemessen
        # Einzelwerte von -0.09 m/s bei 0.4 m/s Gehen). Die Position wird ungefiltert integriert.
        self.vel_tau = 0.15
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
            if dt > 0:
                a = dt / (self.vel_tau + dt)
                self.vel[:2] += a * (-delta[:2] / dt - self.vel[:2])
        self.pos[2] = self._h0 - foot[2]
        if z[1 - self._stance] < z[self._stance] - self.hyst:
            self._stance = 1 - self._stance                  # Wechsel des Standbeins
        self._prev_foot = feet[self._stance]
        self._t_last = t
        return self.pos, self.vel, self.quat
