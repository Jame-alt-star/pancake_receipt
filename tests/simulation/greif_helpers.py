"""Regelkreis Greifen ohne ROS: echte Policy + Robotermodell mit Haenden + Welt greiftisch (Kiste mit Physik).

Bildet nach, was im System die Knoten tun, und benutzt dafuer deren ROS-freie Bausteine:
  - Steuereinheit: Policy (50 Hz), Kurs-/Positionshaltung (vereinfacht), Armfuehrung (armfuehrung.py) mit
    maskierter Policy-Beobachtung und gemischten Armzielen (greifen/manipulation.py)
  - Greif-Dienst: Greifauftrag (greifen/dienst.py) - Anfahrt, Greif-/Ablegeablauf, Pruefung, Zuruecktreten
  - Simulation: PD fuer Koerper und Finger (500 Hz), Objektlagen relativ zum Becken (wie mujoco_node /objekte/<name>)
So prueft der Test dieselbe Logik, die im System laeuft; nur ROS (Topics, Zeitversatz) fehlt.
"""
import sys

import mujoco
import numpy as np
import yaml

from sim_helpers import BESCHREIBUNG, STEUER, WS, load_config, load_controller29

for p in (STEUER, WS + "roboter_simulation"):
    if p not in sys.path:
        sys.path.insert(0, p)

from roboter_simulation.szene import lade_modell  # noqa: E402
from roboter_steuerung.armfuehrung import Armfuehrung  # noqa: E402
from roboter_steuerung.control.rl_locomotion_controller_29dof import MUJOCO_JOINT_ORDER  # noqa: E402
from roboter_steuerung.greifen import ablauf as greifablauf  # noqa: E402
from roboter_steuerung.greifen.anfahrt import Anfahrt, standpose, winkel  # noqa: E402,F401
from roboter_steuerung.greifen.dienst import (AUSBLENDEN, FUEHREN, Eingang, Greifauftrag,  # noqa: E402
                                              Greifparameter)
from roboter_steuerung.greifen.kinematik import Armkinematik  # noqa: E402
from roboter_steuerung.greifen.manipulation import ARM_INDIZES, maskiere_arme, mische  # noqa: E402
from roboter_steuerung.haltung import Kurshaltung, Positionshaltung  # noqa: E402
from roboter_steuerung.kommando import Kommandobegrenzung, rampe  # noqa: E402

WELT = WS + "roboter_welten/welten/greiftisch/szene.xml"
KISTE_HALBBREITE = 0.10      # y-Halbmass der Kiste (szene.xml)
TISCH_OBERKANTE = 0.74
ARMNAMEN = [MUJOCO_JOINT_ORDER[i] for i in ARM_INDIZES]


def roboter_config():
    return yaml.safe_load(open(BESCHREIBUNG + "config/roboter.yaml"))


def yaw_aus(q):
    w, x, y, z = q
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


class Kreis:
    """Synchroner Kreis (500 Hz Physik, 50 Hz Policy). Aufrufer setzt Fahrwunsch (cmd), Armkommandos, Fingerziele."""

    def __init__(self, start=(0.0, 0.0, 0.0), hand_kp=None, hand_kd=None, taille_pitch=-0.15, kiste=None):
        """start: Basis (x, y, yaw). kiste: Startlage der Kiste (x, y, yaw) auf dem Lagertisch, None = wie szene.xml."""
        self.cfg = load_config()
        rob = roboter_config()
        haende = rob["haende"]
        self.ctl = load_controller29(self.cfg)
        self.m = lade_modell(BESCHREIBUNG + rob["modell_xml_haende"], WELT)
        self.m.opt.timestep = 0.002
        self.d = mujoco.MjData(self.m)
        tid = self.m.actuator_trnid[:, 0]
        self.qa, self.va = self.m.jnt_qposadr[tid], self.m.jnt_dofadr[tid]
        self.kps = np.array(self.cfg["kps"])
        self.kds = np.array(self.cfg["kds"])
        import os
        f = float(os.environ.get('ARM_KP_TEST', '1'))   # Versuch: steifere Arme (Faktor auf kp und kd der Armgelenke)
        self.kps[15:29] *= f
        self.kds[15:29] *= f ** 0.5
        self.home = np.array(self.cfg["default_angles"], dtype=np.float32)
        self.hkp = np.full(14, haende["kp"] if hand_kp is None else hand_kp, dtype=float)
        self.hkd = np.full(14, haende["kd"] if hand_kd is None else hand_kd, dtype=float)
        self.taille_pitch = taille_pitch
        self.kiste_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "kiste")
        kj = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "kiste_frei")
        self.kiste_q = self.m.jnt_qposadr[kj]
        if kiste is not None:
            kx, ky, kyaw = kiste
            self.d.qpos[self.kiste_q:self.kiste_q + 2] = (kx, ky)
            self.d.qpos[self.kiste_q + 3:self.kiste_q + 7] = (np.cos(kyaw / 2), 0, 0, np.sin(kyaw / 2))
        x, y, yaw = start
        self.d.qpos[0:2] = (x, y)
        self.d.qpos[3:7] = (np.cos(yaw / 2), 0, 0, np.sin(yaw / 2))
        self.d.qpos[self.qa[:29]] = self.home
        self.d.qpos[self.qa[29:]] = greifablauf.HAND_OFFEN
        mujoco.mj_forward(self.m, self.d)
        self.tgt = self.home.copy()
        self.htgt = greifablauf.HAND_OFFEN.copy()
        self.af = Armfuehrung(ARMNAMEN)          # wie in der Steuereinheit (Standardwerte = steuerung.yaml)
        self.cmd = np.zeros(3, dtype=np.float32)
        self.cmd_rampe = np.zeros(3, dtype=np.float32)
        self.kurshaltung = Kurshaltung()
        self.positionshaltung = Positionshaltung()
        self.begrenzung = Kommandobegrenzung.aus_policy(self.cfg)
        self.gestuerzt = None
        self.kin = Armkinematik(BESCHREIBUNG + rob["modell_xml_haende"], MUJOCO_JOINT_ORDER)
        self.max_neigung = 0.0
        self.protokoll = []                      # (t, phase) bei jedem Phasenwechsel eines Auftrags
        self.beobachter = None                   # optional: f(kreis) je Policy-Takt (Diagnose, Video)

    # ------------------------------------------------------------------ Zustand
    @property
    def t(self):
        return float(self.d.time)

    def basis(self):
        return float(self.d.qpos[0]), float(self.d.qpos[1]), yaw_aus(self.d.qpos[3:7])

    def kiste(self):
        q = self.d.qpos[self.kiste_q:self.kiste_q + 7]
        return q[:3].copy(), yaw_aus(q[3:7])

    def koerper(self):
        return self.d.qpos[self.qa[:29]].copy()

    def kin_setzen(self):
        """Kinematik auf den aktuellen Zustand (Basis, Koerpergelenke)."""
        self.kin.setze(self.d.qpos[0:3], self.d.qpos[3:7], self.koerper())
        return self.kin

    def relativ(self, p, q):
        """Lage relativ zum Becken, wie mujoco_node sie auf /objekte/<name> sendet: (Position, Quaternion)."""
        pb, qb = self.d.qpos[0:3], self.d.qpos[3:7]
        qinv = np.zeros(4)
        mujoco.mju_negQuat(qinv, qb)
        rp = np.zeros(3)
        mujoco.mju_rotVecQuat(rp, np.asarray(p, dtype=float) - pb, qinv)
        rq = np.zeros(4)
        mujoco.mju_mulQuat(rq, qinv, np.asarray(q, dtype=float))
        return rp, rq

    def lage_kiste(self):
        return self.relativ(self.d.xpos[self.kiste_id], self.d.xquat[self.kiste_id])

    def lage_site(self, name):
        sid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, name)
        assert sid >= 0, f"Site {name!r} nicht in der Welt (Konvention ablage_<ort>)"
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, self.d.site_xmat[sid])
        return self.relativ(self.d.site_xpos[sid], q)

    # ------------------------------------------------------------------ Regelung
    def arm_kommando(self, modus, ziele=None, taille=0.0):
        """Wie eine Nachricht auf /arme/kommando an die Steuereinheit."""
        fehler = self.af.empfange(modus, ARMNAMEN, ziele if ziele is not None else [], taille, self.t)
        assert fehler is None, fehler

    def _befehl(self):
        """Fahrbefehl wie SteuereinheitNode._bereite_befehl_auf + befehl_gehen/befehl_stehen (Rampe, Kurshaltung,
        Begrenzung; im Stand Positionshaltung). self.cmd ist der angeforderte Wunsch (cmd_target)."""
        quat = self.d.qpos[3:7].copy()
        self.cmd_rampe = rampe(self.cmd_rampe, self.cmd, 0.02)
        cmd = self.cmd_rampe.copy()
        cmd[2] += self.kurshaltung.korrektur(quat, float(self.cmd[2]))
        cmd = self.begrenzung.begrenze(cmd)
        if float(np.max(np.abs(self.cmd))) < 0.05:          # Balancieren: Stehen mit Positionshaltung
            vx, vy = self.positionshaltung.korrektur(self.d.qpos[0:3], quat, self.cmd, self.cmd_rampe)
            cmd = np.array([self.cmd_rampe[0] + vx, self.cmd_rampe[1] + vy, cmd[2]], dtype=np.float32)
        else:
            self.positionshaltung.zuruecksetzen()
        return cmd.astype(np.float32)

    def _policy(self):
        q, dq = self.d.qpos[self.qa[:29]].copy(), self.d.qvel[self.va[:29]].copy()
        q_mess = q.copy()
        if self.af.aktiv:                        # wie IRobotState._policy_schritt
            q, dq = maskiere_arme(q, dq, self.home)
        cmd = self._befehl()
        z = self.ctl.compute_joint_targets(q, dq, self.d.qpos[3:7].copy(), self.d.qvel[3:6].copy(), cmd)
        if self.af.aktiv:                        # wie SteuereinheitNode.arme_anwenden
            erg = self.af.takt(z[ARM_INDIZES], q_mess[ARM_INDIZES], self.t, 0.02)
            if erg is not None:
                arme, gewicht, taille = erg
                z = mische(z, arme, gewicht, self.taille_pitch, gewicht * taille)
        self.tgt = z

    def laufe(self, dauer, jeder_takt=None):
        """dauer s simulieren; jeder_takt(kreis) wird vor jedem Policy-Takt (50 Hz) aufgerufen, Rueckgabe True = Ende."""
        ende = self.t + dauer
        while self.t < ende:
            for _ in range(10):
                q, dq = self.d.qpos[self.qa], self.d.qvel[self.va]
                self.d.ctrl[:29] = (self.tgt - q[:29]) * self.kps - dq[:29] * self.kds
                self.d.ctrl[29:] = (self.htgt - q[29:]) * self.hkp - dq[29:] * self.hkd
                mujoco.mj_step(self.m, self.d)
            g = 1 - 2 * (self.d.qpos[4] ** 2 + self.d.qpos[5] ** 2)
            self.max_neigung = max(self.max_neigung, float(np.degrees(np.arccos(np.clip(g, -1, 1)))))
            if self.d.qpos[2] < 0.4:
                self.gestuerzt = self.t
                return False
            if self.beobachter is not None:
                self.beobachter(self)
            if jeder_takt is not None and jeder_takt(self):
                self._policy()
                return True
            self._policy()
        return True

    # ------------------------------------------------------------------ Bausteine (Experimente)
    def anfahren(self, soll, max_s=30.0, **kw):
        af = Anfahrt(soll, **kw)
        fertig = {"ok": False}

        def takt(k):
            vx, vy, wz, ok = af.befehl(k.basis(), k.t)
            k.cmd[:] = (vx, vy, wz)
            fertig["ok"] = ok
            return ok
        self.laufe(max_s, takt)
        self.cmd[:] = 0
        return fertig["ok"], af.fehler(self.basis())

    def fuehre_aus(self, ab):
        """Ablauf abspielen: je Takt ein FUEHREN-Kommando an die Armfuehrung, Fingerziele direkt."""
        t0 = self.t

        def takt(k):
            arme, haende, taille, _, fertig = ab.sollwerte(k.t - t0)
            k.arm_kommando(FUEHREN, arme, taille)
            k.htgt = haende
            return fertig
        self.laufe(ab.dauer + 1.0, takt)

    def ausblenden(self, dauer=2.0):
        self.arm_kommando(AUSBLENDEN)
        self.laufe(dauer)

    def gehe(self, cmd, dauer):
        self.cmd[:] = cmd
        self.laufe(dauer)
        self.cmd[:] = 0

    def _arme_jetzt(self):
        return self.af.ausgabe.copy() if self.af.ausgabe is not None else self.koerper()[ARM_INDIZES]

    def plane_greifen(self):
        kp, kyaw = self.kiste()
        k = self.kin_setzen()
        return greifablauf.plane_greifen(k, self._arme_jetzt(), self.htgt, kp, kyaw, KISTE_HALBBREITE,
                                         self.home[15:29], float(self.home[14]), self.taille_pitch,
                                         self.af.taille_anteil)

    def plane_ablegen(self, ablage_pos, ablage_yaw):
        k = self.kin_setzen()
        return greifablauf.plane_ablegen(k, self._arme_jetzt(), self.htgt, ablage_pos, ablage_yaw, KISTE_HALBBREITE,
                                         self.home[15:29], float(self.home[14]), self.taille_pitch,
                                         self.af.taille_anteil)

    # ------------------------------------------------------------------ Greif-Dienst
    def eingang(self, ablage_site=None):
        return Eingang(t=self.t, pos=self.d.qpos[0:3].copy(), quat=self.d.qpos[3:7].copy(), gelenke=self.koerper(),
                       objekt=self.lage_kiste(), ablage=self.lage_site(ablage_site) if ablage_site else None)

    def auftrag(self, modus, ablage_site=None, par=None, vorher=None, max_s=120.0):
        """Einen Greifauftrag wie greif_dienst_node ausfuehren. vorher: vorheriger (gegriffener) Auftrag, dessen
        Tragepose uebernommen wird. Rueckgabe: (Greifauftrag, letzte Ausgabe)."""
        par = par or Greifparameter(taille_pitch=self.taille_pitch)
        kw = {}
        if vorher is not None and vorher.arme is not None:
            kw = dict(start_arme=vorher.arme, start_haende=vorher.haende, start_taille=vorher.taille)
        ga = Greifauftrag(modus, self.kin, self.home, par, **kw)
        letzte = {"a": None, "phase": None}

        def takt(k):
            a = ga.takt(k.eingang(ablage_site))
            if a.cmd is not None:
                k.cmd[:] = a.cmd
            if a.arm_modus:
                k.arm_kommando(a.arm_modus, a.arme, a.taille)
            if a.haende is not None:
                k.htgt = np.asarray(a.haende, dtype=float)
                k.hkp[:], k.hkd[:] = par.hand_kp, par.hand_kd
            name = f"{ga.phase}/{a.phase}"
            if name != letzte["phase"]:
                letzte["phase"] = name
                k.protokoll.append((round(k.t, 2), modus, name))
            letzte["a"] = a
            return a.fertig
        self.laufe(max_s, takt)
        self.cmd[:] = 0
        return ga, letzte["a"]
