"""Arm-Kinematik fuer das beidhaendige Greifen (ROS-frei): Inverse Kinematik auf dem MuJoCo-Kinematikmodell.

Gesteuert wird ein Punkt der Handflaeche je Hand (Mitte der Innenseite, HANDFLAECHE) in Position und Ausrichtung.
Verfahren wie im gemessenen Machbarkeitstest (experimente/arm_feasibility.py): gedaempfte kleinste Quadrate auf der
MuJoCo-Jacobi-Matrix, nur die 7 Armgelenke je Seite bewegen sich, Basis und uebrige Gelenke bleiben fest. Neu ist die
Ausrichtung (die Handflaechen sollen zur Kiste zeigen) und der Punkt auf der Handflaeche statt des Handgelenks.

Dex3-Geometrie im Handgelenk-Frame (wrist_yaw_link, gemessen am Modell g1_29dof_mit_haenden.xml): Finger zeigen nach
+x, die Hand schliesst links nach -y, rechts nach +y. Die Kollisionsbox der Handflaeche liegt 8,4 cm vor dem
Handgelenk, ihre Innenseite 4,0 cm neben der Handgelenkachse. Mit Ausrichtung = Kisten-Frame (x nach vorn, z nach
oben) zeigt die linke Handflaeche nach -y, die rechte nach +y: beide zur Kiste, wenn links bei +y und rechts bei -y
gegriffen wird.
"""
import mujoco
import numpy as np

from roboter_steuerung.zustandsschaetzung.beinodometrie import kinematikmodell

SEITEN = ("left", "right")
ARMGELENKE = {s: [f"{s}_shoulder_pitch_joint", f"{s}_shoulder_roll_joint", f"{s}_shoulder_yaw_joint",
                  f"{s}_elbow_joint", f"{s}_wrist_roll_joint", f"{s}_wrist_pitch_joint", f"{s}_wrist_yaw_joint"]
              for s in SEITEN}
# Punkt auf der Innenseite der Handflaeche im Handgelenk-Frame [m]
HANDFLAECHE = {"left": np.array([0.084, -0.040, 0.0]), "right": np.array([0.084, 0.040, 0.0])}
# Gewicht der Ausrichtung gegen die Position (rad gegen m): 0,3 rad Fehler zaehlt wie 9 cm
GEWICHT_DREHUNG = 0.3


def drehfehler(r_ist: np.ndarray, r_soll: np.ndarray) -> np.ndarray:
    """Kleiner Drehvektor (Welt), der r_ist naeherungsweise in r_soll ueberfuehrt."""
    return 0.5 * sum(np.cross(r_ist[:, i], r_soll[:, i]) for i in range(3))


def yaw_matrix(yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class Armkinematik:
    """Kinematik des Roboters mit Haenden. gelenknamen = Reihenfolge der 29 Koerpergelenke (MUJOCO_JOINT_ORDER)."""

    def __init__(self, xml_path: str, gelenknamen):
        self.m = kinematikmodell(xml_path)
        self.d = mujoco.MjData(self.m)
        self.gelenknamen = list(gelenknamen)
        jid = lambda n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)   # noqa: E731
        self._qadr = np.array([self.m.jnt_qposadr[jid(n)] for n in self.gelenknamen])
        self.handgelenk = {s: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, f"{s}_wrist_yaw_link")
                           for s in SEITEN}
        self._arm = {}
        for s in SEITEN:
            ids = [jid(n) for n in ARMGELENKE[s]]
            self._arm[s] = dict(q=np.array([self.m.jnt_qposadr[j] for j in ids]),
                                v=np.array([self.m.jnt_dofadr[j] for j in ids]),
                                lo=self.m.jnt_range[ids, 0].copy(), hi=self.m.jnt_range[ids, 1].copy())

    def setze(self, basis_pos, basis_quat, gelenke29):
        """Basis-Pose (Welt) und die 29 Koerpergelenke (MUJOCO_JOINT_ORDER) setzen; Finger bleiben 0."""
        self.d.qpos[:] = 0.0
        self.d.qpos[0:3] = basis_pos
        self.d.qpos[3:7] = basis_quat
        self.d.qpos[self._qadr] = gelenke29
        mujoco.mj_kinematics(self.m, self.d)

    def arme(self) -> np.ndarray:
        """Aktuelle 14 Armwinkel (links 7, rechts 7) = Gelenke 15..28 in MUJOCO_JOINT_ORDER."""
        return np.concatenate([self.d.qpos[self._arm[s]["q"]] for s in SEITEN])

    def setze_gelenk(self, name: str, wert: float):
        """Ein Koerpergelenk setzen (z. B. die Taille fuer die Planung mit geneigtem Oberkoerper)."""
        self.d.qpos[self._qadr[self.gelenknamen.index(name)]] = wert
        mujoco.mj_kinematics(self.m, self.d)

    def gelenk(self, name: str) -> float:
        return float(self.d.qpos[self._qadr[self.gelenknamen.index(name)]])

    def setze_arme(self, arme14):
        """14 Armwinkel (links, rechts) setzen."""
        self.d.qpos[self._arm["left"]["q"]] = arme14[:7]
        self.d.qpos[self._arm["right"]["q"]] = arme14[7:]
        mujoco.mj_kinematics(self.m, self.d)

    def schwerkraft_vorhalt(self, arme14, kp, last_kg: float = 0.0) -> np.ndarray:
        """Sollwert-Vorhalt (14 rad) gegen das Durchhaengen der Arme: Die Gelenke werden nur ueber PD geregelt
        (kp 40 Nm/rad); um eine Pose gegen die Schwerkraft zu halten, braucht ein Gelenk das Haltemoment tau, das
        der PD nur mit einer Abweichung tau/kp aufbringt (gemessen 11.11.2026: Kiste 7-8 cm unter dem Soll).
        tau = Schwerkraft der Arme (qfrc_bias bei Geschwindigkeit 0) + J^T * Gewichtskraft der Last, je Hand die
        Haelfte am Handflaechenpunkt. Rueckgabe tau/kp; der Aufrufer addiert ihn auf die IK-Loesung."""
        self.setze_arme(arme14)
        self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)
        kp = np.broadcast_to(np.asarray(kp, dtype=float), (14,))
        tau = []
        for s in SEITEN:
            v = self._arm[s]["v"]
            t = self.d.qfrc_bias[v].copy()
            if last_kg > 0.0:
                jacp = np.zeros((3, self.m.nv))
                p, _ = self.handflaeche(s)
                mujoco.mj_jac(self.m, self.d, jacp, None, p, self.handgelenk[s])
                t += jacp[:, v].T @ np.array([0.0, 0.0, 0.5 * last_kg * 9.81])
            tau.append(t)
        return np.concatenate(tau) / kp

    def loese_bester(self, ziele: dict, startwerte):
        """IK von mehreren Startwerten (je 14 Armwinkel) aus, bestes Ergebnis (Position zuerst). Die gedaempfte IK
        bleibt sonst leicht in einem schlechten Nebenminimum haengen (gemessen: Anheben aus der Zugreif-Pose 13 cm
        Fehler, aus der Ruhepose 0 cm)."""
        bestes = None
        for start in startwerte:
            self.setze_arme(start)
            arme, fp, fr = self.loese(ziele)
            if bestes is None or fp + 0.05 * fr < bestes[1] + 0.05 * bestes[2]:
                bestes = (arme, fp, fr)
        self.setze_arme(bestes[0])
        return bestes

    def handflaeche(self, seite: str):
        """Welt-Position des Handflaechenpunkts und Ausrichtung des Handgelenks (3x3)."""
        b = self.handgelenk[seite]
        r = self.d.xmat[b].reshape(3, 3)
        return self.d.xpos[b] + r @ HANDFLAECHE[seite], r.copy()

    def loese(self, ziele: dict, iterationen: int = 300):
        """ziele: {seite: (position Welt (3,), ausrichtung Welt (3x3))}. Startet bei den gesetzten Armwinkeln.
        Rueckgabe: (14 Armwinkel, max. Positionsfehler [m], max. Drehfehler [rad])."""
        for _ in range(iterationen):
            mujoco.mj_kinematics(self.m, self.d)
            mujoco.mj_comPos(self.m, self.d)
            fp = fr = 0.0
            for s, (p_soll, r_soll) in ziele.items():
                a = self._arm[s]
                p_ist, r_ist = self.handflaeche(s)
                e_p = np.asarray(p_soll) - p_ist
                e_r = drehfehler(r_ist, np.asarray(r_soll))
                fp, fr = max(fp, float(np.linalg.norm(e_p))), max(fr, float(np.linalg.norm(e_r)))
                jacp = np.zeros((3, self.m.nv))
                jacr = np.zeros((3, self.m.nv))
                mujoco.mj_jac(self.m, self.d, jacp, jacr, p_ist, self.handgelenk[s])
                j = np.vstack([jacp[:, a["v"]], GEWICHT_DREHUNG * jacr[:, a["v"]]])
                e = np.concatenate([e_p, GEWICHT_DREHUNG * e_r])
                dq = j.T @ np.linalg.solve(j @ j.T + 1e-3 * np.eye(6), e)
                self.d.qpos[a["q"]] = np.clip(self.d.qpos[a["q"]] + 0.5 * dq, a["lo"], a["hi"])
            if fp < 1e-4 and fr < 1e-3:
                break
        mujoco.mj_kinematics(self.m, self.d)
        fp = fr = 0.0
        for s, (p_soll, r_soll) in ziele.items():
            p_ist, r_ist = self.handflaeche(s)
            fp = max(fp, float(np.linalg.norm(np.asarray(p_soll) - p_ist)))
            fr = max(fr, float(np.linalg.norm(drehfehler(r_ist, np.asarray(r_soll)))))
        return self.arme(), fp, fr
