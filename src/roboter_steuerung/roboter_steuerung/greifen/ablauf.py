"""Ablauf des beidhaendigen Greifens und Ablegens einer Kiste (ROS-frei).

Ein Ablauf ist eine Folge von Phasen. Jede Phase hat eine Dauer und Zielwerte fuer die 14 Armgelenke und die 14
Fingergelenke; dazwischen wird weich (Kosinus) interpoliert. Die Armziele kommen aus der Inversen Kinematik
(kinematik.py), einmal beim Planen, fuer die Kistenpose zum Planungszeitpunkt.

Greifen (Kiste liegt vor dem Roboter, Lage aus der Objektpose):
  vorgreifen  Haende offen seitlich neben die Kiste (Abstand ABSTAND_VOR je Seite), Daumen angelegt
  zugreifen   Handflaechen an die Kistenseiten, DRUCK m in die Kiste hinein (Positionsregelung -> Anpresskraft)
  umfassen    Zeige- und Mittelfinger um die Vorderkante krummen
  anheben     beide Haende um HUB nach oben
  halten      Tragepose (bleibt danach stehen, bis abgelegt wird)
Ablegen (Kiste in den Haenden, Ablagepose auf dem Tisch):
  absenken    Kiste auf die Ablagepose (Unterkante auf Tischhoehe)
  loesen      Finger strecken
  oeffnen     Haende seitlich weg (ABSTAND_VOR)
  zurueck     Arme in die Ruhepose
Reihenfolge der 14 Fingergelenke: links Daumen 0-2, Mittelfinger 0-1, Zeigefinger 0-1, dann rechts ebenso
(= Reihenfolge der Handaktuatoren im Modell, roboter.yaml haende.gelenke).
"""
from dataclasses import dataclass, field

import numpy as np

from roboter_steuerung.greifen.kinematik import yaw_matrix

ABSTAND_VOR = 0.05     # Haende vor dem Zugreifen so weit neben der Kiste [m]
# Griff (gemessen 11.11.2026, 0,5 kg): der Halt kommt vom Anpressdruck der Handflaechen (DRUCK) - mit 0,04 rutschte
# die Kiste 1,4 cm in 20 s nach unten, mit 0,06 nicht. Die Finger zeigen laengs der Kistenseite nach vorn und reichen
# ueber die Vorderkante hinaus: stark gekruemmt (1,4) umgreifen sie die Kante und schieben die Kiste 6 cm zum Roboter,
# sie kippt dann um die Achse zwischen den Handflaechen (21-24 Grad). Leicht gekruemmt (0,5, misst wie gestreckt)
# bleibt sie auf 1,5 cm mittig und kippt 5-9 Grad, auch beim Gehen und Drehen.
DRUCK = 0.06          # Soll-Eindringtiefe je Seite beim Zugreifen [m] -> Anpresskraft ueber die Armregelung
HUB = 0.15             # Anheben [m] (gemessener Betriebspunkt: 0,15 m)
# Tragepose: Kistenmitte relativ zum Becken (vor, seitlich, hoch) [m], unabhaengig davon, wo gegriffen wurde (die
# Anfahrt trifft nur auf +-5 cm). Mit der Kiste so nah am Koerper braucht es nur ein Drittel der Taillenneigung:
# gemessen 11.11.2026 (0,5 kg, zuruecktreten, drehen, 5 s gehen mit 0,5 m/s): Anteil 0 -> 14,8 Grad Neigung,
# 0,33 -> 8,6 Grad, 0,67 -> traege (0,5 m statt 2,4 m in 5 s); voll (-0,15 rad) lief der Roboter rueckwaerts davon.
# Hoehe: Soll 0,16 m ueber dem Becken ergibt die Kiste ~0,10 m darueber. Trotz Schwerkraft-Vorhalt bleiben die
# Handflaechen ~6 cm unter dem Soll: der Anpressdruck biegt die Arme nach aussen (Schulter koppelt in die Hoehe),
# und die Taille erreicht unter Last nur -0,01 statt -0,05 rad (gemessen 11.11.2026).
TRAGEPOSE = (0.24, 0.0, 0.16)
# Schwerkraft-Vorhalt (kinematik.schwerkraft_vorhalt): Arm-kp der Steuerung (policy_29dof.yaml kps 15..28) und die
# Phasen, in denen die Hand die Last traegt.
ARM_KP = 40.0
LAST_PHASEN = ("anheben", "tragen", "halten", "vorsetzen", "absenken", "loesen")
GREIF_DX = 0.0       # Handflaechen laengs der Kiste versetzt (+ = zur Vorderseite, weg vom Roboter) [m]
GREIF_DZ = 0.03      # Handflaechen etwas ueber der Kistenmitte: Unterkante der Hand bleibt ueber der Tischplatte

# Fingerposen (rad). Daumen angelegt: in der Ebene der Handflaeche, buendig mit ihrer Innenseite (gesucht am Modell:
# Daumen 0 = 0, Daumen 1 an der Grenze, Daumen 2 gebeugt). Mit Daumen 0 = 0,75 hing er nach unten und setzte auf der
# Tischplatte auf. Links schliessen die Finger mit negativen Winkeln, rechts mit positiven (Gelenkgrenzen im Modell).
HAND_OFFEN = np.array([0.0, 1.05, 0.9, 0.0, 0.0, 0.0, 0.0,
                       0.0, -1.05, -0.9, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
HAND_UMFASSEN = np.array([0.0, 1.05, 0.9, -0.5, -0.5, -0.5, -0.5,
                          0.0, -1.05, -0.9, 0.5, 0.5, 0.5, 0.5], dtype=np.float32)
HAND_KP = 5.0         # Finger-kp beim Greifen (Nm/rad); roboter.yaml haende.kp gilt fuer die offene Hand


def glatt(a: float) -> float:
    a = min(max(a, 0.0), 1.0)
    return 0.5 - 0.5 * np.cos(np.pi * a)


@dataclass
class Phase:
    name: str
    dauer: float
    arme: np.ndarray
    haende: np.ndarray
    taille: float = 0.0             # Anteil 0..1 der Taillenneigung (Gegengewicht), siehe TAILLE_ANTEIL


@dataclass
class Ablauf:
    start_arme: np.ndarray
    start_haende: np.ndarray
    phasen: list = field(default_factory=list)
    ik_fehler: float = 0.0          # groesster Positionsfehler der IK ueber alle Phasen [m]

    @property
    def dauer(self) -> float:
        return float(sum(p.dauer for p in self.phasen))

    start_taille: float = 0.0

    def sollwerte(self, t: float):
        """(arme 14, haende 14, Taillenanteil 0..1, Phasenname, fertig) zur Zeit t seit Beginn des Ablaufs."""
        arme, haende, taille, t0 = self.start_arme, self.start_haende, self.start_taille, 0.0
        for p in self.phasen:
            if t < t0 + p.dauer:
                s = glatt((t - t0) / p.dauer)
                return ((1 - s) * arme + s * p.arme, (1 - s) * haende + s * p.haende,
                        (1 - s) * taille + s * p.taille, p.name, False)
            arme, haende, taille, t0 = p.arme, p.haende, p.taille, t0 + p.dauer
        return arme.copy(), haende.copy(), taille, (self.phasen[-1].name if self.phasen else ""), True

    @property
    def ende(self):
        return self.phasen[-1].arme.copy(), self.phasen[-1].haende.copy()


def _ziele(kiste_pos, kiste_yaw, halbbreite, seitlich, dz=0.0, dx=0.0):
    """Handflaechenziele links (+y der Kiste) und rechts (-y), Ausrichtung = Kisten-Frame.
    seitlich: Abstand der Handflaechen von den Kistenseiten (negativ = hineindruecken), dz nach oben, dx nach vorn
    (Kisten-Frame)."""
    r = yaw_matrix(kiste_yaw)
    c = np.asarray(kiste_pos, dtype=float) + np.array([0.0, 0.0, dz])
    return {"left": (c + r @ np.array([dx, halbbreite + seitlich, 0.0]), r),
            "right": (c + r @ np.array([dx, -(halbbreite + seitlich), 0.0]), r)}


# Taillenneigung je Phase (Anteil von taille_pitch): Gegengewicht nur, wenn Haende weit vorn sind und die Last
# tragen. Fest -0,15 rad liess den Roboter mit leeren, angehobenen Haenden nach hinten kippen und rueckwaerts laufen
# (gemessen 11.11.2026, Sturz nach 5 s); der Betriebspunkt -0,15 rad gilt mit Last vor dem Koerper.
TAILLE_ANTEIL = {"anheben_vor": 0.0, "ueber": 0.3, "vorgreifen": 0.5, "zugreifen": 0.7, "umfassen": 0.7,
                 "anheben": 0.5, "tragen": 0.33, "halten": 0.33, "vorsetzen": 0.5, "absenken": 0.5, "loesen": 0.5,
                 "oeffnen": 0.5, "hochziehen": 0.3, "vor_brust": 0.0, "zurueck": 0.0}

# Anfahrt der Haende von oben: aus der Ruhepose (Haende neben den Oberschenkeln, unter Tischhoehe) erst vor der
# Brust anheben, dann ueber die Kistenseiten, dann absenken. Direkt schwangen die Finger von unten gegen die
# Tischkante und schoben den Roboter zurueck (gemessen: 7 cm, Kiste nicht gegriffen).
ANHEBEN_VOR = (-0.22, 0.10, 0.10)     # (dx, seitlich, dz) vor der Brust
UEBER = 0.08                          # Hoehe ueber der Greifhoehe, bevor abgesenkt wird [m] (Finger ueber der Platte)


def _plane(kin, ab, schritte, kiste_pos, kiste_yaw, halbbreite, ruhe_arme, letzte, taille_basis, taille_pitch,
           last_kg=0.0, arm_kp=ARM_KP):
    """schritte: (name, dauer, dx, seitlich, dz, fingerpose), (name, dauer, "koerper", (vor, seitlich, hoch),
    seitlich, fingerpose) oder (name, dauer, None, armwinkel, fingerpose).
    Die IK rechnet je Phase mit der Taillenneigung dieser Phase (0,15 rad verschieben die Haende um ~6 cm); auf die
    IK-Loesung kommt der Schwerkraft-Vorhalt (mit Last in LAST_PHASEN)."""
    for schritt in schritte:
        anteil = TAILLE_ANTEIL[schritt[0]]
        kin.setze_gelenk("waist_pitch_joint", taille_basis + anteil * taille_pitch)
        if schritt[2] is None:
            name, dauer, _, arme, hand = schritt
            arme = np.asarray(arme, dtype=float)
        elif schritt[2] == "koerper":                # Ziel relativ zum Becken (Tragepose)
            name, dauer, _, (vx, vy, vz), seitlich, hand = schritt
            pb = np.array(kin.d.qpos[0:3], dtype=float)
            w, x, y, z = kin.d.qpos[3:7]
            yaw_b = float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
            ziel = pb + yaw_matrix(yaw_b) @ np.array([vx, vy, vz])
            arme, fp, _ = kin.loese_bester(_ziele(ziel, yaw_b, halbbreite, seitlich), [letzte, ruhe_arme])
            ab.ik_fehler = max(ab.ik_fehler, fp)
        else:
            name, dauer, dx, seitlich, dz, hand = schritt
            arme, fp, _ = kin.loese_bester(_ziele(kiste_pos, kiste_yaw, halbbreite, seitlich, dz, dx),
                                           [letzte, ruhe_arme])
            ab.ik_fehler = max(ab.ik_fehler, fp)
        letzte = arme
        if schritt[2] is not None and arm_kp:
            arme = arme + kin.schwerkraft_vorhalt(arme, arm_kp, last_kg if name in LAST_PHASEN else 0.0)
        ab.phasen.append(Phase(name, dauer, np.asarray(arme, dtype=np.float32), hand.copy(), anteil))
    return ab


def _ab(schritte, ab_phase):
    """Schritte ab der Phase ab_phase (Nachplanen mitten im Ablauf); None = alle."""
    if ab_phase is None:
        return schritte
    namen = [s[0] for s in schritte]
    return schritte[namen.index(ab_phase):]


# Vor diesen Phasen plant der Greif-Dienst den Rest neu (aktuelle Lage von Basis und Kiste): nach der Anfahrt setzt
# sich der Roboter noch, waehrend die Arme nach vorn gehen (gemessen 11.11.2026: 3 cm zurueck, dann beim Anheben
# vor - die Haende griffen 4,5 cm zu weit hinten und rutschten vorn von der Kiste).
# Beim Ablegen vor dem Absenken: Kiste steht ueber der Sollstelle, ihre gemessene Lage korrigiert den Rest-Fehler
# (Durchhang, Rutschen in der Hand), dann geht es nur noch senkrecht nach unten.
NACHPLANEN = {"greifen": ("vorgreifen", "zugreifen"), "ablegen": ("absenken",)}


def plane_greifen(kin, start_arme, start_haende, kiste_pos, kiste_yaw, halbbreite, ruhe_arme, taille_basis,
                  taille_pitch, start_taille=0.0, ab_phase=None, last_kg=0.0, arm_kp=ARM_KP) -> Ablauf:
    """kin: Armkinematik mit gesetztem Zustand (Basis + Gelenke). taille_basis: Taillen-Pitch ohne Neigung
    (Ruhepose), taille_pitch: volle Neigung (z. B. -0,15 rad). Rueckgabe: Ablauf bis zur Tragepose (Phase halten).
    ab_phase: nur die Phasen ab dieser (Nachplanen)."""
    ab = Ablauf(np.asarray(start_arme, dtype=np.float32), np.asarray(start_haende, dtype=np.float32))
    dxa, sa, dza = ANHEBEN_VOR
    schritte = [("anheben_vor", 2.0, dxa, sa, dza, HAND_OFFEN),
                ("ueber", 2.0, GREIF_DX, ABSTAND_VOR, UEBER, HAND_OFFEN),
                ("vorgreifen", 1.5, GREIF_DX, ABSTAND_VOR, GREIF_DZ, HAND_OFFEN),
                ("zugreifen", 1.5, GREIF_DX, -DRUCK, GREIF_DZ, HAND_OFFEN),
                ("umfassen", 1.0, GREIF_DX, -DRUCK, GREIF_DZ, HAND_UMFASSEN),
                ("anheben", 2.0, GREIF_DX, -DRUCK, GREIF_DZ + HUB, HAND_UMFASSEN),
                ("tragen", 1.5, "koerper", TRAGEPOSE, -DRUCK, HAND_UMFASSEN),
                ("halten", 1.0, "koerper", TRAGEPOSE, -DRUCK, HAND_UMFASSEN)]
    schritte = _ab(schritte, ab_phase)
    ab.start_taille = start_taille
    return _plane(kin, ab, schritte, kiste_pos, kiste_yaw, halbbreite, ruhe_arme, np.asarray(start_arme, float),
                  taille_basis, taille_pitch, last_kg, arm_kp)


# Beim Ablegen die Kiste so weit "in" die Tischplatte fahren [m]: die Platte haelt sie auf, die Arme geben nach.
# Ohne das hing die Kiste durch den Rest-Durchhang 2 cm ueber der Platte und fiel beim Oeffnen (gemessen 11.11.2026).
EINDRUECKEN = 0.03
# Ueber der Sollstelle zuerst so hoch [m] (Kistenunterkante ueber der Platte), dann senkrecht absenken. Gleichzeitig
# vor und abwaerts setzte die Kiste vorzeitig auf, die Reibung hielt sie, und sie rutschte 4,5 cm in den Haenden.
UEBER_ABLAGE = 0.04


def plane_ablegen(kin, start_arme, start_haende, ablage_pos, ablage_yaw, halbbreite, ruhe_arme, taille_basis,
                  taille_pitch, start_taille=1.0, last_kg=0.0, arm_kp=ARM_KP, griff_versatz=None,
                  ab_phase=None) -> Ablauf:
    """ablage_pos: Soll-Mitte der Kiste auf dem Tisch (Welt). ruhe_arme: Armwinkel der Ruhepose (14).
    griff_versatz: gemessene Lage der Handflaechenmitte relativ zur Kistenmitte (Welt, 3). Die Kiste rutscht beim
    Tragen in den Haenden (gemessen: 4,6 cm nach hinten bei einer Drehung mit Seitschritt) - geplant wird deshalb fuer
    die Kiste, nicht fuer die Haende. None = Nennwert (Griff wie geplant).
    Die Haende ziehen nach dem Loesen erst nach oben und vor die Brust, bevor sie in die Ruhepose gehen."""
    ab = Ablauf(np.asarray(start_arme, dtype=np.float32), np.asarray(start_haende, dtype=np.float32))
    dxa, sa, dza = ANHEBEN_VOR
    v = np.array([0.0, 0.0, GREIF_DZ]) if griff_versatz is None else np.asarray(griff_versatz, dtype=float)
    basis = np.asarray(ablage_pos, dtype=float) + v       # Handflaechenmitte, wenn die Kiste auf der Sollstelle steht
    schritte = [("vorsetzen", 2.0, 0.0, -DRUCK, UEBER_ABLAGE, HAND_UMFASSEN),
                ("absenken", 1.5, 0.0, -DRUCK, -EINDRUECKEN, HAND_UMFASSEN),
                ("loesen", 1.0, 0.0, -DRUCK, -EINDRUECKEN, HAND_OFFEN),
                ("oeffnen", 1.5, 0.0, ABSTAND_VOR, 0.0, HAND_OFFEN),
                ("hochziehen", 1.5, 0.0, ABSTAND_VOR, UEBER, HAND_OFFEN),
                ("vor_brust", 1.5, dxa, sa, dza - v[2], HAND_OFFEN),
                ("zurueck", 2.5, None, ruhe_arme, HAND_OFFEN)]
    schritte = _ab(schritte, ab_phase)
    ab.start_taille = start_taille
    return _plane(kin, ab, schritte, basis, ablage_yaw, halbbreite, ruhe_arme, np.asarray(start_arme, float),
                  taille_basis, taille_pitch, last_kg, arm_kp)
