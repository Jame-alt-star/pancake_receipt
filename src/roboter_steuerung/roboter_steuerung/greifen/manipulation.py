"""Mischung von Policy und gefuehrten Armen (Zustand Manipulieren), ROS-frei.

Gemessener Betriebspunkt (docs/statusupdate_g1_projekt.md, Abschnitt "Greifen im Stehen", und
docs/plan_greifen_phase1.md, Messung "Tragen"):
  - Die Policy sieht die Armgelenke in Ruhepose und mit Geschwindigkeit 0 ("blind fuer die Arme"). Sieht sie die
    gefuehrten Arme, faellt der Roboter schon bei 0,30 m Reichweite: die Armwinkel liegen ausserhalb ihres Trainings.
    Teilweises Maskieren ist nicht robust und wird deshalb nicht angeboten.
  - Die Armziele ersetzen die Policy-Ausgabe fuer die Armgelenke.
  - Die Taille neigt sich proportional zur Armauslenkung nach hinten (Pitch -0,15 rad bei voller Greifpose) als
    Gegengewicht zur Last vor dem Koerper.
Gelenkindizes in MuJoCo-Reihenfolge (MUJOCO_JOINT_ORDER): 14 = waist_pitch, 15..21 linker Arm, 22..28 rechter Arm.
"""
import numpy as np

ARM_INDIZES = np.arange(15, 29)
TAILLE_PITCH_INDEX = 14


def maskiere_arme(joint_pos: np.ndarray, joint_vel: np.ndarray, ruhepose: np.ndarray):
    """Beobachtung fuer die Policy: Armgelenke auf Ruhepose, Armgeschwindigkeit 0 (neue Arrays)."""
    q = np.array(joint_pos, dtype=np.float32, copy=True)
    dq = np.array(joint_vel, dtype=np.float32, copy=True)
    q[ARM_INDIZES] = ruhepose[ARM_INDIZES]
    dq[ARM_INDIZES] = 0.0
    return q, dq


def auslenkung(arm_ziele: np.ndarray, ruhepose: np.ndarray, voll_rad: float) -> float:
    """Anteil 0..1 der Armauslenkung: Abstand der Armziele zur Ruhepose relativ zu voll_rad (Norm ueber 14 Gelenke)."""
    a = float(np.linalg.norm(np.asarray(arm_ziele) - ruhepose[ARM_INDIZES]))
    return float(np.clip(a / max(voll_rad, 1e-6), 0.0, 1.0))


def mische(policy_ziele: np.ndarray, arm_ziele: np.ndarray, gewicht: float, taille_pitch: float,
           anteil_taille: float) -> np.ndarray:
    """Ziele an die Gelenke: Armgelenke = gewicht*arm_ziele + (1-gewicht)*Policy (gewicht 1 = voll gefuehrt; Werte
    dazwischen nur zum weichen Ein-/Ausblenden), Taille Pitch + anteil_taille*taille_pitch."""
    z = np.array(policy_ziele, dtype=np.float32, copy=True)
    g = float(np.clip(gewicht, 0.0, 1.0))
    z[ARM_INDIZES] = g * np.asarray(arm_ziele, dtype=np.float32) + (1.0 - g) * z[ARM_INDIZES]
    z[TAILLE_PITCH_INDEX] += float(np.clip(anteil_taille, 0.0, 1.0)) * taille_pitch
    return z
