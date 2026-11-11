"""Belegungskarte (Nav2-Format: karte.pgm + karte.yaml) aus der Szene einer Welt.

In der Simulation ist die Umgebung bekannt; statt sie erst per SLAM abzufahren, wird die Karte direkt aus den
Hindernissen der Szene gerechnet (alle Quader der geom group 2, die das Hoehenband 0,12-1,9 m schneiden - dasselbe
Band, das die Sensorik als Hindernis wertet). Fuer eine echte Halle wird die Karte stattdessen einmal mit SLAM
aufgenommen und gespeichert (ros2 run nav2_map_server map_saver_cli -f welten/<name>/karte).

Kartenursprung = map-Frame = Startpose des Roboters (0, 0, Blick +x), wie die Orte in orte.yaml.

Aufruf (aus dem Projekt-Root, schreibt in den Quellbaum):
    python3 src/roboter_welten/roboter_welten/karte_aus_szene.py [welt ...]     # ohne Namen: alle Welten
"""
import math
import os
import sys

import numpy as np

AUFLOESUNG = 0.05                 # m je Zelle
RAND = 1.0                        # m Rand um die Welt
BAND = (0.12, 1.9)                # Hoehenband der Hindernisse [m], wie roboter_sensorik
FREI, BELEGT = 254, 0             # PGM-Werte (map_server: trinary, negate 0)


def quader(szene_xml: str):
    """(x0, y0, x1, y1) aller Hindernis-Quader (geom group 2, Box, schneidet das Hoehenband) - ohne Roboter."""
    import mujoco
    m = mujoco.MjModel.from_xml_path(szene_xml)
    d = mujoco.MjData(m)
    mujoco.mj_kinematics(m, d)
    boxen = []
    for g in range(m.ngeom):
        if m.geom_group[g] != 2 or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_BOX:
            continue
        R = d.geom_xmat[g].reshape(3, 3)
        halb = np.abs(R) @ m.geom_size[g]             # achsparallele Huelle (auch fuer gedrehte Quader)
        p = d.geom_xpos[g]
        if p[2] + halb[2] < BAND[0] or p[2] - halb[2] > BAND[1]:
            continue
        boxen.append((p[0] - halb[0], p[1] - halb[1], p[0] + halb[0], p[1] + halb[1]))
    return boxen


def rastern(boxen, aufloesung=AUFLOESUNG, rand=RAND, punkte=()):
    """Belegungsraster (Zeile 0 = oben, wie PGM) und Ursprung (x, y der linken unteren Ecke).
    punkte: (x, y), die auf der Karte liegen muessen (Orte der Mission), ohne selbst Hindernis zu sein."""
    alle = list(boxen) + [(x, y, x, y) for x, y in punkte]
    x0 = math.floor((min(b[0] for b in alle) - rand) / aufloesung) * aufloesung
    y0 = math.floor((min(b[1] for b in alle) - rand) / aufloesung) * aufloesung
    x1 = max(b[2] for b in alle) + rand
    y1 = max(b[3] for b in alle) + rand
    breite, hoehe = int(math.ceil((x1 - x0) / aufloesung)), int(math.ceil((y1 - y0) / aufloesung))
    bild = np.full((hoehe, breite), FREI, dtype=np.uint8)
    for bx0, by0, bx1, by1 in boxen:
        c0, c1 = int((bx0 - x0) / aufloesung), int(math.ceil((bx1 - x0) / aufloesung))
        r0, r1 = int((by0 - y0) / aufloesung), int(math.ceil((by1 - y0) / aufloesung))
        bild[hoehe - r1:hoehe - r0, max(c0, 0):c1] = BELEGT
    return bild, (x0, y0)


def schreiben(ordner: str, bild, ursprung, aufloesung=AUFLOESUNG):
    hoehe, breite = bild.shape
    with open(os.path.join(ordner, "karte.pgm"), "wb") as f:
        f.write(f"P5\n# aus szene.xml gerechnet (karte_aus_szene.py)\n{breite} {hoehe}\n255\n".encode())
        f.write(bild.tobytes())
    with open(os.path.join(ordner, "karte.yaml"), "w") as f:
        f.write("# Belegungskarte der Welt, aus szene.xml gerechnet (roboter_welten/karte_aus_szene.py). Nicht von Hand\n"
                "# aendern: nach einer Aenderung der Szene neu erzeugen.\n"
                f"image: karte.pgm\nmode: trinary\nresolution: {aufloesung}\n"
                f"origin: [{ursprung[0]:.3f}, {ursprung[1]:.3f}, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")


def main(namen):
    wurzel = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "welten")
    namen = namen or sorted(n for n in os.listdir(wurzel) if os.path.isdir(os.path.join(wurzel, n)))
    for n in namen:
        ordner = os.path.join(wurzel, n)
        boxen = quader(os.path.join(ordner, "szene.xml"))
        if not boxen:
            print(f"{n}: keine Hindernisse - keine Karte (Navigation ohne Karte)")
            continue
        punkte = []
        orte_yaml = os.path.join(ordner, "orte.yaml")
        if os.path.isfile(orte_yaml):
            import yaml
            with open(orte_yaml) as f:
                punkte = [(float(o["x"]), float(o["y"])) for o in (yaml.safe_load(f) or {}).get("orte", {}).values()]
        bild, ursprung = rastern(boxen, punkte=punkte)
        schreiben(ordner, bild, ursprung)
        print(f"{n}: {bild.shape[1]} x {bild.shape[0]} Zellen, {len(boxen)} Hindernisse, Ursprung {ursprung}")


if __name__ == "__main__":
    main(sys.argv[1:])
