#!/usr/bin/env python3
"""Erzeugt das Robotermodell mit Dex3-Haenden aus unserem Koerpermodell.

Warum ein erzeugtes Modell statt Unitrees g1_29dof_with_hand.xml: Die Lauf-Policy ist auf unser Koerpermodell
abgestimmt (g1_29dof.xml mit armature="0.01", siehe config/roboter.yaml). Unitrees Hand-Variante hat dieselben 29
Koerpergelenke, aber keine armature und eine andere Aktuator-Reihenfolge (Handgelenke zwischen den Armen). Hier wird
deshalb nur die Hand uebernommen:
  - Handgelenk-Glied (wrist_yaw_link): Masse/Traegheit der Unitree-Hand-Variante (Handflaeche eingerechnet), die
    Gummihand-Attrappe entfaellt, die Geometrie der Handflaeche kommt dazu;
  - je Hand drei Finger (Daumen 3 Gelenke, Mittel- und Zeigefinger je 2), Netze und Grenzen von Unitree;
  - 14 Aktuatoren werden HINTER den 29 Koerperaktuatoren angehaengt (links, dann rechts; je Hand Daumen 0-2,
    Mittelfinger 0-1, Zeigefinger 0-1). Die ersten 29 Aktuatoren bleiben unveraendert (Policy, Steuerung).
  - Fingergelenke bekommen armature (FINGER_ARMATURE): die Glieder wiegen nur 20-90 g, ohne Rotortraegheit wird die
    PD-Regelung bei 2 ms Physiktakt instabil.

Aufruf (aus dem Projekt-Root):  python3 scripts/erzeuge_handmodell.py [--pruefen]
  --pruefen  nur vergleichen, ob die eingecheckte Datei aktuell ist (Exit 1 sonst); so prueft es auch der Test.
"""
import argparse
import copy
import os
import sys
import xml.etree.ElementTree as ET

ORDNER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "roboter_beschreibung", "modelle",
                      "g1_description")
KOERPER = "g1_29dof.xml"
UNITREE_HAND = "g1_29dof_with_hand.xml"
ZIEL = "g1_29dof_mit_haenden.xml"
FINGER_ARMATURE = "0.01"
SEITEN = ("left", "right")
FINGER = ("thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1")
HANDGELENKE = [f"{s}_hand_{f}_joint" for s in SEITEN for f in FINGER]


def _koerper(wurzel, name):
    for b in wurzel.find("worldbody").iter("body"):
        if b.get("name") == name:
            return b
    raise KeyError(name)


def erzeuge() -> str:
    koerper = ET.parse(os.path.join(ORDNER, KOERPER)).getroot()
    hand = ET.parse(os.path.join(ORDNER, UNITREE_HAND)).getroot()
    koerper.set("model", "g1_29dof_mit_haenden")

    # Reibung fuer das Greifen: elliptischer Reibungskegel mit impratio 10. Mit MuJoCos Standard (pyramidal,
    # impratio 1) rutschte die Kiste trotz ~9 N Anpresskraft je Seite langsam durch (Haftreibung ist dort weich);
    # damit haelt sie (gemessen 11.11.2026). Stehen und Gehen der Lauf-Policy bleiben unveraendert (geprueft).
    option = ET.Element("option", {"cone": "elliptic", "impratio": "10"})
    koerper.insert(list(koerper).index(koerper.find("compiler")) + 1, option)

    # Netze der Hand (erstes <asset>, dort stehen die Netze); die Gummihand-Attrappe entfaellt ganz
    asset = koerper.find("asset")
    for m in list(asset.iter("mesh")):
        if "rubber" in m.get("name"):
            asset.remove(m)
    vorhanden = {m.get("name") for m in asset.iter("mesh")}
    for m in hand.find("asset").iter("mesh"):
        if "hand" in m.get("name") and "rubber" not in m.get("name") and m.get("name") not in vorhanden:
            asset.append(copy.deepcopy(m))

    for s in SEITEN:
        ziel = _koerper(koerper, f"{s}_wrist_yaw_link")
        quelle = _koerper(hand, f"{s}_wrist_yaw_link")
        ziel.remove(ziel.find("inertial"))
        ziel.insert(0, copy.deepcopy(quelle.find("inertial")))
        for g in list(ziel.findall("geom")):
            if "rubber" in (g.get("mesh") or ""):
                ziel.remove(g)
        for g in quelle.findall("geom"):
            if "hand" in (g.get("mesh") or ""):
                ziel.append(copy.deepcopy(g))
        for b in quelle.findall("body"):
            neu = copy.deepcopy(b)
            for j in neu.iter("joint"):
                j.set("armature", FINGER_ARMATURE)
            ziel.append(neu)

    aktuatoren = koerper.find("actuator")
    for j in HANDGELENKE:
        ET.SubElement(aktuatoren, "motor", {"name": j, "joint": j})

    ET.indent(koerper, space="  ")
    kopf = (f"<!-- ERZEUGT von scripts/erzeuge_handmodell.py aus {KOERPER} + Haenden aus {UNITREE_HAND}. "
            f"Nicht von Hand aendern. -->\n")
    return kopf + ET.tostring(koerper, encoding="unicode") + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pruefen", action="store_true")
    a = ap.parse_args()
    text = erzeuge()
    pfad = os.path.join(ORDNER, ZIEL)
    if a.pruefen:
        aktuell = os.path.exists(pfad) and open(pfad, encoding="utf-8").read() == text
        print("aktuell" if aktuell else f"{ZIEL} ist veraltet - scripts/erzeuge_handmodell.py ausfuehren")
        sys.exit(0 if aktuell else 1)
    with open(pfad, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"geschrieben: {pfad}")


if __name__ == "__main__":
    main()
