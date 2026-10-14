"""Zugriff auf die Welten (share/roboter_welten/welten/<name>/).

Eine Welt beschreibt EINE Umgebung an einer Stelle: szene.xml ist die physische Umgebung fuer die
Simulation (Layer 4), orte.yaml die benannten Orte mit Verbindungen im map-Frame (Layer 1 plant darauf).
Missionen verweisen nur per Namen auf die Welt (Schluessel 'welt'), damit dieselben Koordinaten nicht in
jeder Missionsdatei kopiert werden. Fuer die echte Halle kommt hier spaeter die gespeicherte Karte dazu.
"""
import os

import yaml


def _wurzel() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("roboter_welten"), "welten")


def verfuegbare_welten():
    w = _wurzel()
    return sorted(d for d in os.listdir(w) if os.path.isdir(os.path.join(w, d)))


def welt_verzeichnis(name: str) -> str:
    pfad = os.path.join(_wurzel(), name)
    if not os.path.isdir(pfad):
        raise FileNotFoundError(f"Welt {name!r} nicht gefunden ({pfad}); vorhanden: {verfuegbare_welten()} "
                                f"- 'colcon build' ausgefuehrt?")
    return pfad


def szene_pfad(name: str) -> str:
    """MuJoCo-Datei der Umgebung (ohne Roboter)."""
    return os.path.join(welt_verzeichnis(name), "szene.xml")


def lade_orte(name: str) -> dict:
    """Inhalt von orte.yaml: {'orte': {name: {x, y}}, 'verbindungen': [[a, b], ...] (optional)}."""
    with open(os.path.join(welt_verzeichnis(name), "orte.yaml")) as f:
        return yaml.safe_load(f)
