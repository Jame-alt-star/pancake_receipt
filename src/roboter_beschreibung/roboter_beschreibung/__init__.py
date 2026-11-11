"""Zugriff auf Robotermodell und Geometrie (config/roboter.yaml) fuer alle Schichten."""
import os

import yaml


def share() -> str:
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory("roboter_beschreibung")


def lade(wurzel: str = None) -> dict:
    """Inhalt von config/roboter.yaml; modell_xml als absoluter Pfad.

    wurzel: Paketverzeichnis (Quellbaum fuer Tests ohne ROS); Standard: Share-Verzeichnis.
    """
    w = wurzel or share()
    with open(os.path.join(w, "config", "roboter.yaml")) as f:
        r = yaml.safe_load(f)
    r["modell_xml"] = os.path.join(w, r["modell_xml"])
    if r.get("modell_xml_haende"):
        r["modell_xml_haende"] = os.path.join(w, r["modell_xml_haende"])
    if not os.path.exists(r["modell_xml"]):
        raise FileNotFoundError(f"Robotermodell nicht gefunden: {r['modell_xml']} - 'colcon build' ausgefuehrt?")
    return r


def gelenkgrenzen(modell_xml: str):
    """Untere/obere Grenze je Aktuator [rad], in der Aktuator-Reihenfolge des Modells (= Reihenfolge der Zielwinkel).

    Direkt aus der MJCF-Datei gelesen, bewusst ohne MuJoCo: die Steuerung braucht keine Physik-Engine, und das
    Kompilieren des Modells mit allen Netzen belegte ~410 MB Arbeitsspeicher. Mit Viewer unter WSL2 (2,8 GB) lagerte
    das System dadurch aus, die Steuerung stockte, und der Roboter fiel beim Anhalten (21.10.2026). Gelesen wird nur
    range direkt am Gelenk; <default>-Klassen und <include> werden abgelehnt statt still falsch gelesen. Der Test
    vergleicht mit dem kompilierten Modell.
    """
    import xml.etree.ElementTree as ET
    import numpy as np

    wurzel = ET.parse(modell_xml).getroot()
    if wurzel.find(".//include") is not None or wurzel.find(".//default//joint[@range]") is not None:
        raise ValueError(f"{modell_xml}: <include> oder Gelenkgrenzen in <default> werden nicht unterstuetzt")
    winkel = "degree"                                  # MJCF-Voreinstellung
    for c in wurzel.iter("compiler"):
        winkel = c.get("angle", winkel)
    gelenke = {j.get("name"): j for j in wurzel.find("worldbody").iter("joint")}
    unten, oben = [], []
    for aktuator in wurzel.find("actuator"):
        j = gelenke[aktuator.get("joint")]
        if j.get("range") is None or j.get("limited") == "false":
            u, o = -np.inf, np.inf
        else:
            u, o = (float(v) for v in j.get("range").split())
            if winkel == "degree":
                u, o = np.radians(u), np.radians(o)
        unten.append(u)
        oben.append(o)
    return np.array(unten, dtype=np.float32), np.array(oben, dtype=np.float32)
