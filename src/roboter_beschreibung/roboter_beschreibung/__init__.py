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
    if not os.path.exists(r["modell_xml"]):
        raise FileNotFoundError(f"Robotermodell nicht gefunden: {r['modell_xml']} - 'colcon build' ausgefuehrt?")
    return r
