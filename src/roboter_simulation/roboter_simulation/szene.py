"""Setzt das MuJoCo-Modell aus Roboter und Welt zusammen.

Der Roboter (g1_29dof.xml, Unitree-Modell mit Boden und Licht) bleibt unveraendert im
Herstellerordner; die Umgebung kommt aus roboter_welten (szene.xml ohne Roboter). Beides wird ueber
MjSpec verbunden, statt die Welt per <include> neben das Robotermodell zu legen: so liegen Welten
nicht mehr zwischen den Unitree-Dateien, und dieselbe Welt passt zu jedem Robotermodell.
Geprueft 15.10.2026: fuer testwelt, testwelt3d und demo_lager entsteht dasselbe Modell wie aus den
frueheren scene_*.xml (gleiche Geoms mit Position, Groesse, Gruppe, Kontakt und Farbe, gleiche Massen).
"""
from typing import Optional

import mujoco


def lade_modell(roboter_xml: str, welt_szene: Optional[str] = None) -> mujoco.MjModel:
    """roboter_xml: Pfad zum Robotermodell; welt_szene: Pfad zu einer szene.xml oder None (nur Roboter)."""
    if not welt_szene:
        return mujoco.MjModel.from_xml_path(roboter_xml)
    roboter = mujoco.MjSpec.from_file(roboter_xml)
    welt = mujoco.MjSpec.from_file(welt_szene)
    # prefix "" behaelt die Geom-Namen der Welt bei (z. B. "hindernis").
    roboter.attach(welt, frame=roboter.worldbody.add_frame(), prefix="")
    return roboter.compile()


def lade_welt(roboter_xml: str, welt: str = "") -> mujoco.MjModel:
    """Wie lade_modell, aber mit dem Namen einer Welt aus roboter_welten ('' = nur Roboter)."""
    if not welt:
        return lade_modell(roboter_xml)
    from roboter_welten import szene_pfad
    return lade_modell(roboter_xml, szene_pfad(welt))
