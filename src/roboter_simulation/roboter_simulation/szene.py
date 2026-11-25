"""Setzt das MuJoCo-Modell aus Roboter, Welt und Kamera zusammen.

Der Roboter (g1_29dof.xml, Unitree-Modell mit Boden und Licht) bleibt unveraendert im
Herstellerordner; die Umgebung kommt aus roboter_welten (szene.xml ohne Roboter). Beides wird ueber
MjSpec verbunden, statt die Welt per <include> neben das Robotermodell zu legen: so liegen Welten
nicht mehr zwischen den Unitree-Dateien, und dieselbe Welt passt zu jedem Robotermodell.
Geprueft 14.10.2026: fuer testwelt, testwelt3d und demo_lager entsteht dasselbe Modell wie aus den
frueheren scene_*.xml (gleiche Geoms mit Position, Groesse, Gruppe, Kontakt und Farbe, gleiche Massen).

Die RGBD-Kamera wird auf demselben Weg angehaengt (Montage aus roboter_beschreibung, Eintrag kamera; kamera.py),
statt sie in die Unitree-Datei zu schreiben.
"""
from typing import Optional

import mujoco


def lade_modell(roboter_xml: str, welt_szene: Optional[str] = None, kamera: Optional[dict] = None) -> mujoco.MjModel:
    """roboter_xml: Pfad zum Robotermodell; welt_szene: Pfad zu einer szene.xml oder None (nur Roboter);
    kamera: Montage aus roboter.yaml (Eintrag kamera) oder None (ohne Kamera)."""
    if not welt_szene and not kamera:
        return mujoco.MjModel.from_xml_path(roboter_xml)
    roboter = mujoco.MjSpec.from_file(roboter_xml)
    if welt_szene:
        welt = mujoco.MjSpec.from_file(welt_szene)
        # prefix "" behaelt die Geom-Namen der Welt bei (z. B. "hindernis").
        roboter.attach(welt, frame=roboter.worldbody.add_frame(), prefix="")
    if kamera:
        from roboter_simulation.kamera import fuege_kamera_hinzu
        fuege_kamera_hinzu(roboter, kamera)
    modell = roboter.compile()
    del roboter
    _speicher_zurueckgeben()
    return modell


def _speicher_zurueckgeben():
    """Beim Kompilieren grosser Welten (robocasa_kueche7: ~2700 Geoms, ~200 Meshes) entsteht viel Zwischenspeicher, den
    glibc nach dem Freigeben behaelt (gemessen: RSS 1024 MB, nach malloc_trim 686 MB). WSL2 hat ~3,5 GB fuer die ganze
    Pipeline; Auslagern liess dort Nav2 sekundenlang stehen. Ohne glibc (anderes System) passiert nichts."""
    try:
        import ctypes
        import gc
        gc.collect()
        ctypes.CDLL("libc.so.6").malloc_trim(0)
    except (OSError, AttributeError):
        pass


def lade_welt(roboter_xml: str, welt: str = "", kamera: Optional[dict] = None) -> mujoco.MjModel:
    """Wie lade_modell, aber mit dem Namen einer Welt aus roboter_welten ('' = nur Roboter)."""
    if not welt:
        return lade_modell(roboter_xml, kamera=kamera)
    from roboter_welten import szene_pfad
    return lade_modell(roboter_xml, szene_pfad(welt), kamera)
