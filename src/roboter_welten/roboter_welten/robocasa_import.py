"""Erzeugt die szene.xml einer Welt aus einer RoboCasa-Kueche (Layout + Style aus aufbau.yaml).

RoboCasa (github.com/robocasa/robocasa; Code MIT, Assets CC BY 4.0) baut eine Kueche erst zur Laufzeit aus
Bauplaenen (Layout-/Style-YAML) und Einzelteilen (MJCF + Meshes) zusammen. Ablauf:
  1. robocasa_export.py (laeuft mit dem Python der RoboCasa-Umgebung) baut die Kueche einmal und speichert die
     komplette Szene als szene_roh.xml (mit RoboCasa-Roboter, Pfade absolut nach ~/tools/robocasa).
  2. Dieses Skript (normales Python, mujoco wie die Simulation):
     - entfernt den RoboCasa-Roboter samt Antrieben, Sensoren, Keyframes und nur von ihm benutzten Assets
     - kopiert die benutzten Meshes/Texturen nach welten/<welt>/robocasa/ (Pfade relativ, Texturen <= 1024 px)
     - setzt die Geom-Gruppen fuer Lidar und Karte (siehe unten)
     - verschiebt/dreht die Kueche so, dass der Startpunkt des Roboters (aufbau.yaml: start) im Ursprung liegt
     - schreibt szene.xml (+ fixtures.json mit den Fixture-Lagen im map-Frame)
Zur Laufzeit (Simulation, Tests) wird RoboCasa nicht gebraucht.

Gruppen (anders als in den handgebauten Welten, wo ein Quader zugleich sichtbar, Kollision und Lidar-Ziel ist):
  group 1  Optik (RoboCasa-Meshes und -Texturen): nur gerendert
  group 3  Kollisionskoerper (RoboCasa: group 0): nicht gerendert; contype 0 / conaffinity 1 = kollidieren mit dem
           Roboter und mit Objekten (contype 1), aber nicht untereinander (Tueren/Schubladen gegen ihren Schrank:
           ihre Bewegung begrenzen die Gelenkanschlaege) - halbiert die Kollisionsrechnung
  group 4  je Kuechenteil ein unsichtbarer Ersatzquader ohne Kollision um seine Kollisionskoerper (Tueren zu):
           Lidar-Ziel (lidar.py tastet 2 und 4 ab) und Kartenhindernis (karte_aus_szene.py). Teile, deren Grundriss in
           dem eines groesseren liegt (Geraet auf der Platte, Spuele in der Insel, Haengeschrank), bekommen keinen.
  Gemessen (Layout 7): die ~1000 Optik-Meshes als Lidar-Ziel kosten ~56 ms je 3D-Scan, die Ersatzquader wenige ms.
  Unsichtbare Geoms (rgba-Alpha 0) ignoriert mj_ray; deshalb eine eigene, nicht gerenderte Gruppe.
  Offene Tueren sieht das Lidar damit nicht (Ersatzquader bilden den geschlossenen Zustand ab).
Die Physik-Optionen von RoboCasa (option, size, visual) entfallen: beim Anhaengen an den Roboter gelten ohnehin dessen
Werte (MjSpec.attach warnt sonst bei jedem Laden).

Aufruf (aus dem Projekt-Root; schreibt in den Quellbaum):
    python3 src/roboter_welten/roboter_welten/robocasa_import.py robocasa_kueche7 [--neu]
    --neu: szene_roh.xml neu exportieren (sonst wird ein vorhandener Export wiederverwendet)
Danach: karte_aus_szene.py <welt>, colcon build. Pfade: ROBOCASA_PYTHON (Standard ~/tools/robocasa_venv/bin/python),
ROBOCASA_EXPORT (Standard ~/tools/robocasa_export).
"""
import json
import math
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

try:
    from roboter_welten.libero_import import verkleinere_textur
except ImportError:                       # Aufruf als Skript aus dem Quellbaum
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from libero_import import verkleinere_textur

ASSET_ORDNER = "robocasa"
# RoboCasa-Roboter (PandaOmron) und die IK-Zielmarker von robosuite (left/right_eef_target)
ROBOTER_PRAEFIXE = ("robot0_", "mobilebase0_", "gripper0_", "fixed_mount0_", "left_eef_target", "right_eef_target")
OPTIK_GRUPPE, KOLLISION_GRUPPE, ERSATZ_GRUPPE = 1, 3, 4
ABSCHNITTE_ROBOTER = ("actuator", "sensor", "tendon", "equality", "contact")
ABSCHNITTE_OPTIONEN = ("option", "size", "visual", "statistic")


def _praefix(name: str) -> bool:
    return bool(name) and name.startswith(ROBOTER_PRAEFIXE)


def roh_export(aufbau: dict, welt: str, neu: bool) -> str:
    ordner = os.path.join(os.path.expanduser(os.environ.get("ROBOCASA_EXPORT", "~/tools/robocasa_export")), welt)
    roh = os.path.join(ordner, "szene_roh.xml")
    if neu or not os.path.exists(roh):
        python = os.path.expanduser(os.environ.get("ROBOCASA_PYTHON", "~/tools/robocasa_venv/bin/python"))
        skript = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robocasa_export.py")
        subprocess.run([python, "-E", skript, str(aufbau["layout"]), str(aufbau["style"]), ordner,
                        str(aufbau.get("seed", 0))], check=True)
    return roh


def entferne_roboter(wurzel: ET.Element) -> int:
    """Koerper mit Roboter-Praefix samt allem, was auf sie verweist. Liefert die Zahl entfernter Elemente."""
    weg = set()
    eltern = {k: e for e in wurzel.iter() for k in e}
    for body in list(wurzel.iter("body")):
        if _praefix(body.get("name", "")) and body in eltern:
            for el in body.iter():
                if el.get("name"):
                    weg.add(el.get("name"))
            eltern[body].remove(body)
    n = len(weg)
    for abschnitt in ABSCHNITTE_ROBOTER:
        for a in wurzel.findall(abschnitt):
            for el in list(a):
                if any(v in weg or _praefix(v) for v in el.attrib.values()):
                    a.remove(el)
                    n += 1
            if len(a) == 0:
                wurzel.remove(a)
    for k in wurzel.findall("keyframe"):     # qpos-Laengen passen ohne Roboter nicht mehr
        wurzel.remove(k)
    return n


def entferne_unbenutzte_assets(wurzel: ET.Element) -> int:
    """Meshes/Materialien/Texturen, auf die nichts mehr verweist (v. a. die des Roboters)."""
    n = 0
    while True:
        benutzt = {(k, v) for el in wurzel.iter() if el.tag not in ("mesh", "material", "texture")
                   for k, v in el.attrib.items() if k in ("mesh", "material", "texture", "hfield")}
        for mat in wurzel.iter("material"):
            if mat.get("texture"):
                benutzt.add(("texture", mat.get("texture")))
        entfernt = 0
        for asset in wurzel.findall("asset"):
            for el in list(asset):
                if el.tag in ("mesh", "material") and (el.tag, el.get("name")) not in benutzt:
                    asset.remove(el); entfernt += 1
                elif el.tag == "texture" and el.get("type") != "skybox" and ("texture", el.get("name")) not in benutzt:
                    asset.remove(el); entfernt += 1
        n += entfernt
        if not entfernt:
            return n


def dedupliziere_assets(wurzel: ET.Element) -> dict:
    """Gleiche Assets zusammenfuehren: RoboCasa legt je Fixture eigene Textur-, Material- und Mesh-Elemente an, auch
    wenn sie dieselbe Datei mit denselben Einstellungen nutzen (Layout 7: 77 grosse Texturen aus 28 Dateien), und
    MuJoCo laedt jedes Element einzeln in den Speicher. Zusammengefuehrt wird nur bei identischen Attributen (ausser
    dem Namen) und Kindelementen; Verweise (material@texture, geom@material, geom@mesh, ...) zeigen danach auf das
    erste Element. Reihenfolge Textur -> Material (wird nach dem Umbiegen gleich) -> Mesh."""
    zaehler = {}
    for tag in ("texture", "material", "mesh"):
        erstes, ersatz = {}, {}
        for asset in wurzel.findall("asset"):
            for el in list(asset):
                if el.tag != tag or el.get("type") == "skybox" or not el.get("name"):
                    continue
                schluessel = (tuple(sorted((k, v) for k, v in el.attrib.items() if k != "name")),
                              tuple(ET.tostring(kind) for kind in el))
                if schluessel in erstes:
                    ersatz[el.get("name")] = erstes[schluessel]
                    asset.remove(el)
                else:
                    erstes[schluessel] = el.get("name")
        if ersatz:
            for el in wurzel.iter():
                v = el.get(tag)
                if el.tag != tag and v in ersatz:
                    el.set(tag, ersatz[v])
        zaehler[tag] = len(ersatz)
    return zaehler


def kopiere_dateien(wurzel: ET.Element, ziel_ordner: str, quellen: dict, max_textur: int) -> int:
    """Dateien der Assets nach <welt>/robocasa/<quelle>/<Pfad> kopieren (Texturen auf max_textur px verkleinert),
    Pfade relativ zur szene.xml setzen."""
    n = 0
    for comp in wurzel.findall("compiler"):
        for k in ("meshdir", "texturedir", "assetdir"):
            comp.attrib.pop(k, None)
    for el in wurzel.iter():
        if el.tag not in ("mesh", "texture", "hfield", "skin"):
            continue
        for k, v in list(el.attrib.items()):
            if not k.startswith("file") or not v:
                continue
            pfad = os.path.abspath(v)
            for kuerzel, basis in quellen.items():
                if pfad.startswith(basis + os.sep):
                    rel = os.path.join(kuerzel, os.path.relpath(pfad, basis))
                    break
            else:
                raise ValueError(f"Datei ausserhalb der bekannten Quellen: {pfad}")
            ziel = os.path.join(ziel_ordner, ASSET_ORDNER, rel)
            if not os.path.exists(ziel):
                os.makedirs(os.path.dirname(ziel), exist_ok=True)
                if not verkleinere_textur(pfad, ziel, max_textur):
                    shutil.copy2(pfad, ziel)
                n += 1
            el.set(k, os.path.join(ASSET_ORDNER, rel).replace(os.sep, "/"))
    return n


def boden_anpassen(spec, boden_praefixe, anheben: float = 0.002) -> int:
    """RoboCasa-Boden: Kollision entfernen (den Boden stellt das Robotermodell, eine Ebene auf z = 0; doppelt hiesse
    doppelte Fusskontakte) und die Optik um 2 mm anheben (sonst flackern beide Boeden auf z = 0 ineinander)."""
    n = 0
    for g in list(spec.geoms):
        koerper = g.parent.name if g.parent is not None else ""
        if not any(koerper.startswith(p) for p in boden_praefixe):
            continue
        if g.contype or g.conaffinity:
            spec.delete(g)
        else:
            pos = np.array(g.pos, dtype=float)
            pos[2] += anheben
            g.pos = pos
        n += 1
    return n


def setze_gruppen(spec) -> dict:
    zaehler = {"optik": 0, "kollision": 0}
    for g in spec.geoms:
        if g.contype == 0 and g.conaffinity == 0:
            g.group = OPTIK_GRUPPE; zaehler["optik"] += 1
        else:
            g.group = KOLLISION_GRUPPE
            g.contype, g.conaffinity = 0, 1
            zaehler["kollision"] += 1
    return zaehler


def ersatzquader(spec, boden_praefixe, toleranz: float = 0.02, park_hoehe: float = 4.0, band_oben: float = 1.9) -> int:
    """Je Kuechenteil (Koerper direkt unter der Welt) ein unsichtbarer Quader (group 4, ohne Kollision und Masse) um
    alle Kollisionskoerper seines Teilbaums in Ruhelage, im Frame des Teils. Liefert die Zahl der Quader.
    Ausgelassen: Geoms ueber park_hoehe (RoboCasa parkt nicht benutzte Sockelteile als 1-cm-Wuerfel auf ~10 m),
    Teile ganz ueber band_oben (Dunstabzug, Oberschrank ueber dem Kuehlschrank: fuer Lidar und Karte ohne Bedeutung)
    und Quader, die bis band_oben vollstaendig in einem anderen liegen (Grundriss UND Hoehe; Lidar-Aufwand je
    Quader ~0.06 ms je 3D-Scan)."""
    import mujoco
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_kinematics(m, d)
    einheit = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], dtype=float)
    lokal = {}
    for g in range(m.ngeom):
        wurzel = m.body_rootid[m.geom_bodyid[g]]
        if m.geom_group[g] != KOLLISION_GRUPPE or wurzel == 0:
            continue
        welt = (einheit * m.geom_aabb[g, 3:] + m.geom_aabb[g, :3]) @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        if welt[:, 2].mean() > park_hoehe:
            continue
        p = (welt - d.xpos[wurzel]) @ d.xmat[wurzel].reshape(3, 3)       # in den Frame des Teils
        lo, hi = lokal.get(wurzel, (np.full(3, np.inf), np.full(3, -np.inf)))
        lokal[wurzel] = (np.minimum(lo, p.min(0)), np.maximum(hi, p.max(0)))
    kandidaten = []
    for wurzel, (lo, hi) in lokal.items():
        name = m.body(wurzel).name
        if any(name.startswith(p) for p in boden_praefixe):
            continue
        welt = (einheit * (hi - lo) / 2 + (hi + lo) / 2) @ d.xmat[wurzel].reshape(3, 3).T + d.xpos[wurzel]
        if welt[:, 2].min() > band_oben:
            continue
        q = (welt[:, 0].min(), welt[:, 1].min(), welt[:, 2].min(),
             welt[:, 0].max(), welt[:, 1].max(), min(welt[:, 2].max(), band_oben))
        kandidaten.append((np.prod(np.subtract(q[3:], q[:3])), name, lo, hi, q))
    behalten = []
    for _, name, lo, hi, q in sorted(kandidaten, key=lambda k: -k[0]):
        if any(all(q[i] >= b[i] - toleranz for i in range(3)) and all(q[i] <= b[i] + toleranz for i in range(3, 6))
               for b in behalten):
            continue
        behalten.append(q)
        geom = spec.body(name).add_geom()
        geom.name = f"{name}_lidar"
        geom.type = mujoco.mjtGeom.mjGEOM_BOX
        geom.pos = (lo + hi) / 2
        geom.size = np.maximum((hi - lo) / 2, 0.005)
        geom.group, geom.contype, geom.conaffinity, geom.density = ERSATZ_GRUPPE, 0, 0, 0.0
        geom.rgba = [0.2, 0.6, 1.0, 0.35]
    return len(behalten)


def erzeuge(welt: str, neu: bool = False, welten_wurzel: str = None) -> str:
    import mujoco
    welten_wurzel = welten_wurzel or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "welten")
    ordner = os.path.abspath(os.path.join(welten_wurzel, welt))
    with open(os.path.join(ordner, "aufbau.yaml")) as f:
        aufbau = yaml.safe_load(f)
    roh = roh_export(aufbau, welt, neu)
    robocasa_assets = os.path.expanduser(aufbau.get("robocasa_assets", "~/tools/robocasa/robocasa/models/assets"))
    quellen = {"robocasa": os.path.abspath(robocasa_assets),
               "robosuite": os.path.abspath(os.path.expanduser(
                   aufbau.get("robosuite_assets", "~/tools/robosuite/robosuite/models/assets")))}

    wurzel = ET.parse(roh).getroot()
    for abschnitt in ABSCHNITTE_OPTIONEN:
        for el in wurzel.findall(abschnitt):
            wurzel.remove(el)
    n_roboter = entferne_roboter(wurzel)
    n_assets = entferne_unbenutzte_assets(wurzel)
    doppelt = dedupliziere_assets(wurzel)
    shutil.rmtree(os.path.join(ordner, ASSET_ORDNER), ignore_errors=True)
    n_dateien = kopiere_dateien(wurzel, ordner, quellen, int(aufbau.get("max_textur", 512)))

    # Kueche so legen, dass aufbau.yaml:start (Kuechenkoordinaten) im Ursprung liegt, Blick +x = gier_grad
    sx, sy = aufbau["start"]["xy"]
    gier = math.radians(aufbau["start"].get("blick_grad", 0.0))
    c, s = math.cos(-gier), math.sin(-gier)
    pos = [-(c * sx - s * sy), -(s * sx + c * sy), 0.0]
    weltkoerper = wurzel.find("worldbody")
    rahmen = ET.Element("frame", {"name": "kueche", "pos": " ".join(f"{v:.6g}" for v in pos),
                                  "euler": f"0 0 {-gier:.9g}"})
    for kind in list(weltkoerper):
        weltkoerper.remove(kind)
        rahmen.append(kind)
    weltkoerper.append(rahmen)

    alt = os.getcwd()
    os.chdir(ordner)
    try:
        spec = mujoco.MjSpec.from_string(ET.tostring(wurzel, encoding="unicode"))
        boden_anpassen(spec, aufbau.get("boden_praefixe", ["floor"]))
        gruppen = setze_gruppen(spec)
        gruppen["ersatzquader"] = ersatzquader(spec, aufbau.get("boden_praefixe", ["floor"]))
        m = spec.compile()
        xml = spec.to_xml()
    finally:
        os.chdir(alt)

    kopf = (f"<!-- ERZEUGT von roboter_welten/robocasa_import.py aus aufbau.yaml (RoboCasa Layout {aufbau['layout']}, "
            f"Style {aufbau['style']}, Seed {aufbau.get('seed', 0)}) - nicht von Hand aendern.\n"
            "     Meshes/Texturen in robocasa/ stammen aus RoboCasa (Code MIT, Assets CC BY 4.0) und robosuite (MIT). -->\n")
    erste, rest = xml.split("\n", 1)
    with open(os.path.join(ordner, "szene.xml"), "w") as f:
        f.write(erste + "\n" + kopf + rest)

    # Fixture-Lagen in den map-Frame umrechnen (fuer Orte, Missionen, Pruefungen)
    roh_fx = os.path.join(os.path.dirname(roh), "fixtures.json")
    if os.path.exists(roh_fx):
        with open(roh_fx) as f:
            info = json.load(f)
        for fx in info["fixtures"].values():
            if fx.get("pos"):
                x, y = fx["pos"][0], fx["pos"][1]
                fx["pos_map"] = [round(c * x - s * y + pos[0], 3), round(s * x + c * y + pos[1], 3), round(fx["pos"][2], 3)]
        with open(os.path.join(ordner, "fixtures.json"), "w") as f:
            json.dump(info, f, indent=1)

    groesse = sum(os.path.getsize(os.path.join(r, n)) for r, _, ns in os.walk(os.path.join(ordner, ASSET_ORDNER))
                  for n in ns)
    print(f"{welt}: szene.xml geschrieben (Layout {aufbau['layout']}, Style {aufbau['style']}): {m.nbody} Koerper, "
          f"{m.ngeom} Geoms, {m.njnt} Gelenke, {m.nmesh} Meshes, {m.ntex} Texturen; Roboter-Elemente entfernt: "
          f"{n_roboter}, unbenutzte Assets: {n_assets}, zusammengefuehrt: {doppelt}; {n_dateien} Dateien, "
          f"{groesse / 1e6:.1f} MB; Texturdaten im Modell {m.ntexdata / 1e6:.0f} MB; Gruppen {gruppen}")
    return os.path.join(ordner, "szene.xml")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        sys.exit("Aufruf: robocasa_import.py <welt> [--neu]")
    erzeuge(args[0], neu="--neu" in sys.argv)
