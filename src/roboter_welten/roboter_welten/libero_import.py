"""Erzeugt die szene.xml einer Welt aus einem Raum (raum.xml) und LIBERO-Fixtures/-Objekten (aufbau.yaml).

LIBERO (github.com/Lifelong-Robot-Learning/LIBERO, MIT-Lizenz) liefert Fixtures mit Gelenken (Schubladenschrank,
Mikrowelle, Herd) und Objekte (Schuessel, Teller, Becher, ...) als MJCF mit Meshes und Texturen. LIBERO selbst
laeuft ueber robosuite; hier wird nur gelesen: jedes Teil wird mit MjSpec an den Raum gehaengt, die benoetigten
Mesh-/Texturdateien werden nach welten/<welt>/libero/ kopiert (Pfade relativ), und das Ergebnis ist eine normale
szene.xml wie bei den anderen Welten. Zur Laufzeit (Simulation, Tests) wird LIBERO nicht gebraucht.

Anpassungen beim Import (robosuite erledigt sie in LIBERO stillschweigend, MuJoCo direkt nicht):
  - compiler/inertiagrouprange entfernt (Mikrowelle: "4 4" schliesst alle Geoms aus -> Tuer ohne Masse)
  - doppelte Asset-Namen umbenannt (Herd: zwei Texturen 'tex-stove_knob'); Verweise zeigen auf die jeweils letzte
  - reine Optik-Geoms (contype = conaffinity = 0) ohne Masse: Masse und Traegheit nur aus den Kollisions-Geoms
  - bewegliche Objekte bekommen ein freies Gelenk (LIBERO: free, damping 0.0005)
  - Texturen ueber 1024 px werden verkleinert (LIBERO bis 4096 px; braucht Pillow, sonst unveraendert)
  - Kollisions-Geoms von group 0 nach group 3: kollidieren weiter, werden aber nicht mitgerendert (LIBERO-Kollision
    sind viele halbtransparente Quader, die sonst im Kamerabild erscheinen). Optik bleibt group 1.
LIBERO-Teile haben damit nichts in group 2: fuer Lidar und Karte unsichtbar (sie stehen auf Platte und Tisch).

Aufruf (aus dem Projekt-Root, schreibt in den Quellbaum; danach karte_aus_szene.py und colcon build):
    python3 src/roboter_welten/roboter_welten/libero_import.py [welt]        # Standard: libero_kueche
LIBERO-Pfad: Umgebungsvariable LIBERO_PFAD, sonst quelle_standard aus aufbau.yaml (~/tools/LIBERO).
"""
import math
import os
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ASSET_ORDNER = "libero"           # Zielordner der Meshes/Texturen in der Welt (relativ zu szene.xml)
DATEI_ELEMENTE = ("mesh", "texture", "hfield", "skin")
OPTIK_DICHTE = 0.0                # reine Optik-Geoms tragen keine Masse bei
MAX_TEXTUR = 1024                 # px, groessere Texturen werden beim Kopieren verkleinert
KOLLISION_GRUPPE = 3              # LIBERO-Kollisionsquader: in Viewer/Kamera ausgeblendet (Standard: Gruppen 0-2)


def libero_wurzel(aufbau: dict) -> str:
    pfad = os.environ.get("LIBERO_PFAD") or aufbau.get("quelle_standard", "~/tools/LIBERO")
    pfad = os.path.expanduser(pfad)
    assets = os.path.join(pfad, "libero", "libero", "assets")
    if not os.path.isdir(assets):
        raise FileNotFoundError(f"LIBERO-Assets nicht gefunden: {assets} (LIBERO_PFAD setzen oder nach "
                                f"~/tools/LIBERO klonen: git clone https://github.com/Lifelong-Robot-Learning/LIBERO)")
    return assets


def bereinigtes_xml(asset_xml: str) -> str:
    """LIBERO-XML mit absoluten Dateipfaden, ohne inertiagrouprange/size und mit eindeutigen Asset-Namen."""
    ordner = os.path.dirname(asset_xml)
    wurzel = ET.parse(asset_xml).getroot()
    meshdir = texturedir = ""
    for comp in wurzel.findall("compiler"):
        meshdir = comp.attrib.pop("meshdir", meshdir)
        texturedir = comp.attrib.pop("texturedir", texturedir)
        comp.attrib.pop("inertiagrouprange", None)
        comp.attrib.pop("assetdir", None)
    for s in wurzel.findall("size"):
        wurzel.remove(s)
    for asset in wurzel.findall("asset"):
        aktuell, gesehen = {}, set()       # Verweisname -> zuletzt definierter (eindeutiger) Name
        for el in list(asset):
            name = el.get("name")
            if name is not None:
                neu, i = name, 2
                while (el.tag, neu) in gesehen:
                    neu, i = f"{name}_{i}", i + 1
                gesehen.add((el.tag, neu))
                el.set("name", neu)
                aktuell[(el.tag, name)] = neu
            if el.tag == "material" and el.get("texture"):
                el.set("texture", aktuell.get(("texture", el.get("texture")), el.get("texture")))
            if el.tag in DATEI_ELEMENTE and el.get("file"):
                basis = texturedir if el.tag == "texture" else meshdir
                el.set("file", os.path.normpath(os.path.join(ordner, basis, el.get("file"))))
    return ET.tostring(wurzel, encoding="unicode")


def verkleinere_textur(quelle: str, ziel: str, max_px: int = None) -> bool:
    """Bild mit mehr als max_px (Standard MAX_TEXTUR) Pixeln Kantenlaenge verkleinert speichern; False = unveraendert
    kopieren. LIBERO-Texturen haben bis 4096 x 4096 (ungepackt ~50 MB Arbeitsspeicher je Textur); die Kamera rendert
    320 x 240."""
    max_px = max_px or MAX_TEXTUR
    if not quelle.lower().endswith((".png", ".jpg", ".jpeg")):
        return False
    try:
        from PIL import Image
    except ImportError:
        print(f"  Hinweis: Pillow fehlt, Textur unverkleinert: {os.path.basename(quelle)}")
        return False
    with Image.open(quelle) as bild:
        if max(bild.size) <= max_px:
            return False
        f = max_px / max(bild.size)
        bild.resize((round(bild.width * f), round(bild.height * f)), Image.LANCZOS).save(ziel, optimize=True)
    return True


def kopiere_dateien(spec, assets: str, welt_ordner: str) -> int:
    """Mesh/Textur-Dateien nach <welt>/libero/<Pfad wie in LIBERO> kopieren; Pfade im Spec relativ setzen."""
    n = 0
    for el in list(spec.meshes) + list(spec.textures) + list(spec.hfields) + list(spec.skins):
        quelle = getattr(el, "file", "")
        if not quelle:
            continue
        rel = os.path.relpath(quelle, assets)
        if rel.startswith(".."):
            raise ValueError(f"Datei ausserhalb der LIBERO-Assets: {quelle}")
        ziel = os.path.join(welt_ordner, ASSET_ORDNER, rel)
        if not os.path.exists(ziel):
            os.makedirs(os.path.dirname(ziel), exist_ok=True)
            if not verkleinere_textur(quelle, ziel):
                shutil.copy2(quelle, ziel)
            n += 1
        el.file = os.path.join(ASSET_ORDNER, rel).replace(os.sep, "/")
    return n


def huellbox(spec):
    """(min, max) aller Geoms im Koordinatensystem des Teils (vor Drehung/Verschiebung)."""
    import mujoco
    m = spec.copy().compile()
    d = mujoco.MjData(m)
    mujoco.mj_kinematics(m, d)
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for g in range(m.ngeom):
        R = d.geom_xmat[g].reshape(3, 3)
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mid = m.geom_dataid[g]
            v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            p = v @ R.T + d.geom_xpos[g]
        else:
            r = np.abs(R) @ m.geom_size[g]
            p = np.array([d.geom_xpos[g] - r, d.geom_xpos[g] + r])
        lo, hi = np.minimum(lo, p.min(0)), np.maximum(hi, p.max(0))
    return lo, hi


def drehung(kipp_x_grad: float, gier_grad: float):
    """Rotationsmatrix und Quaternion (w, x, y, z): erst um x kippen, dann um z drehen."""
    a, b = math.radians(kipp_x_grad), math.radians(gier_grad)
    qx = np.array([math.cos(a / 2), math.sin(a / 2), 0, 0])
    qz = np.array([math.cos(b / 2), 0, 0, math.sin(b / 2)])
    w1, x1, y1, z1 = qz
    w2, x2, y2, z2 = qx
    q = np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                  w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])
    Rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    Rz = np.array([[math.cos(b), -math.sin(b), 0], [math.sin(b), math.cos(b), 0], [0, 0, 1]])
    return Rz @ Rx, q


def lade_teil(eintrag: dict, assets: str, welt_ordner: str):
    """LIBERO-Teil als MjSpec (bereinigt, Dateien kopiert) und die Pose fuer den Rahmen im Raum."""
    import mujoco
    xml = bereinigtes_xml(os.path.join(assets, eintrag["asset"]))
    spec = mujoco.MjSpec.from_string(xml)
    for g in spec.geoms:
        if g.contype == 0 and g.conaffinity == 0:
            g.density = OPTIK_DICHTE
        else:
            g.group = KOLLISION_GRUPPE
    kopiert = kopiere_dateien(spec, assets, welt_ordner)
    if eintrag.get("beweglich", False):
        koerper = spec.worldbody.first_body()
        gelenk = koerper.add_freejoint(name="frei")
        daempfung = np.array(gelenk.damping, dtype=float)      # MuJoCo >= 3.12: Vektor, [0] = lineare Daempfung
        daempfung[0] = 0.0005
        gelenk.damping = daempfung
    alt = os.getcwd()
    os.chdir(welt_ordner)             # Dateipfade sind jetzt relativ zum Weltordner
    try:
        lo, hi = huellbox(spec)
    finally:
        os.chdir(alt)
    R, q = drehung(eintrag.get("kipp_x_grad", 0.0), eintrag.get("gier_grad", 0.0))
    ecken = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]) @ R.T
    mitte = (ecken.min(0) + ecken.max(0)) / 2
    pos = np.array([eintrag["xy"][0] - mitte[0], eintrag["xy"][1] - mitte[1],
                    eintrag["auf"] - ecken.min(0)[2] + 0.001])
    groesse = ecken.max(0) - ecken.min(0)
    return spec, pos, q, groesse, kopiert


def erzeuge(welt: str = "libero_kueche", welten_wurzel: str = None) -> str:
    import mujoco
    welten_wurzel = welten_wurzel or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "welten")
    ordner = os.path.abspath(os.path.join(welten_wurzel, welt))
    with open(os.path.join(ordner, "aufbau.yaml")) as f:
        aufbau = yaml.safe_load(f)
    assets = libero_wurzel(aufbau)
    shutil.rmtree(os.path.join(ordner, ASSET_ORDNER), ignore_errors=True)

    raum = mujoco.MjSpec.from_file(os.path.join(ordner, "raum.xml"))
    zeilen = []
    for art in ("fixtures", "objekte"):
        for e in aufbau.get(art) or []:
            teil, pos, q, groesse, kopiert = lade_teil(e, assets, ordner)
            rahmen = raum.worldbody.add_frame(pos=pos.tolist(), quat=q.tolist())
            raum.attach(teil, frame=rahmen, prefix=f"{e['name']}_")
            zeilen.append(f"{e['name']:<20} {e['asset']:<62} {np.round(groesse, 3)} m, {kopiert} Dateien")

    alt = os.getcwd()
    os.chdir(ordner)
    try:
        raum.compile()                # prueft das Ergebnis, bevor szene.xml ueberschrieben wird
        xml = raum.to_xml()
    finally:
        os.chdir(alt)
    kopf = ("<!-- ERZEUGT von roboter_welten/libero_import.py aus raum.xml + aufbau.yaml - nicht von Hand aendern.\n"
            "     Meshes/Texturen in libero/ stammen aus LIBERO (MIT-Lizenz, Lifelong-Robot-Learning/LIBERO). -->\n")
    erste, rest = xml.split("\n", 1)
    with open(os.path.join(ordner, "szene.xml"), "w") as f:
        f.write(erste + "\n" + kopf + rest)
    groesse = sum(os.path.getsize(os.path.join(r, n)) for r, _, ns in os.walk(os.path.join(ordner, ASSET_ORDNER))
                  for n in ns)
    print(f"{welt}: szene.xml geschrieben, {len(zeilen)} LIBERO-Teile, libero/ {groesse / 1e6:.1f} MB")
    for z in zeilen:
        print("  " + z)
    return os.path.join(ordner, "szene.xml")


if __name__ == "__main__":
    erzeuge(*(sys.argv[1:2] or ["libero_kueche"]))
