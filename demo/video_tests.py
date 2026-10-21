#!/usr/bin/env python3
"""Testvideo: rendert die Aufzeichnungen aller Systemtests zu EINEM Video.

Aufzeichnen:  bash tests/alle_tests.sh --mit-slam --video /tmp/testvideo     (jeder Test schreibt pose.npz,
              ereignisse.json, lauf.json in einen eigenen Unterordner, siehe tests/system/testaufnahme.py)
Rendern:      MUJOCO_GL=glfw python3 demo/video_tests.py /tmp/testvideo demo/ausgabe/testserie.mp4
Standbild:    ... --probe <kapitel> <sekunde>   (PNG statt Video, zum Pruefen der Darstellung)

Je Test ein Kapitel: Titelkarte (was geprueft wird, Konfiguration, Ergebnis), dann der Lauf aus der Pose der
Simulation mit dem aktuellen Testschritt und den letzten Pruefergebnissen; bei Missionen zusaetzlich der Plan, bei
Welten die Karte (feste bzw. aufgenommene Karte, 3D-Lidar-Hindernisse, Fahrspur). Zeitraffer; nach NOT-AUS, Stopp
und Ausfaellen einige Sekunden Echtzeit. Bausteine wie im System und in render_demo.py.
"""
import ast, json, math, os, subprocess, sys, time
import cv2, mujoco, numpy as np, yaml
from PIL import Image, ImageDraw, ImageFont

os.environ.setdefault("MUJOCO_GL", "glfw")
W, H, FPS = 1280, 720, 25
ZIEL_S = 30.0           # angestrebte Laenge eines Laufs im Video [s] (ohne Echtzeit-Abschnitte)
ZEITLUPE_S = 6.0        # nach einem Schluesselmoment so lange (Simzeit) in Echtzeit
TITEL_S, ENDE_S, INTRO_S = 4.0, 2.0, 7.0
SCHLUESSEL = ("NOT-AUS", "Stopp", "Steuerung beendet", "Sensorausfall")
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")
for paket in ("roboter_simulation", "roboter_beschreibung", "roboter_welten"):
    sys.path.insert(0, os.path.join(SRC, paket))
from roboter_beschreibung.geometrie import yaw_aus_quat  # noqa: E402
from roboter_simulation.lidar import Lidar  # noqa: E402
from roboter_simulation.szene import lade_modell  # noqa: E402

ROBOTER = yaml.safe_load(open(os.path.join(SRC, "roboter_beschreibung", "config", "roboter.yaml")))
ROBOTER_XML = os.path.join(SRC, "roboter_beschreibung", ROBOTER["modell_xml"])
BAND = (0.12, 1.9)      # Hoehenband der Hinderniswolke [m], wie roboter_sensorik

# Was jeder Test zeigt (Titel aus lauf.json -> Zeilen der Titelkarte)
BESCHREIBUNG = {
    "Pipeline (Layer 3 + 4)": [
        "Steuerung + Simulation ohne Navigation: Freigabe, Stehen, Positionshaltung,",
        "Gehen mit Kurshaltung, Begrenzung, NOT-AUS, Hand-Vorrang, Stopp,",
        "zum Schluss faellt die Steuerung aus -> die Simulation daempft."],
    "Pipeline realistisch + Beinodometrie": [
        "Wie die Pipeline, aber mit Sensorrauschen, Latenz und Gyro-Drift;",
        "die Position kommt aus der Beinodometrie statt aus der Simulation."],
    "Watchdog und Sensorausfall": [
        "Ein einzelner Fahrbefehl darf den Roboter nicht dauerhaft laufen lassen.",
        "Danach faellt die Sensorik aus -> die Steuerung meldet Stoerung."],
    "Mission mit Testdouble": [
        "Layer 1 plant die Mission (PDDL, Fast Downward); ein einfacher Ersatz fuer",
        "Nav2 faehrt geradeaus zu den Orten. Danach NOT-AUS mitten in der Mission."],
    "Mission Testwelt (Nav2)": [
        "Nav2 mit 3D-Lidar, ohne Karte: um das Hindernis herum und durch die Tuer.",
        "Danach NOT-AUS mitten in der Mission."],
    "Mission Patrouille": [
        "Orte a und b besuchen, in b balancieren, zurueck zum Start (Nav2, 3D-Lidar)."],
    "Mission 3D-Palette": [
        "Eine niedrige Palette (0,45 m), die eine einzelne Lidar-Ebene nicht sieht:",
        "das 3D-Lidar erkennt sie, Nav2 faehrt drum herum."],
    "Mission auf fester Karte (AMCL)": [
        "Karte der Welt liegt vorher vor; AMCL lokalisiert den Roboter per Lidar auf ihr.",
        "Die Reihenfolge der Ziele rechnet mit echten Weglaengen vom Nav2-Planer."],
    "Mission mit SLAM": [
        "Keine Karte: slam_toolbox baut sie waehrend der Fahrt.",
        "Realistische Sensoren und Beinodometrie."],
    "Karte aufnehmen (1/2): Erkundung mit SLAM": [
        "Wie in echt, Schritt 1: Welt einmal abfahren, SLAM baut die Karte,",
        "map_saver speichert sie. Realistische Sensoren und Beinodometrie."],
    "Karte aufnehmen (2/2): Mission auf der aufgenommenen Karte (AMCL)": [
        "Wie in echt, Schritt 2: Neustart ohne SLAM. Nur die gespeicherte Karte,",
        "AMCL lokalisiert per Lidar, die Mission laeuft darauf."],
}
SCHRITT_TXT = {"gehe_zu": "Fahre zu", "balanciere": "Balanciere bei", "warte": "Warte bei"}


def font(sz, bold=False):
    return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf" % ("-Bold" if bold else ""), sz)


F_GROSS, F_TITEL, F_FETT, F_MITTEL, F_KLEIN = font(40, True), font(26, True), font(18, True), font(16), font(14)
DUNKEL = (14, 18, 26, 215)
GRUEN, ROT, GELB, GRAU, WEISS = (110, 220, 130, 255), (255, 95, 85, 255), (255, 200, 60, 255), (170, 170, 170, 255), \
    (255, 255, 255, 255)


def umbrechen(dr, text, schrift, breite):
    """Zeilen, die in breite Pixel passen (Woerter bleiben ganz)."""
    zeilen, zeile = [], ""
    for wort in text.split():
        probe = (zeile + " " + wort).strip()
        if zeile and dr.textlength(probe, font=schrift) > breite:
            zeilen.append(zeile)
            zeile = wort
        else:
            zeile = probe
    return zeilen + [zeile] if zeile else zeilen


def kurz(dr, text, schrift, breite):
    """Text auf breite Pixel kuerzen (mit Auslassungszeichen)."""
    if dr.textlength(text, font=schrift) <= breite:
        return text
    while text and dr.textlength(text + "…", font=schrift) > breite:
        text = text[:-1]
    return text + "…"


class Kapitel:
    def __init__(self, ordner, nr, anzahl):
        self.ordner, self.nr, self.anzahl = ordner, nr, anzahl
        self.lauf = json.load(open(os.path.join(ordner, "lauf.json")))
        self.ev = json.load(open(os.path.join(ordner, "ereignisse.json")))
        pose = np.load(os.path.join(ordner, "pose.npz"))
        self.wall, self.t, self.qpos = pose["wall"], pose["t"], pose["qpos"]
        welt = self.lauf.get("welt")
        if not welt and self.lauf.get("mission"):
            welt = yaml.safe_load(open(os.path.join(SRC, "roboter_task_planning", "missions",
                                                    self.lauf["mission"] + ".yaml"))).get("welt")
        self.welt = welt
        welt_dir = os.path.join(SRC, "roboter_welten", "welten", welt) if welt else None
        self.m = lade_modell(ROBOTER_XML, os.path.join(welt_dir, "szene.xml") if welt_dir else None)
        self.d = mujoco.MjData(self.m)
        self._welt_einfaerben()
        self.m.vis.global_.offwidth, self.m.vis.global_.offheight = W, H
        self.boxen = [(p[0] - s[0], p[1] - s[1], p[0] + s[0], p[1] + s[1]) for p, s, g, typ in
                      zip(self.m.geom_pos, self.m.geom_size, self.m.geom_group, self.m.geom_type)
                      if g == 2 and typ == mujoco.mjtGeom.mjGEOM_BOX]
        self.lidar = Lidar(self.m, ROBOTER["lidar"], 360, 10.0, ringe=24) if self.boxen else None
        orte = yaml.safe_load(open(os.path.join(welt_dir, "orte.yaml")))["orte"] if welt_dir else {}
        self.orte = {k: (v["x"], v["y"]) for k, v in orte.items()}
        for e in self.ev:
            e["t"] = self.simzeit(e["wall"])
        self.schritte = [e for e in self.ev if e["art"] == "schritt"]
        self.checks = [e for e in self.ev if e["art"] == "check"]
        self.mission = [e for e in self.ev if e["art"] == "mission"]
        self.momente = [e for e in self.ev if e["art"] == "moment"
                        or (e["art"] == "schritt" and any(k in e["text"] for k in SCHLUESSEL))]
        self._zeitplan()
        self._karte_vorbereiten()
        self.renderer = mujoco.Renderer(self.m, H, W, max_geom=4000)
        self.cam = mujoco.MjvCamera()
        self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.opt = mujoco.MjvOption()
        self.opt.geomgroup[3:] = 0
        self.look = self.yaw_s = None

    def simzeit(self, wall):
        i = int(np.clip(np.searchsorted(self.wall, wall), 0, len(self.wall) - 1))
        return float(self.t[i])

    def _welt_einfaerben(self):
        """Waende halbtransparent (Verfolgerkamera), wie render_demo.py."""
        m = self.m
        for g in range(m.ngeom):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
            if name == "floor":
                m.geom_matid[g] = -1
                m.geom_rgba[g] = [0.30, 0.34, 0.38, 1.0]
            if name.startswith(("wand_", "trenn_")):
                m.geom_matid[g] = -1
                m.geom_rgba[g] = [0.80, 0.82, 0.86, 0.22]
            elif name.startswith(("stapel_", "regal_", "schrank", "hindernis", "palette")):
                base = m.mat_rgba[m.geom_matid[g]].copy() if m.geom_matid[g] >= 0 else m.geom_rgba[g].copy()
                m.geom_matid[g] = -1
                m.geom_rgba[g] = [base[0], base[1], base[2], 0.62]

    def _zeitplan(self):
        """Simzeit je Videobild: Zeitraffer, nach Schluesselmomenten Echtzeit."""
        t0, t1 = float(self.t[0]), float(self.t[-1])
        self.schnell = max(2, int(math.ceil((t1 - t0) / ZIEL_S)))
        lupe = [(e["t"], e["t"] + ZEITLUPE_S) for e in self.momente]
        self.zeiten, self.tempo = [], []
        tc = t0
        while tc <= t1:
            f = 1 if any(a <= tc < b for a, b in lupe) else self.schnell
            self.zeiten.append(tc)
            self.tempo.append(f)
            tc += f / FPS

    # ---------------------------------------------------------------- Karte
    def _karte_vorbereiten(self):
        self.hat_karte = bool(self.boxen or self.orte)
        if not self.hat_karte:
            return
        xs = [b[0] for b in self.boxen] + [b[2] for b in self.boxen] + [p[0] for p in self.orte.values()]
        ys = [b[1] for b in self.boxen] + [b[3] for b in self.boxen] + [p[1] for p in self.orte.values()]
        xs += [float(self.qpos[:, 0].min()), float(self.qpos[:, 0].max())]
        ys += [float(self.qpos[:, 1].min()), float(self.qpos[:, 1].max())]
        self.X0, self.Y0 = min(xs) - 0.6, min(ys) - 0.6
        X1, Y1 = max(xs) + 0.6, max(ys) + 0.6
        self.MAPW = 400
        self.SC = self.MAPW / (X1 - self.X0)
        self.MAPH = min(int(round((Y1 - self.Y0) * self.SC)), 380)
        self.SC = min(self.SC, self.MAPH / (Y1 - self.Y0))
        self.seen = np.zeros((self.MAPH, self.MAPW), dtype=np.uint8)
        self.spur = []
        self.hintergrund, self.karten_titel = None, "Leitstand: 3D-Lidar-Hindernisse + Fahrspur"
        karte = self.lauf.get("karte")
        pfad = None
        if karte == "welt" and self.welt:
            pfad = os.path.join(SRC, "roboter_welten", "welten", self.welt, "karte.yaml")
            self.karten_titel = "Feste Karte der Welt + Lidar + Fahrspur"
        elif karte:
            kandidat = os.path.join(self.ordner, os.path.basename(karte))
            pfad = kandidat if os.path.exists(kandidat) else karte
            self.karten_titel = "Aufgenommene Karte (Phase 1) + Lidar + Fahrspur"
        elif self.lauf.get("slam"):
            self.karten_titel = "SLAM: Karte entsteht beim Fahren (Lidar) + Fahrspur"
        if pfad and os.path.exists(pfad):
            self.hintergrund = self._karte_rastern(pfad)

    def _karte_rastern(self, yaml_pfad):
        """Nav2-Karte (PGM) in das Pixelraster der Leitstand-Karte umrechnen."""
        k = yaml.safe_load(open(yaml_pfad))
        bild = cv2.imread(os.path.join(os.path.dirname(yaml_pfad), k["image"]), cv2.IMREAD_GRAYSCALE)
        res, (ox, oy) = float(k["resolution"]), k["origin"][:2]
        px, py = np.meshgrid(np.arange(self.MAPW), np.arange(self.MAPH))
        x = self.X0 + (px + 0.5) / self.SC
        y = self.Y0 + (self.MAPH - py - 0.5) / self.SC
        spalte = ((x - ox) / res).astype(int)
        zeile = (bild.shape[0] - 1 - ((y - oy) / res)).astype(int)
        drin = (spalte >= 0) & (spalte < bild.shape[1]) & (zeile >= 0) & (zeile < bild.shape[0])
        wert = np.full(px.shape, 205, dtype=np.uint8)
        wert[drin] = bild[zeile[drin], spalte[drin]]
        farbe = np.zeros((self.MAPH, self.MAPW, 3), dtype=np.uint8)
        farbe[:] = (24, 28, 34)                         # unbekannt
        farbe[wert > 230] = (46, 54, 64)                # frei
        farbe[wert < 100] = (150, 160, 175)             # belegt
        return farbe

    def to_px(self, x, y):
        return int((x - self.X0) * self.SC), int(self.MAPH - (y - self.Y0) * self.SC)

    def _karte_zeichnen(self, pos, yw, welt_pts):
        if welt_pts is not None:
            for p in welt_pts:
                u, v = self.to_px(p[0], p[1])
                if 0 <= u < self.MAPW and 0 <= v < self.MAPH:
                    self.seen[v, u] = 255
        self.spur.append((pos[0], pos[1]))
        mp = self.hintergrund.copy() if self.hintergrund is not None else \
            np.full((self.MAPH, self.MAPW, 3), (28, 32, 38), dtype=np.uint8)
        mp[self.seen > 0] = (240, 240, 245)
        if self.hintergrund is None:
            for x0, y0, x1, y1 in self.boxen:           # Umriss der Welt, nur schwach (Orientierung)
                cv2.rectangle(mp, self.to_px(x0, y1), self.to_px(x1, y0), (70, 75, 85), 1)
        for k, (ox, oy) in self.orte.items():
            cv2.circle(mp, self.to_px(ox, oy), 6, (90, 200, 120) if k == "start" else (80, 170, 255), 2)
        if len(self.spur) > 2:
            pts = np.array([self.to_px(x, y) for x, y in self.spur[::3]], dtype=np.int32)
            cv2.polylines(mp, [pts], False, (255, 190, 60), 2)
        rc = self.to_px(pos[0], pos[1])
        cv2.circle(mp, rc, 7, (60, 230, 90), -1)
        cv2.line(mp, rc, (int(rc[0] + 16 * math.cos(yw)), int(rc[1] - 16 * math.sin(yw))), (255, 255, 255), 3)
        cv2.rectangle(mp, (0, 0), (self.MAPW - 1, self.MAPH - 1), (200, 200, 200), 1)
        return mp

    # ---------------------------------------------------------------- Bild
    def _szene(self, q, erstes):
        d = self.d
        d.qpos[:] = q
        mujoco.mj_forward(self.m, d)
        welt_pts = None
        if self.lidar is not None:
            pts = self.lidar.wolke(d)
            ursprung, R = self.lidar.pose(d)
            w = ursprung + pts.astype(float) @ R.T
            welt_pts = w[(w[:, 2] >= BAND[0]) & (w[:, 2] <= BAND[1])]
        pos, yw = q[0:3], yaw_aus_quat(q[3:7])
        if self.yaw_s is None or erstes:
            self.yaw_s, self.look = yw, pos.copy()
        self.yaw_s += 0.06 * ((yw - self.yaw_s + math.pi) % (2 * math.pi) - math.pi)
        self.look = self.look + 0.15 * (pos - self.look)
        cam = self.cam
        if self.boxen:
            cam.lookat[:] = [self.look[0], self.look[1], 0.8]
            cam.distance, cam.elevation = 5.6, -50
        else:                                            # Roboter allein: naeher, flacher (Beine sichtbar); die
            cam.lookat[:] = [self.look[0], self.look[1], 0.2 + 0.5 * self.look[2]]   # Zielhoehe folgt dem Koerper
            cam.distance, cam.elevation = 3.4, -20
        cam.azimuth = math.degrees(self.yaw_s) + 180.0 + 35.0
        self.renderer.update_scene(d, camera=cam, scene_option=self.opt)
        sc = self.renderer.scene
        if welt_pts is not None:
            nah = welt_pts[np.linalg.norm(welt_pts[:, :2] - pos[:2], axis=1) < 6.0]
            for p in nah[::3]:
                if sc.ngeom >= sc.maxgeom - 2:
                    break
                a = float(np.clip((p[2] - BAND[0]) / 1.2, 0.0, 1.0))
                mujoco.mjv_initGeom(sc.geoms[sc.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.025, 0, 0]), p,
                                    np.eye(3).flatten(),
                                    np.array([0.25 + 0.75 * a, 0.85 - 0.55 * a, 0.25 - 0.15 * a, 0.95], np.float32))
                sc.ngeom += 1
        return self.renderer.render().copy(), pos, yw, welt_pts

    def _plan(self, tc):
        plan, idx, ende = None, -1, None
        for e in self.mission:
            if e["t"] > tc:
                break
            if e.get("phase") == "plan":
                plan = e["plan"] if not isinstance(e["plan"], str) else ast.literal_eval(e["plan"])
                idx, ende = -1, None
            elif e.get("phase") == "schritt":
                idx = int(e.get("index", idx))
            elif e.get("phase") == "ende":
                ende = e.get("status")
        return plan, idx, ende

    def bild(self, k, erstes=False):
        tc = self.zeiten[k]
        i = int(np.clip(np.searchsorted(self.t, tc), 0, len(self.t) - 1))
        img, pos, yw, welt_pts = self._szene(self.qpos[i], erstes)
        pil = Image.fromarray(img)
        dr = ImageDraw.Draw(pil, "RGBA")
        # Kopfzeile
        dr.rectangle([0, 0, W, 76], fill=DUNKEL)
        dr.text((20, 8), f"Test {self.nr}/{self.anzahl}: {self.lauf['titel']}", font=F_TITEL, fill=WEISS)
        dr.text((20, 46), self.konfiguration(), font=F_KLEIN, fill=(170, 200, 235, 255))
        # Karte rechts
        if self.hat_karte:
            mp = self._karte_zeichnen(pos, yw, welt_pts)
            MX, MY = W - self.MAPW - 16, 90
            arr = np.array(pil)
            arr[MY:MY + self.MAPH, MX:MX + self.MAPW] = mp
            pil = Image.fromarray(arr)
            dr = ImageDraw.Draw(pil, "RGBA")
            for name, (ox, oy) in self.orte.items():
                u, v = self.to_px(ox, oy)
                dr.text((MX + u + 9, MY + v - 8), name.upper() if len(name) <= 2 else name, font=F_KLEIN, fill=WEISS)
            dr.rounded_rectangle([MX, MY + self.MAPH + 4, MX + self.MAPW, MY + self.MAPH + 28], radius=7, fill=DUNKEL)
            dr.text((MX + 9, MY + self.MAPH + 7), self.karten_titel, font=F_KLEIN, fill=WEISS)
        self._infobox(dr, tc)
        # Moment-Banner
        for e in self.momente:
            if e["art"] == "moment" and e["t"] <= tc < e["t"] + 3.0:
                farbe = ROT if "NOT-AUS" in e["text"] else (GELB if e["text"] == "Stopp" else (255, 150, 60, 255))
                breite = dr.textlength(e["text"], font=F_GROSS)
                dr.rounded_rectangle([W / 2 - breite / 2 - 20, 92, W / 2 + breite / 2 + 20, 150], radius=12, fill=DUNKEL)
                dr.text((W / 2 - breite / 2, 98), e["text"], font=F_GROSS, fill=farbe)
                if "NOT-AUS" in e["text"]:
                    dr.rectangle([0, 0, W - 1, H - 1], outline=(255, 70, 60, 255), width=10)
        # Zeit und Tempo
        tempo = "Echtzeit" if self.tempo[k] == 1 else f"Zeitraffer {self.tempo[k]}x"
        dr.rounded_rectangle([W - 300, H - 42, W - 16, H - 14], radius=8, fill=DUNKEL)
        dr.text((W - 290, H - 38), f"Simzeit {tc - self.t[0]:6.1f} s  ·  {tempo}", font=F_KLEIN,
                fill=GELB if self.tempo[k] == 1 else (230, 230, 230, 255))
        return np.array(pil)

    def _infobox(self, dr, tc):
        schritt = next((e["text"] for e in reversed(self.schritte) if e["t"] <= tc), "Start, Simulation laeuft an")
        checks = [e for e in self.checks if e["t"] <= tc][-5:]
        plan, idx, ende = self._plan(tc)
        BX, BW = 14, 470
        zeilen_schritt = umbrechen(dr, schritt, F_FETT, BW - 24)[:2]
        hoehe = 30 + 20 * len(zeilen_schritt) + 26 + 20 * max(1, len(checks))
        if plan:
            hoehe += 30 + 20 * len(plan[:6])
        BY = H - 14 - hoehe
        dr.rounded_rectangle([BX, BY, BX + BW, H - 14], radius=10, fill=DUNKEL)
        x, y = BX + 12, BY + 8
        dr.text((x, y), "Testschritt", font=F_KLEIN, fill=GRAU)
        y += 20
        for z in zeilen_schritt:
            dr.text((x, y), z, font=F_FETT, fill=WEISS)
            y += 20
        y += 6
        dr.text((x, y), "Pruefungen", font=F_KLEIN, fill=GRAU)
        y += 20
        if not checks:
            dr.text((x, y), "-", font=F_KLEIN, fill=GRAU)
        for e in checks:
            dr.text((x, y), ("✓ " if e["ok"] else "✗ ") + kurz(dr, e["name"], F_KLEIN, BW - 44), font=F_KLEIN,
                    fill=GRUEN if e["ok"] else ROT)
            y += 20
        if plan:
            y += 8
            status = {"erfolg": "Mission erfolgreich", "abgebrochen": "Mission abgebrochen"}.get(ende, "Mission laeuft")
            dr.text((x, y), f"Plan (Layer 1) - {status}", font=F_KLEIN,
                    fill=GRUEN if ende == "erfolg" else (ROT if ende else GELB))
            y += 22
            for j, st in enumerate(plan[:6]):
                if ende == "erfolg" or j < idx:
                    mark, col = "[x]", GRUEN
                elif j == idx and not ende:
                    mark, col = "[>]", GELB
                else:
                    mark, col = "[ ]", GRAU
                dr.text((x, y), f"{mark} {self._schritt_text(st)}", font=F_KLEIN, fill=col)
                y += 20

    @staticmethod
    def _schritt_text(s):
        name = s.split("(")[0]
        arg = [a.strip() for a in s.split("(")[1].rstrip(")").split(",")] if "(" in s else [""]
        ort = arg[-1].upper() if len(arg[-1]) <= 2 else arg[-1]
        return f"{SCHRITT_TXT.get(name, name)} {ort}"

    def konfiguration(self):
        teile = [f"Welt: {self.welt}" if self.welt else "Ebene ohne Hindernisse"]
        nav = self.lauf.get("navigation")
        if nav == "nav2":
            teile.append("Nav2" + (" + SLAM" if self.lauf.get("slam") else "")
                         + (" + Karte/AMCL" if self.lauf.get("karte") else ""))
        elif nav == "testdouble":
            teile.append("Navigations-Testdouble")
        if self.lauf.get("mission"):
            teile.append(f"Mission {self.lauf['mission']}")
        teile.append("realistische Sensoren" if self.lauf.get("realistisch") else "ideale Sensoren")
        teile.append("Beinodometrie" if self.lauf.get("beinodometrie") else "Odometrie aus der Simulation")
        return "  ·  ".join(teile)

    def ergebnis_text(self):
        return ergebnis_text(self.lauf)

    def titelkarte(self):
        """Erstes Bild abgedunkelt mit Titel, Beschreibung, Konfiguration und Ergebnis."""
        basis = (self._szene(self.qpos[0], True)[0] * 0.3).astype(np.uint8)
        pil = Image.fromarray(basis)
        dr = ImageDraw.Draw(pil, "RGBA")
        dr.rounded_rectangle([50, 120, W - 50, 520], radius=18, fill=(10, 13, 20, 225))
        dr.text((80, 150), f"Test {self.nr} von {self.anzahl}", font=F_TITEL, fill=GRAU)
        y = 190
        for z in umbrechen(dr, self.lauf["titel"], F_GROSS, W - 180):
            dr.text((80, y), z, font=F_GROSS, fill=WEISS)
            y += 50
        y += 16
        for z in BESCHREIBUNG.get(self.lauf["titel"], []):
            dr.text((80, y), z, font=F_MITTEL, fill=(220, 225, 235, 255))
            y += 26
        y += 14
        dr.text((80, y), self.konfiguration(), font=F_KLEIN, fill=(170, 200, 235, 255))
        text, farbe = self.ergebnis_text()
        dr.text((80, y + 44), "Ergebnis: " + text, font=F_TITEL, fill=farbe)
        return np.array(pil)

    def schlussbild(self, letztes):
        pil = Image.fromarray(letztes)
        dr = ImageDraw.Draw(pil, "RGBA")
        text, farbe = self.ergebnis_text()
        breite = dr.textlength(text, font=F_GROSS)
        dr.rounded_rectangle([W / 2 - breite / 2 - 24, H / 2 - 40, W / 2 + breite / 2 + 24, H / 2 + 30], radius=14,
                             fill=DUNKEL)
        dr.text((W / 2 - breite / 2, H / 2 - 32), text, font=F_GROSS, fill=farbe)
        return np.array(pil)


def ergebnis_text(lauf):
    b, g = lauf.get("bestanden"), lauf.get("gesamt")
    return f"{b}/{g} Pruefungen bestanden", (GRUEN if b == g else ROT)


def intro(pfade, ordner):
    pil = Image.new("RGB", (W, H), (16, 20, 28))
    dr = ImageDraw.Draw(pil, "RGBA")
    dr.text((70, 50), "Humanoider Roboter G1 - alle Systemtests", font=F_GROSS, fill=WEISS)
    dr.text((70, 102), "Simulation (MuJoCo, ROS 2 Jazzy): Planung (PDDL) - Navigation (Nav2, 3D-Lidar, SLAM/AMCL) - "
                       "Gehen (RL-Policy)", font=F_MITTEL, fill=(170, 200, 235, 255))
    y = 160
    for nr, pfad in enumerate(pfade, 1):
        lauf = json.load(open(os.path.join(pfad, "lauf.json")))
        text, farbe = ergebnis_text(lauf)
        dr.text((70, y), f"{nr:2d}. {lauf['titel']}", font=F_MITTEL, fill=WEISS)
        dr.text((820, y), text, font=F_MITTEL, fill=farbe)
        y += 30
    serie = os.path.join(ordner, "ergebnis.txt")
    if os.path.exists(serie):
        y += 16
        dr.text((70, y), "Komplette Testserie (tests/alle_tests.sh):", font=F_FETT, fill=GRAU)
        y += 26
        zeilen = [z.rstrip() for z in open(serie) if z.strip()]
        for j, z in enumerate(zeilen):
            dr.text((70 + 560 * (j % 2), y + 22 * (j // 2)), z, font=F_KLEIN,
                    fill=GRUEN if z.endswith("OK") else ROT)
    dr.text((70, H - 50), "Zeitraffer; nach NOT-AUS, Stopp und Ausfaellen einige Sekunden Echtzeit.", font=F_KLEIN,
            fill=GRAU)
    return np.array(pil)


def main():
    ordner, ausgabe = sys.argv[1], sys.argv[2]
    pfade = sorted(w for w, _, dateien in os.walk(ordner) if "lauf.json" in dateien and "pose.npz" in dateien)
    if "--probe" in sys.argv:
        k, sek = int(sys.argv[sys.argv.index("--probe") + 1]), float(sys.argv[sys.argv.index("--probe") + 2])
        kap = Kapitel(pfade[k - 1], k, len(pfade))
        ziel = min(int(sek * FPS), len(kap.zeiten) - 1)
        for j in range(0, ziel, max(1, ziel // 60)):       # Karte und Spur grob aufbauen
            kap.bild(j, erstes=(j == 0))
        Image.fromarray(kap.bild(ziel)).save(ausgabe)
        Image.fromarray(kap.titelkarte()).save(ausgabe.replace(".png", "_titel.png"))
        print("Standbild:", ausgabe)
        return
    print(f"{len(pfade)} Kapitel: " + ", ".join(os.path.relpath(p, ordner) for p in pfade), flush=True)
    ffmpeg = subprocess.check_output([sys.executable, "-c", "import imageio_ffmpeg;print(imageio_ffmpeg.get_ffmpeg_exe())"],
                                     text=True).strip()
    pipe = subprocess.Popen([ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                             "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", "22",
                             "-pix_fmt", "yuv420p", "-movflags", "+faststart", ausgabe], stdin=subprocess.PIPE)
    t0, n = time.time(), 0
    bild = intro(pfade, ordner).tobytes()
    for _ in range(int(INTRO_S * FPS)):
        pipe.stdin.write(bild)
    for nr, pfad in enumerate(pfade, 1):
        # Kapitel nacheinander laden: jedes Modell mit Netzen belegt ~400 MB, alle zugleich passten nicht in die WSL
        kap = Kapitel(pfad, nr, len(pfade))
        print(f"Kapitel {nr}: {kap.lauf['titel']} - {len(kap.zeiten)} Bilder, Zeitraffer {kap.schnell}x", flush=True)
        bild = kap.titelkarte().tobytes()
        for _ in range(int(TITEL_S * FPS)):
            pipe.stdin.write(bild)
        letztes = None
        for k in range(len(kap.zeiten)):
            letztes = kap.bild(k, erstes=(k == 0))
            pipe.stdin.write(letztes.tobytes())
            n += 1
            if n % 500 == 0:
                print(f"{n} Bilder ({time.time() - t0:.0f} s), Kapitel {kap.nr}", flush=True)
        bild = kap.schlussbild(letztes).tobytes()
        for _ in range(int(ENDE_S * FPS)):
            pipe.stdin.write(bild)
        kap.renderer.close()
        del kap
    pipe.stdin.close()
    pipe.wait()
    print(f"Video: {ausgabe} ({os.path.getsize(ausgabe) / 1e6:.1f} MB, {time.time() - t0:.0f} s)")


if __name__ == "__main__":
    main()
