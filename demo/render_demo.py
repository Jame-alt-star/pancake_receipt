#!/usr/bin/env python3
"""Rendert aus der Aufzeichnung (pose.npz + ereignisse.json + lauf.json) ein Demo-Video (H.264).

Aufruf: python3 demo/render_demo.py <laufordner> <ausgabe.mp4> [--probe SEKUNDEN]
  --probe: nur ein Standbild bei t Sekunden nach Videostart (zum Pruefen), Ausgabe als PNG.
Ansicht: Verfolgerkamera (3D) mit den Treffern des 3D-Lidars, Leitstand-Karte (2D, aus der Hinderniswolke
akkumuliert wie in roboter_sensorik), Auftrag mit Fortschritt, Titel und Zeit. Welt, Orte und Lidar kommen aus
denselben Dateien und Bausteinen wie im System (roboter_welten, roboter_beschreibung, roboter_simulation/lidar.py).
"""
import ast, json, math, os, subprocess, sys, time
import cv2, mujoco, numpy as np, yaml
from PIL import Image, ImageDraw, ImageFont

os.environ.setdefault("MUJOCO_GL", "glfw")
RUN, OUT = sys.argv[1], sys.argv[2]
PROBE = float(sys.argv[sys.argv.index("--probe") + 1]) if "--probe" in sys.argv else None
W, H, FPS = 1280, 720, 25
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")   # relativ zum Repo
sys.path.insert(0, os.path.join(SRC, "roboter_simulation"))
sys.path.insert(0, os.path.join(SRC, "roboter_beschreibung"))
from roboter_beschreibung.geometrie import quat_zu_rotmat, yaw_aus_quat  # noqa: E402
from roboter_simulation.lidar import Lidar  # noqa: E402
from roboter_simulation.szene import lade_modell  # noqa: E402

LAUF = json.load(open(f"{RUN}/lauf.json")) if os.path.exists(f"{RUN}/lauf.json") else {"mission": "demo_lager",
                                                                                     "welt": "demo_lager", "slam": True}
WELT_DIR = os.path.join(SRC, "roboter_welten", "welten", LAUF["welt"])
ROBOTER = yaml.safe_load(open(os.path.join(SRC, "roboter_beschreibung", "config", "roboter.yaml")))
MISSION = yaml.safe_load(open(os.path.join(SRC, "roboter_task_planning", "missions", LAUF["mission"] + ".yaml")))
ORTE_YAML = yaml.safe_load(open(os.path.join(WELT_DIR, "orte.yaml")))["orte"]

pose = np.load(f"{RUN}/pose.npz")
events = json.load(open(f"{RUN}/ereignisse.json"))
wall, qpos = pose["wall"], pose["qpos"]

m = lade_modell(os.path.join(SRC, "roboter_beschreibung", ROBOTER["modell_xml"]), os.path.join(WELT_DIR, "szene.xml"))
d = mujoco.MjData(m)
# Waende halbtransparent, damit die Verfolgerkamera nicht verdeckt wird
for g in range(m.ngeom):
    name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
    if name == "floor":
        m.geom_matid[g] = -1
        m.geom_rgba[g] = [0.30, 0.34, 0.38, 1.0]
    if name.startswith(("wand_", "trenn_")):
        m.geom_matid[g] = -1
        m.geom_rgba[g] = [0.80, 0.82, 0.86, 0.22]
    elif name.startswith(("stapel_", "regal_", "schrank", "hindernis")):
        base = m.mat_rgba[m.geom_matid[g]].copy() if m.geom_matid[g] >= 0 else m.geom_rgba[g].copy()
        m.geom_matid[g] = -1
        m.geom_rgba[g] = [base[0], base[1], base[2], 0.62]
m.vis.global_.offwidth, m.vis.global_.offheight = W, H

# 3D-Lidar wie in der Simulation (24 Ringe -40..+10 Grad, 360 Azimutwerte, Montage ueber dem Becken)
LIDAR = Lidar(m, float(ROBOTER["lidar_hoehe_ueber_becken"]), 360, 10.0, ringe=24)
BAND = (0.12, 1.9)            # Hoehenband der Hinderniswolke [m ueber Boden], wie roboter_sensorik


def lidar_welt(q):
    """Treffer der 3D-Punktwolke in Weltkoordinaten; Maske der Hindernispunkte (Hoehenband, ohne Boden)."""
    d.qpos[:] = q
    mujoco.mj_kinematics(m, d)
    pts = LIDAR.wolke(d, q[3:7])                       # im lidar_link-Frame
    R = quat_zu_rotmat(q[3:7])
    welt = q[0:3] + R @ np.array([0.0, 0.0, LIDAR.hoehe]) + pts.astype(float) @ R.T
    hind = (welt[:, 2] >= BAND[0]) & (welt[:, 2] <= BAND[1])
    return welt, hind


def yaw_of(q):
    return yaw_aus_quat(q[3:7])


# ---- Zeitachse -----------------------------------------------------------------------
akt1 = next(e["wall"] for e in events if e.get("text", "").startswith("Akt 1"))
t_start = akt1 - 4.0
t_end = next(e["wall"] for e in events if e.get("text", "").startswith("Ende")) + 1.0
ev_plan = [e for e in events if e.get("phase") == "plan"]
ev_step = [e for e in events if e.get("phase") == "schritt"]
ev_end = [e for e in events if e.get("phase") == "ende"]
t_akt2 = next(e["wall"] for e in events if e.get("text", "").startswith("Akt 2"))
t_notaus = next(e["wall"] for e in events if "NOT-AUS" in e.get("text", ""))
PLAN = (ev_plan[0]["plan"] if ev_plan else [])
if isinstance(PLAN, str):
    PLAN = ast.literal_eval(PLAN)

# ---- Leitstand-Karte (2D) ------------------------------------------------------------
boxes = []
for g in range(m.ngeom):
    if m.geom_group[g] == 2 and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
        p, s = m.geom_pos[g], m.geom_size[g]
        boxes.append((p[0] - s[0], p[1] - s[1], p[0] + s[0], p[1] + s[1]))
X0, Y0 = min(b[0] for b in boxes) - 0.5, min(b[1] for b in boxes) - 0.5
X1, Y1 = max(b[2] for b in boxes) + 0.5, max(b[3] for b in boxes) + 0.5
MAPW = 430
SC = MAPW / (X1 - X0)
MAPH = int(round((Y1 - Y0) * SC))


def to_px(x, y):
    return int((x - X0) * SC), int(MAPH - (y - Y0) * SC)


# Anzeigenamen der Orte (Lager-Demo mit sprechenden Namen, sonst Ortsname gross)
SCHOEN = {"start": "Start", "k1": "K1 Regalgang", "k2": "K2 Hallenecke", "k3": "K3 Buero", "lader": "Ladestation"}
ORTE = {k: (v["x"], v["y"], SCHOEN.get(k, k.upper() if len(k) <= 2 else k.capitalize())) for k, v in ORTE_YAML.items()}
NAME = {k: v[2] for k, v in ORTE.items()}
seen = np.zeros((MAPH, MAPW), dtype=np.uint8)       # akkumulierte Hindernispunkte
trail = []

# ---- Schrift --------------------------------------------------------------------------
def font(sz, bold=False):
    p = "/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf" % ("-Bold" if bold else "")
    return ImageFont.truetype(p, sz)


F_TITLE, F_BIG, F_MID, F_SM = font(28, True), font(19, True), font(16), font(14)
SCHRITT_TXT = {"gehe_zu": "Fahre zu", "balanciere": "Balanciere bei", "warte": "Warte bei"}
ZIEL_TXT = {"besucht": "{} anfahren", "balanciert": "bei {} balancieren", "gewartet": "bei {} warten",
            "am_ort": "zum Schluss nach {}"}


def step_label(s):
    n = s.split("(")[0]
    a = [x.strip() for x in s.split("(")[1].rstrip(")").split(",")]
    return f"{SCHRITT_TXT.get(n, n)} {NAME.get(a[-1], a[-1])}"


def auftrag_zeilen(breite_zeichen=46):
    teile = [ZIEL_TXT[z["art"]].format(NAME.get(z["ort"], z["ort"])) for z in MISSION["ziele"]]
    text, zeilen = "Auftrag: " + ", ".join(teile), []
    while len(text) > breite_zeichen:
        cut = text.rfind(" ", 0, breite_zeichen)
        zeilen.append(text[:cut])
        text = text[cut + 1:]
    return zeilen + [text]


AUFTRAG = auftrag_zeilen()


def state_at(tw):
    """(aktueller Schritt-Index, fertig?) zur Wandzeit tw."""
    idx, done = -1, False
    for e in ev_step:
        if e["wall"] <= tw:
            idx = int(e["index"])
    for e in ev_end:
        if e["wall"] <= tw:
            done = True
    return idx, done


renderer = mujoco.Renderer(m, H, W, max_geom=4000)
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
look = None
yaw_s = None
opt = mujoco.MjvOption()
opt.geomgroup[3:] = 0
TITEL = "Humanoider Roboter G1 - autonomer Kontrollrundgang"
UNTERTITEL = ("Simulation: Planung (PDDL) - Navigation (Nav2" + (" + SLAM" if LAUF.get("slam") else "")
              + ") - 3D-Lidar - Balance-Regelung (RL-Policy)")


def karte_eintragen(welt, hind):
    for p in welt[hind]:
        px, py = to_px(p[0], p[1])
        if 0 <= px < MAPW and 0 <= py < MAPH:
            seen[py, px] = 255


def punktfarbe(z):
    """niedrig gruen -> hoch orange (die niedrige Palette faellt so auf)."""
    a = float(np.clip((z - BAND[0]) / 1.2, 0.0, 1.0))
    return np.array([0.25 + 0.75 * a, 0.85 - 0.55 * a, 0.25 - 0.15 * a, 0.95], dtype=np.float32)


def frame_at(tw, first=False):
    global look, yaw_s
    i = int(np.clip(np.searchsorted(wall, tw), 0, len(wall) - 1))
    q = qpos[i]
    welt, hind = lidar_welt(q)
    mujoco.mj_forward(m, d)
    pos = q[0:3]
    yw = yaw_of(q)
    if yaw_s is None or first:
        yaw_s = yw
        look = pos.copy()
    dy = (yw - yaw_s + math.pi) % (2 * math.pi) - math.pi
    yaw_s += 0.06 * dy                                  # Kamera folgt der Blickrichtung traege
    look = look + 0.15 * (pos - look)
    cam.lookat[:] = [look[0], look[1], 0.8]
    cam.distance, cam.elevation = 5.6, -52
    cam.azimuth = math.degrees(yaw_s) + 180.0 + 28.0    # hinter dem Roboter, leicht seitlich versetzt
    renderer.update_scene(d, camera=cam, scene_option=opt)
    sc = renderer.scene
    nah = welt[hind & (np.linalg.norm(welt[:, :2] - pos[:2], axis=1) < 6.0)]
    for p in nah[::3]:
        if sc.ngeom >= sc.maxgeom - 2:
            break
        mujoco.mjv_initGeom(sc.geoms[sc.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.025, 0, 0]),
                            p, np.eye(3).flatten(), punktfarbe(p[2]))
        sc.ngeom += 1
    img = renderer.render().copy()

    # ---- Leitstand-Karte ----
    karte_eintragen(welt, hind)
    trail.append((pos[0], pos[1]))
    mp = np.full((MAPH, MAPW, 3), (28, 32, 38), dtype=np.uint8)
    mp[seen > 0] = (230, 230, 235)                     # aus der 3D-Hinderniswolke akkumuliert
    mp = cv2.dilate(mp, np.ones((2, 2), np.uint8))
    for x0, y0, x1, y1 in boxes:                       # Umriss der Welt, nur schwach (Orientierung)
        cv2.rectangle(mp, to_px(x0, y1), to_px(x1, y0), (70, 75, 85), 1)
    for k, (ox, oy, _) in ORTE.items():
        cv2.circle(mp, to_px(ox, oy), 6, (90, 200, 120) if k in ("lader", "start") else (80, 170, 255), 2)
    if len(trail) > 2:
        pts = np.array([to_px(x, y) for x, y in trail[::4]], dtype=np.int32)
        cv2.polylines(mp, [pts], False, (255, 190, 60), 2)
    rc = to_px(pos[0], pos[1])
    cv2.circle(mp, rc, 7, (60, 230, 90), -1)
    cv2.line(mp, rc, (int(rc[0] + 16 * math.cos(yw)), int(rc[1] - 16 * math.sin(yw))), (255, 255, 255), 3)
    cv2.rectangle(mp, (0, 0), (MAPW - 1, MAPH - 1), (200, 200, 200), 1)
    MX, MY = W - MAPW - 18, 92
    img[MY:MY + MAPH, MX:MX + MAPW] = mp

    # ---- Beschriftung (PIL) ----
    pil = Image.fromarray(img)
    dr = ImageDraw.Draw(pil, "RGBA")
    dr.rectangle([0, 0, W, 78], fill=(14, 18, 26, 215))
    dr.text((22, 9), TITEL, font=F_TITLE, fill=(255, 255, 255, 255))
    dr.text((22, 48), UNTERTITEL, font=F_SM, fill=(170, 200, 235, 255))
    for k, (ox, oy, name) in ORTE.items():             # Ortsnamen in die Karte
        px, py = to_px(ox, oy)
        dr.text((MX + px + 9, MY + py - 8), name, font=F_SM, fill=(200, 215, 235, 255))
    dr.rounded_rectangle([MX, MY + MAPH + 4, MX + MAPW, MY + MAPH + 28], radius=7, fill=(14, 18, 26, 215))
    dr.text((MX + 9, MY + MAPH + 7), "Leitstand: 3D-Lidar-Hindernisse + Fahrspur", font=F_SM, fill=(255, 255, 255, 255))

    # Auftragsfenster unten links, kompakt (Hoehe nach Inhalt)
    idx, done = state_at(tw)
    BW, ZEILE = 430, 21
    if tw < t_akt2:
        hoehe = 12 + len(AUFTRAG) * 18 + 8 + len(PLAN) * ZEILE + 34
    else:
        hoehe = 112
    BX, BY = 14, H - 14 - hoehe
    dr.rounded_rectangle([BX, BY, BX + BW, H - 14], radius=10, fill=(14, 18, 26, 215))
    px0 = BX + 12
    if tw < t_akt2:
        for j, z in enumerate(AUFTRAG):
            dr.text((px0, BY + 8 + j * 18), z, font=F_SM, fill=(255, 255, 255, 255))
        y = BY + 12 + len(AUFTRAG) * 18 + 6
        for j, st in enumerate(PLAN):
            if done or j < idx:
                mark, col = "[x]", (110, 220, 130, 255)
            elif j == idx:
                mark, col = "[>]", (255, 200, 60, 255)
            else:
                mark, col = "[ ]", (170, 170, 170, 255)
            dr.text((px0, y + j * ZEILE), f"{mark} {j + 1}. {step_label(st)}", font=F_MID, fill=col)
        status = "Mission erfolgreich abgeschlossen" if done else "Mission laeuft"
        dr.text((px0, H - 14 - 28), status, font=F_BIG, fill=(110, 220, 130, 255) if done else (255, 200, 60, 255))
    else:
        if tw >= t_notaus:
            dr.text((px0, BY + 10), "NOT-AUS ausgeloest", font=F_TITLE, fill=(255, 90, 80, 255))
            dr.text((px0, BY + 52), "Roboter bremst aktiv und steht stabil.", font=F_MID, fill=(255, 255, 255, 255))
            dr.text((px0, BY + 76), "Mission abgebrochen, kein Neustart.", font=F_MID, fill=(255, 255, 255, 255))
        else:
            dr.text((px0, BY + 12), "Sicherheitstest: erneute Mission,", font=F_BIG, fill=(255, 255, 255, 255))
            dr.text((px0, BY + 40), "dann NOT-AUS waehrend der Fahrt", font=F_BIG, fill=(255, 255, 255, 255))
            dr.text((px0, BY + 76), "Roboter faehrt los ...", font=F_MID, fill=(255, 200, 60, 255))
    # Zeitanzeige
    dr.rounded_rectangle([W - 250, H - 42, W - 18, H - 14], radius=8, fill=(14, 18, 26, 215))
    dr.text((W - 240, H - 38), f"t = {tw - akt1 + 0:5.1f} s (Echtzeit)", font=F_SM, fill=(230, 230, 230, 255))
    if t_notaus <= tw < t_notaus + 6:
        dr.rectangle([0, 0, W - 1, H - 1], outline=(255, 70, 60, 255), width=10)
    return np.array(pil)


if PROBE is not None:
    # Karte bis zum Standbild aufbauen (alle 0,4 s), dann ein Bild rendern
    t_probe = t_start + PROBE
    tw = t_start
    while tw < t_probe:
        i = int(np.clip(np.searchsorted(wall, tw), 0, len(wall) - 1))
        karte_eintragen(*lidar_welt(qpos[i]))
        trail.append((qpos[i][0], qpos[i][1]))
        tw += 0.4
    Image.fromarray(frame_at(t_probe, first=True)).save(OUT)
    print("Standbild:", OUT)
    sys.exit(0)

ffmpeg = subprocess.check_output([sys.executable, "-c",
                                  "import imageio_ffmpeg;print(imageio_ffmpeg.get_ffmpeg_exe())"], text=True).strip()
cmd = [ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS),
       "-i", "-", "-c:v", "libx264", "-preset", "medium", "-crf", "21", "-pix_fmt", "yuv420p", "-movflags", "+faststart", OUT]
pipe = subprocess.Popen(cmd, stdin=subprocess.PIPE)
n = int((t_end - t_start) * FPS)
t0 = time.time()
for k in range(n):
    tw = t_start + k / FPS
    pipe.stdin.write(frame_at(tw, first=(k == 0)).tobytes())
    if k % 250 == 0:
        print(f"{k}/{n} Bilder ({time.time() - t0:.0f} s)", flush=True)
pipe.stdin.close()
pipe.wait()
print(f"Video: {OUT} ({n / FPS:.0f} s, {os.path.getsize(OUT) / 1e6:.1f} MB)")
