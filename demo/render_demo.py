#!/usr/bin/env python3
"""Rendert aus der Aufzeichnung (pose.npz + ereignisse.json) ein Demo-Video (H.264).

Aufruf: python3 demo/render_demo.py <laufordner> <ausgabe.mp4> [--probe SEKUNDEN]
  --probe: nur ein Standbild bei t Sekunden nach Videostart (zum Pruefen), Ausgabe als PNG.
Ansicht: Verfolgerkamera (3D) mit Lidar-Treffern, Leitstand-Karte (2D, aus Lidar akkumuliert),
Missionsfortschritt als Leiste, Titel und Zustand.
"""
import json, math, os, subprocess, sys, time
import cv2, mujoco, numpy as np
from PIL import Image, ImageDraw, ImageFont

os.environ.setdefault("MUJOCO_GL", "glfw")
RUN, OUT = sys.argv[1], sys.argv[2]
PROBE = float(sys.argv[sys.argv.index("--probe") + 1]) if "--probe" in sys.argv else None
W, H, FPS = 1280, 720, 25
SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src")   # relativ zum Repo
ROBOTER = os.path.join(SRC, "r1_bringup", "models", "robots", "g1_description", "g1_29dof.xml")
WELT = os.path.join(SRC, "roboter_welten", "welten", "demo_lager", "szene.xml")
sys.path.insert(0, os.path.join(SRC, "r1_bringup"))
from r1_bringup.szene import lade_modell  # noqa: E402  (Roboter + Welt wie in der Simulation)

pose = np.load(f"{RUN}/pose.npz")
events = json.load(open(f"{RUN}/ereignisse.json"))
wall, qpos = pose["wall"], pose["qpos"]

m = lade_modell(ROBOTER, WELT)
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
    elif name.startswith(("stapel_", "regal_", "schrank")):
        base = m.mat_rgba[m.geom_matid[g]].copy() if m.geom_matid[g] >= 0 else m.geom_rgba[g].copy()
        m.geom_matid[g] = -1
        m.geom_rgba[g] = [base[0], base[1], base[2], 0.62]
m.vis.global_.offwidth, m.vis.global_.offheight = W, H
pelvis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
mask = np.array([0, 0, 1, 0, 0, 0], dtype=np.uint8)
NRAY = 360
ang = np.linspace(-np.pi, np.pi, NRAY, endpoint=False)
dirs_local = np.stack([np.cos(ang), np.sin(ang), np.zeros(NRAY)], 1)


def rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def lidar(q):
    R = rot(q[3:7])
    org = q[0:3] + R @ np.array([0, 0, 0.45])
    dirs = dirs_local @ R.T
    dist = np.zeros(NRAY)
    gid = np.zeros(NRAY, dtype=np.int32)
    mujoco.mj_multiRay(m, d, org, dirs.reshape(-1), mask, True, -1, gid, dist, None, NRAY, 10.0)
    ok = (gid >= 0) & (dist > 0.15) & (dist < 10.0)
    return org, org + dirs[ok] * dist[ok, None]


def yaw_of(q):
    w, x, y, z = q[3:7]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


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
    PLAN = eval(PLAN)

# ---- Leitstand-Karte (2D) ------------------------------------------------------------
MAPW, MAPH = 500, 350
X0, X1, Y0, Y1 = -2.0, 10.5, -4.0, 4.0
SC = min(MAPW / (X1 - X0), MAPH / (Y1 - Y0))


def to_px(x, y):
    return int((x - X0) * SC), int(MAPH - (y - Y0) * SC)


boxes = []
for g in range(m.ngeom):
    if m.geom_group[g] == 2 and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
        p, s = m.geom_pos[g], m.geom_size[g]
        boxes.append((p[0] - s[0], p[1] - s[1], p[0] + s[0], p[1] + s[1]))
ORTE = {"start": (0.0, 0.0, "Start"), "k1": (3.0, 0.0, "K1 Regalgang"), "k2": (5.5, -2.6, "K2 Hallenecke"),
        "k3": (8.5, 1.5, "K3 Buero"), "lader": (0.0, -2.6, "Ladestation")}
seen = np.zeros((MAPH, MAPW), dtype=np.uint8)       # akkumulierte Lidar-Treffer
trail = []

# ---- Schrift --------------------------------------------------------------------------
def font(sz, bold=False):
    p = "/usr/share/fonts/truetype/dejavu/DejaVuSans%s.ttf" % ("-Bold" if bold else "")
    return ImageFont.truetype(p, sz)


F_TITLE, F_BIG, F_MID, F_SM = font(30, True), font(24, True), font(20), font(17)
SCHRITT_TXT = {"gehe_zu": "Fahre zu", "balanciere": "Balanciere am", "warte": "Warte am"}
NAME = {k: v[2] for k, v in ORTE.items()}


def step_label(s):
    n = s.split("(")[0]
    a = [x.strip() for x in s.split("(")[1].rstrip(")").split(",")]
    return f"{SCHRITT_TXT[n]} {NAME.get(a[-1], a[-1])}"


def state_at(tw):
    """(aktueller Schritt-Index, fertig?, Missionsstatus-Text) zur Wandzeit tw."""
    idx, done = -1, False
    for e in ev_step:
        if e["wall"] <= tw:
            idx = int(e["index"])
    for e in ev_end:
        if e["wall"] <= tw:
            done = True
    return idx, done


renderer = mujoco.Renderer(m, H, W, max_geom=3000)
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
look = None
yaw_s = None
opt = mujoco.MjvOption()
opt.geomgroup[3:] = 0


def frame_at(tw, first=False):
    global look, yaw_s
    i = int(np.clip(np.searchsorted(wall, tw), 0, len(wall) - 1))
    q = qpos[i]
    d.qpos[:] = q
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
    org, hits = lidar(q)
    renderer.update_scene(d, camera=cam, scene_option=opt)
    sc = renderer.scene
    near = hits[np.linalg.norm(hits[:, :2] - pos[:2], axis=1) < 6.0] if len(hits) else hits
    for p in near[::4]:
        if sc.ngeom >= sc.maxgeom - 2:
            break
        mujoco.mjv_initGeom(sc.geoms[sc.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.03, 0, 0]),
                            p, np.eye(3).flatten(), np.array([1.0, 0.25, 0.1, 0.9], dtype=np.float32))
        sc.ngeom += 1
    img = renderer.render().copy()

    # ---- Leitstand-Karte ----
    for p in hits:
        px, py = to_px(p[0], p[1])
        if 0 <= px < MAPW and 0 <= py < MAPH:
            seen[py, px] = 255
    trail.append((pos[0], pos[1]))
    mp = np.full((MAPH, MAPW, 3), (28, 32, 38), dtype=np.uint8)
    mp[seen > 0] = (230, 230, 235)                     # akkumulierte Lidar-Karte
    mp = cv2.dilate(mp, np.ones((2, 2), np.uint8))
    for x0, y0, x1, y1 in boxes:                       # Umriss der Welt, nur schwach (Orientierung)
        a, b = to_px(x0, y1), to_px(x1, y0)
        cv2.rectangle(mp, a, b, (70, 75, 85), 1)
    for k, (ox, oy, name) in ORTE.items():
        c = to_px(ox, oy)
        cv2.circle(mp, c, 7, (90, 200, 120) if k == "lader" else (80, 170, 255), 2)
    if len(trail) > 2:
        pts = np.array([to_px(x, y) for x, y in trail[::4]], dtype=np.int32)
        cv2.polylines(mp, [pts], False, (255, 190, 60), 2)
    rc = to_px(pos[0], pos[1])
    cv2.circle(mp, rc, 8, (60, 230, 90), -1)
    cv2.line(mp, rc, (int(rc[0] + 18 * math.cos(yw)), int(rc[1] - 18 * math.sin(yw))), (255, 255, 255), 3)
    cv2.rectangle(mp, (0, 0), (MAPW - 1, MAPH - 1), (200, 200, 200), 1)
    img[96:96 + MAPH, W - MAPW - 18:W - 18] = mp

    # ---- Beschriftung (PIL) ----
    pil = Image.fromarray(img)
    dr = ImageDraw.Draw(pil, "RGBA")
    dr.rectangle([0, 0, W, 84], fill=(14, 18, 26, 215))
    dr.text((24, 10), "Humanoider Roboter G1 - autonomer Kontrollrundgang", font=F_TITLE, fill=(255, 255, 255, 255))
    dr.text((24, 50), "Simulation: Planung (PDDL) - Navigation (Nav2 + SLAM) - Balance-Regelung (RL-Policy)",
            font=F_SM, fill=(170, 200, 235, 255))
    dr.rounded_rectangle([W - MAPW - 18, 96 + MAPH + 4, W - 18, 96 + MAPH + 34], radius=8, fill=(14, 18, 26, 215))
    dr.text((W - MAPW - 8, 96 + MAPH + 8), "Leitstand: Lidar-Karte + Fahrspur", font=F_SM, fill=(255, 255, 255, 255))

    idx, done = state_at(tw)
    BX, BY = 14, H - 305
    dr.rounded_rectangle([BX, BY, 690, H - 14], radius=12, fill=(14, 18, 26, 222))
    px0 = BX + 14
    if tw < t_akt2:
        dr.text((px0, BY + 10), "Auftrag: Pruefpunkte K1-K3 anfahren, im Buero", font=F_SM, fill=(255, 255, 255, 255))
        dr.text((px0, BY + 32), "balancieren und warten, dann zur Ladestation", font=F_SM, fill=(255, 255, 255, 255))
        for j, st in enumerate(PLAN):
            ycol = BY + 66 + j * 27
            if done or j < idx:
                mark, col = "[x]", (110, 220, 130, 255)
            elif j == idx:
                mark, col = "[>]", (255, 200, 60, 255)
            else:
                mark, col = "[ ]", (170, 170, 170, 255)
            dr.text((px0, ycol), f"{mark} {j + 1}. {step_label(st)}", font=F_MID, fill=col)
        status = "Mission erfolgreich abgeschlossen" if done else "Mission laeuft"
        dr.text((px0, H - 52), status, font=F_BIG, fill=(110, 220, 130, 255) if done else (255, 200, 60, 255))
    else:
        if tw >= t_notaus:
            dr.text((px0, BY + 30), "NOT-AUS ausgeloest", font=F_TITLE, fill=(255, 90, 80, 255))
            dr.text((px0, BY + 86), "Roboter bremst aktiv und steht stabil", font=F_MID, fill=(255, 255, 255, 255))
            dr.text((px0, BY + 118), "Mission wird abgebrochen,", font=F_MID, fill=(255, 255, 255, 255))
            dr.text((px0, BY + 148), "kein automatischer Neustart", font=F_MID, fill=(255, 255, 255, 255))
        else:
            dr.text((px0, BY + 30), "Sicherheitstest: erneute Mission,", font=F_BIG, fill=(255, 255, 255, 255))
            dr.text((px0, BY + 64), "dann NOT-AUS waehrend der Fahrt", font=F_BIG, fill=(255, 255, 255, 255))
            dr.text((px0, BY + 120), "Roboter faehrt los ...", font=F_MID, fill=(255, 200, 60, 255))
    # Zeitanzeige
    dr.rounded_rectangle([W - 270, H - 46, W - 18, H - 14], radius=8, fill=(14, 18, 26, 215))
    dr.text((W - 260, H - 42), f"t = {tw - akt1 + 0:5.1f} s (Echtzeit)", font=F_SM, fill=(230, 230, 230, 255))
    if tw >= t_notaus and tw < t_notaus + 6:
        dr.rectangle([0, 0, W - 1, H - 1], outline=(255, 70, 60, 255), width=10)
    return np.array(pil)


if PROBE is not None:
    # Zustand der Karte vor dem Standbild aufbauen: alle 0,5 s bis dahin rendern
    t_probe = t_start + PROBE
    tw = t_start
    while tw < t_probe:
        frame_at(tw, first=(tw == t_start)) if False else None
        i = int(np.clip(np.searchsorted(wall, tw), 0, len(wall) - 1))
        _, h = lidar(qpos[i]) if (d.qpos.__setitem__(slice(None), qpos[i]) or True) else (0, 0)
        mujoco.mj_forward(m, d)
        for p in h:
            px, py = to_px(p[0], p[1])
            if 0 <= px < MAPW and 0 <= py < MAPH:
                seen[py, px] = 255
        trail.append((qpos[i][0], qpos[i][1]))
        tw += 0.4
    img = frame_at(t_probe, first=True)
    Image.fromarray(img).save(OUT)
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
