#!/usr/bin/env python3
"""Integrationstest Layer 1 + 3 + 4 in der Simulation (Nav2-TESTDOUBLE statt Layer 2).

Aufruf: python3 tests/system/integration_mission.py [--nav2]   (ROS2 + Workspace gesourct, Display noetig)
        --nav2: echtes Nav2 aus roboter_navigation (ros-jazzy-navigation2 muss installiert sein)
        --karte: Navigation auf der festen Karte der Welt (map_server + AMCL, Wegkosten ueber den Nav2-Planer)
        --aufnehmen: wie in echt - (1) Welt mit SLAM abfahren, Karte speichern (map_saver_cli), (2) neu starten und die
                     Mission nur mit der gespeicherten Karte (map_server + AMCL) fahren
        --kueche: Mission libero_kueche in der Kuechenwelt mit LIBERO-Fixtures/-Objekten (Weltgelenke + freie Objekte
                  im Modell), echtes Nav2
        --aufnahme <ordner>: Pose, Schritte und Ergebnisse fuer das Testvideo mitschreiben (testaufnahme.py)
Szenarien: (1) Mission laeuft komplett durch, (2) NotAus waehrend der Mission bricht ab, Start per Name abgelehnt.
"""
import json, os, shutil, signal, subprocess, sys, tempfile, time
from types import SimpleNamespace

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from std_srvs.srv import Trigger

from roboter_interfaces.srv import MissionStarten
from testaufnahme import Aufnahme

R = []
AUF = Aufnahme(None, "")        # je Lauf bzw. Phase neu gesetzt (main)


def check(name, ok, detail=""):
    R.append((name, bool(ok), detail))
    AUF.check(name, ok, detail)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}  {detail}", flush=True)


class Probe(Node):
    def __init__(self):
        super().__init__("mission_probe")
        lat = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.p = None
        self.min_clear = 9.0      # kleinster Abstand Roboterzentrum -> Hindernisrechtecke
        self.status = []
        self.create_subscription(Point, "/base_position", self._on_pos, 10)
        lat50 = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/mission/status", self._on_status, lat50)
        self.not_aus = self.create_publisher(Bool, "/not_aus", lat)
        self.map = None
        self.tf_map_odom = False
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m), lat)
        self.create_subscription(TFMessage, "/tf", self._tf, 50)
        self.start = self.create_client(Trigger, "/mission/start")
        self.starte = self.create_client(MissionStarten, "/mission/starte")

    def _on_status(self, m):
        s = json.loads(m.data)
        self.status.append(s)
        AUF.ereignis("mission", **s)

    def _tf(self, m):
        for t in m.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "odom":
                self.tf_map_odom = True

    def map_occupied(self, x, y, r=0.15):
        """True, wenn im Umkreis r um (x, y) eine belegte Zelle (>50) der /map liegt."""
        m = self.map
        if m is None:
            return False
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        k = int(round(r / res))
        cx, cy = int((x - ox) / res), int((y - oy) / res)
        for j in range(cy - k, cy + k + 1):
            for i in range(cx - k, cx + k + 1):
                if 0 <= i < m.info.width and 0 <= j < m.info.height and m.data[j * m.info.width + i] > 50:
                    return True
        return False

    def map_occupied_rect(self, x0, x1, y0, y1):
        """Anzahl belegter Zellen (>50) im Rechteck der /map (Karten-Rasterung robust)."""
        m = self.map
        if m is None:
            return 0
        res = m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        n = 0
        for j in range(max(0, int((y0 - oy) / res)), min(m.info.height, int((y1 - oy) / res) + 1)):
            for i in range(max(0, int((x0 - ox) / res)), min(m.info.width, int((x1 - ox) / res) + 1)):
                if m.data[j * m.info.width + i] > 50:
                    n += 1
        return n

    def _on_pos(self, m):
        self.p = (m.x, m.y, m.z)
        for x0, x1, y0, y1 in BOXEN:
            dx = max(x0 - m.x, 0.0, m.x - x1)
            dy = max(y0 - m.y, 0.0, m.y - y1)
            self.min_clear = min(self.min_clear, float(np.hypot(dx, dy)))

    def spin(self, s):
        e = time.monotonic() + s          # monotone Uhr: die Systemzeit der WSL springt (siehe integration_ros.py)
        while time.monotonic() < e:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_end(self, timeout):
        e = time.monotonic() + timeout
        while time.monotonic() < e:
            self.spin(0.2)
            if self.status and self.status[-1].get("phase") == "ende":
                return self.status[-1]
        return None

    def starte_mission(self, name):
        f = self.starte.call_async(MissionStarten.Request(mission=name))
        while not f.done():
            self.spin(0.05)
        return f.result()

    def start_mission(self):
        n0 = len(self.status)
        f = self.start.call_async(Trigger.Request())
        while not f.done():
            self.spin(0.05)
        return f.result(), n0


NAV2 = "--nav2" in sys.argv     # echtes Nav2 (roboter_navigation) statt Testdouble
DREI_D = "--drei-d" in sys.argv       # Welt mit niedriger Palette (3D-Lidar ist Standard) (impliziert --welt)
NUR_2D = "--nur-2d" in sys.argv        # Gegenprobe: dieselbe Welt mit alter Einzel-Scanebene (Kontakt erwartet)
SLAM = "--slam" in sys.argv       # slam_toolbox (impliziert --welt); prueft /map und map->odom
KARTE = "--karte" in sys.argv     # feste Karte + AMCL (impliziert --welt); prueft /map, map->odom, Wegkosten
PATROUILLE = "--patrouille" in sys.argv     # Patrouille a, b, zurueck zum Start (impliziert --welt)
AUFNEHMEN = "--aufnehmen" in sys.argv     # Karte mit SLAM aufnehmen, dann darauf arbeiten (impliziert --welt)
KUECHE = "--kueche" in sys.argv     # Kuechenwelt libero_kueche (impliziert --nav2)
WELT = "--welt" in sys.argv or PATROUILLE or SLAM or DREI_D or NUR_2D or KARTE or AUFNEHMEN   # Testwelt mit Waenden/Hindernis + Lidar (impliziert --nav2)
if WELT or KUECHE:
    NAV2 = True
# Die Simulation uebernimmt die Welt aus der Missionsdatei (Schluessel welt).
MISSION = ("libero_kueche" if KUECHE else
           ("patrouille" if PATROUILLE else ("testwelt3d" if (DREI_D or NUR_2D) else "testwelt")) if WELT else "beispiel")
# Hindernisse der Testwelt als Rechtecke (xmin, xmax, ymin, ymax); Roboter ~0.25 m Radius
BOXEN = [(1.25, 1.75, -0.8, 0.8), (4.45, 4.55, 0.8, 2.95), (4.45, 4.55, -2.95, -0.8)]
if DREI_D or NUR_2D:
    BOXEN = BOXEN + [(2.0, 2.8, -1.5, -0.7)]     # niedrige Palette (0.45 m)
if KUECHE:      # Kuechenzeile, Kuehlschrank, Esstisch (roboter_welten/welten/libero_kueche/raum.xml)
    BOXEN = [(0.8, 4.2, 2.37, 3.0), (4.4, 5.1, 2.3, 3.0), (1.6, 2.8, -2.0, -1.2)]


def launch(logfile, extra=()):
    env = dict(os.environ)
    nav = ["navigation:=nav2", "headless:=true"] if NAV2 else ["navigation:=testdouble"]   # Nav2 + Viewer: zu wenig RAM
    nav += list(extra) + AUF.launch_args()
    if SLAM:
        nav += ["slam:=true"]
    if KARTE:
        nav += ["karte:=true"]
    if "--realistisch" in sys.argv:
        nav += ["realistic_sensors:=true"]
    if "--beinodometrie" in sys.argv:
        nav += ["odom_quelle:=beinodometrie"]
    if NUR_2D:
        nav += ["lidar_modus:=2d"]
    return subprocess.Popen(["ros2", "launch", "roboter_system", "system.launch.py", f"mission:={MISSION}", *nav],
                            stdout=open(logfile, "w"), stderr=subprocess.STDOUT, start_new_session=True, env=env)


def stop(proc):
    try:
        os.killpg(proc.pid, signal.SIGINT); proc.wait(timeout=20)
    except Exception:
        try: os.killpg(proc.pid, signal.SIGKILL)
        except Exception: pass


TITEL = ("Mission LIBERO-Kueche" if KUECHE else "Karte aufnehmen und damit arbeiten" if AUFNEHMEN else "Mission mit SLAM" if SLAM
         else "Mission auf fester Karte (AMCL)" if KARTE else "Mission 3D-Palette" if DREI_D
         else "Gegenprobe nur 2D-Lidar" if NUR_2D else "Mission Patrouille" if PATROUILLE
         else "Mission Testwelt (Nav2)" if WELT else "Mission mit Testdouble")


def lauf_info(**mehr):
    return dict(welt=None if MISSION == "beispiel" else ("libero_kueche" if KUECHE else
                                                         "testwelt3d" if (DREI_D or NUR_2D) else "testwelt"),
                mission=MISSION, navigation="nav2" if NAV2 else "testdouble",
                realistisch="--realistisch" in sys.argv, beinodometrie="--beinodometrie" in sys.argv, **mehr)


def system_bereit(pr, praefix="S0"):
    e = time.monotonic() + 90
    while time.monotonic() < e and not (pr.p and pr.start.service_is_ready()):
        pr.spin(0.2)
    check(f"{praefix} System bereit (Position + /mission/start)", pr.p is not None and pr.start.service_is_ready())
    pr.spin(60 if NAV2 else 25)     # Nav2-Lifecycle braucht laenger
    check(f"{praefix} Roboter steht (z>0.7)", pr.p[2] > 0.7, f"z={pr.p[2]:.3f}")


def mission_durchfahren(pr, log, praefix="S1", tol_b=0.6):
    """Mission starten, Ende abwarten, Ergebnis, Abstand zu Hindernissen und Ziel pruefen."""
    r, n0 = pr.start_mission()
    check(f"{praefix} Start akzeptiert", r.success, r.message)
    end = pr.wait_end(300)
    check(f"{praefix} Mission beendet", end is not None)
    if end:
        if not NUR_2D:
            check(f"{praefix} Status erfolg", end.get("status") == "erfolg", str(end))
    plans = [s for s in pr.status[n0:] if s.get("phase") == "plan"]
    if NUR_2D:
        # Gegenprobe: ohne 3D-Lidar ist die niedrige Palette unsichtbar -> Kontakt erwartet
        kontakt = pr.min_clear < 0.20
        check(f"{praefix} KONTROLLE: 2D-Lidar sieht die niedrige Palette nicht (Roboter kommt ihr zu nah)",
              kontakt, f"min_abstand={pr.min_clear:.2f} m (Koerperhalbbreite ~0.2: darunter Kontakt)")
    elif DREI_D:
        check(f"{praefix} 3D-Lidar: niedrige Palette umfahren, kein Kontakt (Abstand > 0.20 m)", pr.min_clear > 0.20,
              f"min_abstand={pr.min_clear:.2f} m")
    elif WELT or KUECHE:
        check(f"{praefix} kein Kontakt mit Hindernis/Waenden (Abstand > 0.2 m)", pr.min_clear > 0.2,
              f"min_abstand={pr.min_clear:.2f} m")
    else:
        check(f"{praefix} Plan besucht a vor b", bool(plans) and "gehe_zu(start, a)" in plans[0]["plan"]
              and plans[0]["plan"].index("gehe_zu(start, a)") < plans[0]["plan"].index("gehe_zu(a, b)"),
              str(plans[0]["plan"]) if plans else "")
    goal = (2.2, -0.55) if KUECHE else (0.0, 0.0) if PATROUILLE else ((5.8, 1.5) if WELT else (3.0, 2.0))
    dist_b = float(np.hypot(pr.p[0] - goal[0], pr.p[1] - goal[1]))
    check(f"{praefix} Roboter nahe {'tisch' if KUECHE else 'b'} (<{tol_b} m)", dist_b < tol_b, f"abstand={dist_b:.2f}")
    check(f"{praefix} steht (z>0.7)", pr.p[2] > 0.7, f"z={pr.p[2]:.3f}")
    check(f"{praefix} Freigabe nach Missionsende entzogen (Review 2)",
          "Bewegungsfreigabe entzogen" in open(log, errors="replace").read())


def karte_pruefen(pr, praefix):
    """Bekannte Waende in pr.map (Welt = map-Frame, da Start im Ursprung mit Yaw 0). Fenster entlang der Waende
    (x0, x1, y0, y1) - mind. 6 belegte Zellen = Wand erkannt."""
    for name, (x0, x1, y0, y1) in {
            "Westwand": (-1.7, -1.3, -1.5, 1.5),
            "Suedwand": (1.0, 3.0, -3.2, -2.7), "Hindernis": (1.1, 1.9, -0.6, 0.6),
            "Trennwand": (4.3, 4.7, 1.2, 2.6)}.items():
        n = pr.map_occupied_rect(x0, x1, y0, y1)
        check(f"{praefix} Karte enthaelt {name}", n >= 6, f"{n} belegte Zellen")
    # Die Nordwand liegt auf der oberen Rasterkante; slam_toolbox verwirft dort die aeusserste
    # Zeile. Geprueft wird daher nur, dass die Karte bis zur Nordwand reicht.
    top = pr.map.info.origin.position.y + pr.map.info.height * pr.map.info.resolution if pr.map else 0.0
    check(f"{praefix} Karte reicht bis zur Nordwand (y >= 2.9)", top >= 2.9, f"oberer Rand y={top:.2f}")
    check(f"{praefix} Tuer in der Karte frei", not pr.map_occupied(4.5, 0.0, 0.15))


def lade_karte(yaml_pfad):
    """Gespeicherte Karte (map_saver_cli: PGM, trinary) als OccupancyGrid-Ersatz fuer die Pruefungen der Probe."""
    import yaml
    k = yaml.safe_load(open(yaml_pfad))
    with open(os.path.join(os.path.dirname(yaml_pfad), k["image"]), "rb") as f:
        kopf = []
        while len(kopf) < 4:                      # P5, Breite, Hoehe, Maximalwert (Kommentarzeilen ueberspringen)
            zeile = f.readline().split(b"#")[0].split()
            kopf += zeile
        breite, hoehe = int(kopf[1]), int(kopf[2])
        bild = np.frombuffer(f.read(breite * hoehe), dtype=np.uint8).reshape(hoehe, breite)
    belegt = (255 - bild.astype(float)) / 255.0 > float(k["occupied_thresh"])
    daten = np.where(belegt, 100, 0)[::-1].flatten()     # PGM: Zeile 0 oben; OccupancyGrid: Zeile 0 unten
    info = SimpleNamespace(resolution=float(k["resolution"]), width=breite, height=hoehe,
                           origin=SimpleNamespace(position=SimpleNamespace(x=k["origin"][0], y=k["origin"][1])))
    return SimpleNamespace(info=info, data=daten.tolist())


def log_pruefen(log, praefix="S3"):
    txt = open(log, errors="replace").read()
    check(f"{praefix} keine MuJoCo-Instabilitaet", "simulation is unstable" not in txt)
    check(f"{praefix} keine Tracebacks", "Traceback" not in txt)


def ergebnis():
    fails = [x for x in R if not x[1]]
    print(f"\n== {len(R)-len(fails)}/{len(R)} bestanden")
    for n, _, dd in fails:
        print("   FAIL:", n, dd)
    return fails


def main_aufnehmen():
    """Wie in echt: erst die Welt mit SLAM abfahren und die Karte speichern, dann neu starten und nur mit dieser
    Karte arbeiten. (Die Karten der Welten aus der Szene bleiben fuer --karte bestehen.)"""
    global AUF
    d = tempfile.mkdtemp(prefix="imission_aufnehmen_")
    basis = Aufnahme.ordner_aus(sys.argv)
    karte = os.path.join(d, "karte_aufgenommen")
    rclpy.init(); pr = Probe()
    try:
        # ---- Phase 1: Erkundung mit SLAM ----
        AUF = Aufnahme(basis and os.path.join(basis, "1_erkundung"), "Karte aufnehmen (1/2): Erkundung mit SLAM",
                       **lauf_info(slam=True))
        log = os.path.join(d, "launch_erkundung.log")
        proc = launch(log, ["slam:=true"])
        try:
            AUF.schritt(f"== E0 Phase 1: Erkundung mit SLAM, keine Karte vorhanden (Log {log})")
            system_bereit(pr, "E0")
            AUF.schritt("== E1 Welt abfahren (Mission), slam_toolbox baut die Karte")
            mission_durchfahren(pr, log, "E1", tol_b=1.0)
            pr.spin(8)
            check("E1 /map von slam_toolbox", pr.map is not None)
            AUF.schritt("== E2 Karte speichern (map_saver_cli) und pruefen")
            r = subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", karte,
                                "--ros-args", "-p", "save_map_timeout:=20.0"], capture_output=True, text=True, timeout=90)
            ok = r.returncode == 0 and os.path.exists(karte + ".yaml") and os.path.exists(karte + ".pgm")
            check("E2 Karte gespeichert (karte_aufgenommen.yaml/.pgm)", ok, r.stderr.strip().splitlines()[-1][:120]
                  if (not ok and r.stderr.strip()) else karte + ".yaml")
            if ok:
                pr.map = lade_karte(karte + ".yaml")
                karte_pruefen(pr, "E2")
            log_pruefen(log, "E3")
        finally:
            stop(proc)
            subprocess.run(["fastdds", "shm", "clean"], capture_output=True)
            AUF.speichern(sum(1 for x in R if x[1]), len(R))
        if not os.path.exists(karte + ".yaml"):
            return
        n_phase1 = len(R)

        # ---- Phase 2: Betrieb nur mit der gespeicherten Karte ----
        AUF = Aufnahme(basis and os.path.join(basis, "2_betrieb"),
                       "Karte aufnehmen (2/2): Mission auf der aufgenommenen Karte (AMCL)",
                       **lauf_info(karte=karte + ".yaml"))
        if AUF.ordner:
            for endung in (".yaml", ".pgm"):          # fuer das Testvideo (Kartenanzeige)
                shutil.copy(karte + endung, AUF.ordner)
        pr.map, pr.tf_map_odom, pr.min_clear, pr.status, pr.p = None, False, 9.0, [], None
        log = os.path.join(d, "launch_betrieb.log")
        proc = launch(log, [f"karte:={karte}.yaml"])
        try:
            AUF.schritt("== B0 Phase 2: Neustart, nur die gespeicherte Karte (map_server + AMCL, kein SLAM)")
            system_bereit(pr, "B0")
            txt = open(log, errors="replace").read()
            check("B0 map_server laedt die aufgenommene Karte", "karte_aufgenommen.yaml" in txt)
            check("B0 kein SLAM in Phase 2", "slam_toolbox" not in txt)
            AUF.schritt("== B1 Mission auf der aufgenommenen Karte")
            mission_durchfahren(pr, log, "B1", tol_b=1.0)
            pr.spin(3)
            check("B1 /map von map_server", pr.map is not None)
            check("B1 TF map->odom von AMCL", pr.tf_map_odom)
            log_pruefen(log, "B2")
        finally:
            stop(proc)
            AUF.speichern(sum(1 for x in R[n_phase1:] if x[1]), len(R) - n_phase1)
    finally:
        pr.destroy_node(); rclpy.shutdown()


def main():
    global AUF
    if AUFNEHMEN:
        main_aufnehmen()
        sys.exit(1 if ergebnis() else 0)
    AUF = Aufnahme(Aufnahme.ordner_aus(sys.argv), TITEL,
                   **lauf_info(slam=SLAM, karte="welt" if KARTE else None, lidar="2d" if NUR_2D else "3d"))
    d = tempfile.mkdtemp(prefix="imission_")
    log = os.path.join(d, "launch.log")
    rclpy.init(); pr = Probe()
    proc = launch(log)
    try:
        print(f"== Pipeline + Layer 1 gestartet, Log {log}")
        system_bereit(pr)

        AUF.schritt("== S1 Mission komplett: herd, schrank, mikrowelle besuchen, am tisch balancieren und warten"
                    if KUECHE else "== S1 Mission komplett: start -> a -> b, balanciere b, warte b")
        mission_durchfahren(pr, log, tol_b=1.0 if SLAM else 0.6)   # SLAM: Ziele im map-Frame, Scan-Matching-Versatz

        if SLAM:
            AUF.schritt("== S1b SLAM: /map und map->odom")
            pr.spin(8)
            check("S1b /map empfangen", pr.map is not None)
            check("S1b TF map->odom von slam_toolbox", pr.tf_map_odom)
            karte_pruefen(pr, "S1b")

        if KARTE:
            AUF.schritt("== S1c feste Karte: map_server + AMCL, Wegkosten ueber Nav2")
            pr.spin(3)
            check("S1c /map von map_server", pr.map is not None)
            check("S1c TF map->odom von AMCL", pr.tf_map_odom)
            txt = open(log, errors="replace").read()
            check("S1c AMCL laeuft", "amcl" in txt and "Mission startet" in txt)
            # Luftlinie: start->a 335 + a->b 410 + balanciere 100 + warte 100 = 945. Mit Wegkosten ueber den Nav2-Planer
            # (um Hindernis und Palette, durch die Tuer) muss der erste Plan teurer sein.
            import re
            kosten = [int(k) for k in re.findall(r"Plan \((\d+)\)", txt)]
            check("S1c Wegkosten vom Nav2-Planer (Plankosten > Luftlinie 945)", bool(kosten) and kosten[0] > 945,
                  f"Plankosten={kosten[:1]}")

        AUF.schritt("== S2 NotAus waehrend der Mission")
        # Roboter ist bei b; Mission erneut starten: Ziele sind bezogen auf start_ort 'start' -> er muss laufen
        r, n0 = pr.start_mission()
        check("S2 zweiter Start akzeptiert", r.success, r.message)
        pr.spin(8)
        AUF.schritt("== S2 NOT-AUS ausgeloest")
        pr.not_aus.publish(Bool(data=True))
        end = pr.wait_end(40)
        check("S2 Status abgebrochen", end is not None and end.get("status") == "abgebrochen", str(end))
        pr.spin(3)                                   # Bremsphase aus voller Fahrt abwarten
        px = pr.p; pr.spin(5)
        dxy = float(np.hypot(pr.p[0] - px[0], pr.p[1] - px[1]))
        check("S2 Roboter steht nach NotAus (nach Bremsphase < 0.1 m in 5 s)", pr.p[2] > 0.7 and dxy < 0.1,
              f"z={pr.p[2]:.3f}, Strecke={dxy:.3f} m")
        r, _ = pr.start_mission()
        check("S2 Start bei aktivem NotAus abgelehnt", not r.success, r.message)
        r = pr.starte_mission("beispiel")
        check("S2 Start per Name (/mission/starte) bei NotAus ebenfalls abgelehnt", not r.erfolg, r.meldung)
        pr.not_aus.publish(Bool(data=False)); pr.spin(1)
        r = pr.starte_mission("gibt_es_nicht")
        check("S2 unbekannte Mission per Name abgelehnt (Meldung statt Absturz)", not r.erfolg and "nicht ladbar" in r.meldung,
              r.meldung[:80])

        log_pruefen(log)
    finally:
        pr.destroy_node(); rclpy.shutdown(); stop(proc)
        AUF.speichern(sum(1 for x in R if x[1]), len(R))
    sys.exit(1 if ergebnis() else 0)


if __name__ == "__main__":
    main()
