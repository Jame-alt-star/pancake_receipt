#!/usr/bin/env python3
"""Integrationstest Layer 1 + 3 + 4 in der Simulation (Nav2-TESTDOUBLE statt Layer 2).

Aufruf: python3 tests/system/integration_mission.py [--nav2]   (ROS2 + Workspace gesourct, Display noetig)
        --nav2: echtes Nav2 aus roboter_navigation (ros-jazzy-navigation2 muss installiert sein)
        --karte: Navigation auf der festen Karte der Welt (map_server + AMCL, Wegkosten ueber den Nav2-Planer)
Szenarien: (1) Mission laeuft komplett durch, (2) NotAus waehrend der Mission bricht ab, Start per Name abgelehnt.
"""
import json, os, signal, subprocess, sys, tempfile, time
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

R = []


def check(name, ok, detail=""):
    R.append((name, bool(ok), detail))
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
        self.create_subscription(String, "/mission/status", lambda m: self.status.append(json.loads(m.data)), lat50)
        self.not_aus = self.create_publisher(Bool, "/not_aus", lat)
        self.map = None
        self.tf_map_odom = False
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m), lat)
        self.create_subscription(TFMessage, "/tf", self._tf, 50)
        self.start = self.create_client(Trigger, "/mission/start")
        self.starte = self.create_client(MissionStarten, "/mission/starte")

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
WELT = "--welt" in sys.argv or PATROUILLE or SLAM or DREI_D or NUR_2D or KARTE   # Testwelt mit Waenden/Hindernis + Lidar (impliziert --nav2)
if WELT:
    NAV2 = True
# Hindernisse der Testwelt als Rechtecke (xmin, xmax, ymin, ymax); Roboter ~0.25 m Radius
BOXEN = [(1.25, 1.75, -0.8, 0.8), (4.45, 4.55, 0.8, 2.95), (4.45, 4.55, -2.95, -0.8)]
if DREI_D or NUR_2D:
    BOXEN = BOXEN + [(2.0, 2.8, -1.5, -0.7)]     # niedrige Palette (0.45 m)


def launch(logfile):
    env = dict(os.environ)
    nav = ["navigation:=nav2", "headless:=true"] if NAV2 else ["navigation:=testdouble"]   # Nav2 + Viewer: zu wenig RAM
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
    # Die Simulation uebernimmt die Welt aus der Missionsdatei (Schluessel welt).
    mission = ("patrouille" if PATROUILLE else ("testwelt3d" if (DREI_D or NUR_2D) else "testwelt")) if WELT else "beispiel"
    return subprocess.Popen(["ros2", "launch", "roboter_system", "system.launch.py", f"mission:={mission}", *nav],
                            stdout=open(logfile, "w"), stderr=subprocess.STDOUT, start_new_session=True, env=env)


def stop(proc):
    try:
        os.killpg(proc.pid, signal.SIGINT); proc.wait(timeout=20)
    except Exception:
        try: os.killpg(proc.pid, signal.SIGKILL)
        except Exception: pass


def main():
    d = tempfile.mkdtemp(prefix="imission_")
    log = os.path.join(d, "launch.log")
    rclpy.init(); pr = Probe()
    proc = launch(log)
    try:
        print(f"== Pipeline + Layer 1 gestartet, Log {log}")
        e = time.monotonic() + 90
        while time.monotonic() < e and not (pr.p and pr.start.service_is_ready()):
            pr.spin(0.2)
        check("S0 System bereit (Position + /mission/start)", pr.p is not None and pr.start.service_is_ready())
        pr.spin(60 if NAV2 else 25)     # Nav2-Lifecycle braucht laenger
        check("S0 Roboter steht (z>0.7)", pr.p[2] > 0.7, f"z={pr.p[2]:.3f}")

        print("== S1 Mission komplett: start -> a -> b, balanciere b, warte b")
        r, n0 = pr.start_mission()
        check("S1 Start akzeptiert", r.success, r.message)
        end = pr.wait_end(300)
        check("S1 Mission beendet", end is not None)
        if end:
            if not NUR_2D:
                check("S1 Status erfolg", end.get("status") == "erfolg", str(end))
        plans = [s for s in pr.status[n0:] if s.get("phase") == "plan"]
        if NUR_2D:
            # Gegenprobe: ohne 3D-Lidar ist die niedrige Palette unsichtbar -> Kontakt erwartet
            kontakt = pr.min_clear < 0.20
            check("S1 KONTROLLE: 2D-Lidar sieht die niedrige Palette nicht (Roboter kommt ihr zu nah)",
                  kontakt, f"min_abstand={pr.min_clear:.2f} m (Koerperhalbbreite ~0.2: darunter Kontakt)")
        elif DREI_D:
            check("S1 3D-Lidar: niedrige Palette umfahren, kein Kontakt (Abstand > 0.20 m)", pr.min_clear > 0.20,
                  f"min_abstand={pr.min_clear:.2f} m")
        elif WELT:
            check("S1 kein Kontakt mit Hindernis/Waenden (Abstand > 0.2 m)", pr.min_clear > 0.2,
                  f"min_abstand={pr.min_clear:.2f} m")
        else:
            check("S1 Plan besucht a vor b", bool(plans) and "gehe_zu(start, a)" in plans[0]["plan"]
                  and plans[0]["plan"].index("gehe_zu(start, a)") < plans[0]["plan"].index("gehe_zu(a, b)"),
                  str(plans[0]["plan"]) if plans else "")
        goal = (0.0, 0.0) if PATROUILLE else ((5.8, 1.5) if WELT else (3.0, 2.0))
        dist_b = float(np.hypot(pr.p[0] - goal[0], pr.p[1] - goal[1]))
        tol_b = 1.0 if SLAM else 0.6   # SLAM: Ziele im map-Frame, Scan-Matching-Versatz gegen die Welt
        check(f"S1 Roboter nahe b (<{tol_b} m)", dist_b < tol_b, f"abstand={dist_b:.2f}")
        check("S1 steht (z>0.7)", pr.p[2] > 0.7, f"z={pr.p[2]:.3f}")
        check("S1 Freigabe nach Missionsende entzogen (Review 2)",
              "Bewegungsfreigabe entzogen" in open(log, errors="replace").read())

        if SLAM:
            print("== S1b SLAM: /map und map->odom")
            pr.spin(8)
            check("S1b /map empfangen", pr.map is not None)
            check("S1b TF map->odom von slam_toolbox", pr.tf_map_odom)
            # bekannte Wandpunkte (Welt = map-Frame, da Start im Ursprung mit Yaw 0)
            # Fenster entlang der Waende (x0, x1, y0, y1) - mind. 6 belegte Zellen = Wand erkannt
            for name, (x0, x1, y0, y1) in {
                    "Westwand": (-1.7, -1.3, -1.5, 1.5),
                    "Suedwand": (1.0, 3.0, -3.2, -2.7), "Hindernis": (1.1, 1.9, -0.6, 0.6),
                    "Trennwand": (4.3, 4.7, 1.2, 2.6)}.items():
                n = pr.map_occupied_rect(x0, x1, y0, y1)
                check(f"S1b Karte enthaelt {name}", n >= 6, f"{n} belegte Zellen")
            # Die Nordwand liegt auf der oberen Rasterkante; slam_toolbox verwirft dort die aeusserste
            # Zeile. Geprueft wird daher nur, dass die Karte bis zur Nordwand reicht.
            top = pr.map.info.origin.position.y + pr.map.info.height * pr.map.info.resolution if pr.map else 0.0
            check("S1b Karte reicht bis zur Nordwand (y >= 2.9)", top >= 2.9, f"oberer Rand y={top:.2f}")
            check("S1b Tuer in der Karte frei", not pr.map_occupied(4.5, 0.0, 0.15))

        if KARTE:
            print("== S1c feste Karte: map_server + AMCL, Wegkosten ueber Nav2")
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

        print("== S2 NotAus waehrend der Mission")
        # Roboter ist bei b; Mission erneut starten: Ziele sind bezogen auf start_ort 'start' -> er muss laufen
        r, n0 = pr.start_mission()
        check("S2 zweiter Start akzeptiert", r.success, r.message)
        pr.spin(8)
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

        txt = open(log, errors="replace").read()
        check("S3 keine MuJoCo-Instabilitaet", "simulation is unstable" not in txt)
        check("S3 keine Tracebacks", "Traceback" not in txt)
    finally:
        pr.destroy_node(); rclpy.shutdown(); stop(proc)
    fails = [x for x in R if not x[1]]
    print(f"\n== {len(R)-len(fails)}/{len(R)} bestanden")
    for n, _, dd in fails:
        print("   FAIL:", n, dd)
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
