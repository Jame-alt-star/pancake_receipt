#!/usr/bin/env python3
"""Integrationstest Layer 1 + 3 + 4 in der Simulation (Nav2-TESTDOUBLE statt Layer 2).

Aufruf: python3 tests/integration_mission.py [--nav2]   (ROS2 + Workspace gesourct, Display noetig)
        --nav2: echtes Nav2 aus roboter_navigation (ros-jazzy-navigation2 muss installiert sein)
Szenarien: (1) Mission laeuft komplett durch, (2) NotAus waehrend der Mission bricht ab.
"""
import json, os, signal, subprocess, sys, tempfile, time
import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

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
        self.start = self.create_client(Trigger, "/mission/start")

    def _on_pos(self, m):
        self.p = (m.x, m.y, m.z)
        for x0, x1, y0, y1 in BOXEN:
            dx = max(x0 - m.x, 0.0, m.x - x1)
            dy = max(y0 - m.y, 0.0, m.y - y1)
            self.min_clear = min(self.min_clear, float(np.hypot(dx, dy)))

    def spin(self, s):
        e = time.time() + s
        while time.time() < e:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_end(self, timeout):
        e = time.time() + timeout
        while time.time() < e:
            self.spin(0.2)
            if self.status and self.status[-1].get("phase") == "ende":
                return self.status[-1]
        return None

    def start_mission(self):
        n0 = len(self.status)
        f = self.start.call_async(Trigger.Request())
        while not f.done():
            self.spin(0.05)
        return f.result(), n0


NAV2 = "--nav2" in sys.argv     # echtes Nav2 (roboter_navigation) statt Testdouble
WELT = "--welt" in sys.argv     # Testwelt mit Waenden/Hindernis + Lidar (impliziert --nav2)
if WELT:
    NAV2 = True
# Hindernisse der Testwelt als Rechtecke (xmin, xmax, ymin, ymax); Roboter ~0.25 m Radius
BOXEN = [(1.25, 1.75, -0.8, 0.8), (4.45, 4.55, 0.8, 3.0), (4.45, 4.55, -3.0, -0.8)]


def launch(logfile):
    env = dict(os.environ)
    nav = ["nav2:=true", "headless:=true"] if NAV2 else ["nav_testdouble:=true"]   # Nav2 + Viewer: zu wenig RAM
    if WELT:
        share = subprocess.check_output(["ros2", "pkg", "prefix", "roboter_task_planning"], text=True).strip()
        nav += ["scene_file:=models/robots/g1_description/scene_testwelt_29dof.xml",
                f"mission_file:={share}/share/roboter_task_planning/missions/testwelt.yaml"]
    return subprocess.Popen(["ros2", "launch", "roboter_task_planning", "mission.launch.py",
                             *nav, "config_file:=g1_policy_29dof.yaml"],
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
        e = time.time() + 90
        while time.time() < e and not (pr.p and pr.start.service_is_ready()):
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
            check("S1 Status erfolg", end.get("status") == "erfolg", str(end))
        plans = [s for s in pr.status[n0:] if s.get("phase") == "plan"]
        if WELT:
            check("S1 kein Kontakt mit Hindernis/Waenden (Abstand > 0.2 m)", pr.min_clear > 0.2,
                  f"min_abstand={pr.min_clear:.2f} m")
        else:
            check("S1 Plan besucht a vor b", bool(plans) and "gehe_zu(start, a)" in plans[0]["plan"]
                  and plans[0]["plan"].index("gehe_zu(start, a)") < plans[0]["plan"].index("gehe_zu(a, b)"),
                  str(plans[0]["plan"]) if plans else "")
        goal = (5.8, 1.5) if WELT else (3.0, 2.0)
        dist_b = float(np.hypot(pr.p[0] - goal[0], pr.p[1] - goal[1]))
        check("S1 Roboter nahe b (<0.6 m)", dist_b < 0.6, f"abstand={dist_b:.2f}")
        check("S1 steht (z>0.7)", pr.p[2] > 0.7, f"z={pr.p[2]:.3f}")

        print("== S2 NotAus waehrend der Mission")
        # Roboter ist bei b; Mission erneut starten: Ziele sind bezogen auf start_ort 'start' -> er muss laufen
        r, n0 = pr.start_mission()
        check("S2 zweiter Start akzeptiert", r.success, r.message)
        pr.spin(8)
        pr.not_aus.publish(Bool(data=True))
        end = pr.wait_end(40)
        check("S2 Status abgebrochen", end is not None and end.get("status") == "abgebrochen", str(end))
        px = pr.p; pr.spin(6)
        check("S2 Roboter steht nach NotAus", pr.p[2] > 0.7 and np.hypot(pr.p[0]-px[0], pr.p[1]-px[1]) < 0.1,
              f"z={pr.p[2]:.3f}")
        r, _ = pr.start_mission()
        check("S2 Start bei aktivem NotAus abgelehnt", not r.success, r.message)

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
