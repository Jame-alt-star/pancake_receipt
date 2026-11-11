#!/usr/bin/env python3
"""Systemtest Greifen Phase 1 (Layer 1 + 3 + 4, Nav2-Testdouble): Mission 'greiftisch' holt die Kiste vom Lagertisch
und legt sie am Arbeitsplatz ab.

Aufruf: python3 tests/system/integration_greifen.py [--nav2] [--timeout 400]   (ROS 2 + Workspace gesourct)
Prueft: Mission endet mit 'erfolg', die Kiste liegt am Ende an der Ablagestelle (Abstand aus /objekte/*),
Roboter nicht gestuerzt (Beckenhoehe aus /base_position).
"""
import json
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

R = []


def check(name, ok, detail=""):
    R.append(bool(ok))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}  {detail}", flush=True)


class Probe(Node):
    def __init__(self):
        super().__init__("greifen_probe")
        lat50 = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status, self.pos, self.z_min = [], None, 9.0
        self.objekte = {}
        self.verlauf = []          # (t, Kiste rel. Becken x/y/z, Basis x/y) etwa jede Sekunde
        self._t_letzt = 0.0
        self.create_subscription(String, "/mission/status", lambda m: self.status.append(json.loads(m.data)), lat50)
        self.create_subscription(Point, "/base_position", self._pos, 10)
        for n in ("kiste", "ablage_arbeitsplatz"):
            self.create_subscription(PoseStamped, f"/objekte/{n}", lambda m, n=n: self._objekt(n, m), 10)

    def _objekt(self, n, m):
        self.objekte[n] = m
        t = time.monotonic()
        if n == "kiste" and t - self._t_letzt >= 1.0 and self.pos is not None:
            self._t_letzt = t
            p = m.pose.position
            self.verlauf.append((round(t - self.t0, 1), round(p.x, 2), round(p.y, 2), round(p.z, 2),
                                 round(self.pos[0], 2), round(self.pos[1], 2)))

    def _pos(self, m):
        self.pos = (m.x, m.y, m.z)
        self.z_min = min(self.z_min, m.z)

    def ende(self):
        return next((s for s in self.status if s.get("phase") == "ende"), None)


def main():
    timeout = float(sys.argv[sys.argv.index("--timeout") + 1]) if "--timeout" in sys.argv else 400.0
    nav = "nav2" if "--nav2" in sys.argv else "testdouble"
    cmd = ["ros2", "launch", "roboter_system", "system.launch.py", "mission:=greiftisch", f"navigation:={nav}",
           "auto_start:=true", "headless:=true"]
    if nav == "nav2":
        cmd.append("karte:=true")
    launch = subprocess.Popen(cmd, stdout=open("/tmp/greifen_launch.log", "w"), stderr=subprocess.STDOUT,
                              start_new_session=True)
    rclpy.init()
    probe = Probe()
    t0 = probe.t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < timeout and probe.ende() is None and launch.poll() is None:
            rclpy.spin_once(probe, timeout_sec=0.2)
        for _ in range(25):                       # letzte Posen einsammeln
            rclpy.spin_once(probe, timeout_sec=0.1)
        ende = probe.ende()
        print("Verlauf (t, Kiste rel. Becken x y z, Becken x y):", flush=True)
        for z in probe.verlauf:
            print("  ", z, flush=True)
        print(f"Dauer {time.monotonic() - t0:.0f} s, Mission: {ende}", flush=True)
        check("Mission endet", ende is not None)
        check("Mission erfolgreich", ende is not None and ende.get("status") == "erfolg",
              str(ende.get("meldung") if ende else ""))
        check("Roboter nicht gestuerzt", probe.z_min > 0.5, f"min. Beckenhoehe {probe.z_min:.2f} m")
        k, a = probe.objekte.get("kiste"), probe.objekte.get("ablage_arbeitsplatz")
        if k is not None and a is not None:
            d = np.array([k.pose.position.x - a.pose.position.x, k.pose.position.y - a.pose.position.y,
                          k.pose.position.z - a.pose.position.z])
            check("Kiste an der Ablagestelle", np.hypot(d[0], d[1]) < 0.06 and abs(d[2]) < 0.03,
                  f"Abweichung {np.round(d * 100, 1)} cm")
        else:
            check("Kiste und Ablagestelle gesehen", False, "keine /objekte-Nachrichten")
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(launch.pid, signal.SIGINT)
            launch.wait(timeout=20)
        except Exception:
            os.killpg(launch.pid, signal.SIGKILL)
        subprocess.run(["bash", os.path.join(os.path.dirname(__file__), "stop_all.sh")], check=False)
    ok = all(R)
    print("ERGEBNIS:", "BESTANDEN" if ok else "FEHLGESCHLAGEN", flush=True)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
