#!/usr/bin/env python3
"""Integrationstest cmd_vel-Watchdog (Layer 3): ein einzelnes /cmd_vel darf den Roboter nicht
dauerhaft laufen lassen. Aufruf: python3 tests/integration_watchdog.py"""
import os, signal, subprocess, sys, tempfile, time
import numpy as np, rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from std_msgs.msg import Bool

R = []


def check(n, ok, d=""):
    R.append((n, bool(ok), d)); print(f"  [{'PASS' if ok else 'FAIL'}] {n}  {d}", flush=True)


def main():
    log = os.path.join(tempfile.mkdtemp(prefix="iwd_"), "launch.log")
    proc = subprocess.Popen(["ros2", "launch", "roboter_steuerung", "steuereinheit.launch.py",
                             "config_file:=g1_policy_29dof.yaml", "headless:=true", "cmd_vel_timeout_s:=1.0"],
                            stdout=open(log, "w"), stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(); n = Node("wd_probe"); pub = n.create_publisher(Twist, "/cmd_vel", 10); s = {}
    fpub = n.create_publisher(Bool, "/freigabe", 10)
    n.create_subscription(Point, "/base_position", lambda m: s.__setitem__("p", (m.x, m.y, m.z)), 10)

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(n, timeout_sec=0.02)
    try:
        while "p" not in s: spin(0.2)
        spin(25)
        m = Twist(); pub.publish(m)
        fpub.publish(Bool(data=True)); spin(4)                    # Freigabe ueber /freigabe
        p0 = s["p"]; m.linear.x = 0.4; pub.publish(m)             # EIN Kommando, danach nichts mehr
        spin(2.5); pa = s["p"]
        spin(3.0); pb = s["p"]
        spin(3.0); pc = s["p"]
        d_early = np.hypot(pa[0]-p0[0], pa[1]-p0[1])
        v_late = np.hypot(pc[0]-pb[0], pc[1]-pb[1]) / 3.0
        check("Roboter laeuft nach dem Kommando los (> 0.1 m in 2.5 s)", d_early > 0.1, f"s={d_early:.2f} m")
        check("Roboter steht nach Watchdog-Timeout (v < 0.03 m/s)", v_late < 0.03, f"v={v_late:.3f} m/s")
        check("kein Sturz", s["p"][2] > 0.7, f"z={s['p'][2]:.3f}")
        txt = open(log, errors="replace").read()
        check("Watchdog meldet", "nicht empfangen" in txt)
        check("keine Instabilitaet", "simulation is unstable" not in txt)
    finally:
        rclpy.shutdown()
        try: os.killpg(proc.pid, signal.SIGINT); proc.wait(timeout=20)
        except Exception:
            try: os.killpg(proc.pid, signal.SIGKILL)
            except Exception: pass
    f = [r for r in R if not r[1]]
    print(f"== {len(R)-len(f)}/{len(R)} bestanden")
    sys.exit(1 if f else 0)


if __name__ == "__main__":
    main()
