#!/usr/bin/env python3
"""Demo-Lauf: startet die komplette Pipeline (Nav2 + SLAM, realistische Sensorik, Beinodometrie, 3D-Lidar),
fuehrt eine Mission aus, zeigt zum Schluss den NotAus und schreibt
  <ausgabe>/pose.npz         Roboterpose (aus mujoco_node, 25 Hz)
  <ausgabe>/ereignisse.json  Missionsereignisse mit Wandzeit
  <ausgabe>/lauf.json        Mission, Welt und Konfiguration (fuer render_demo.py)

Aufruf: python3 demo/run_demo.py <ausgabeordner> [--mission NAME] [--einfach]
  --mission: Mission aus roboter_task_planning/missions (Standard demo_lager, z. B. testwelt3d)
  --einfach: ohne SLAM/realistische Sensorik
"""
import json, os, signal, subprocess, sys, time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

out = sys.argv[1]
os.makedirs(out, exist_ok=True)
einfach = "--einfach" in sys.argv
mission = sys.argv[sys.argv.index("--mission") + 1] if "--mission" in sys.argv else "demo_lager"
share = subprocess.check_output(["ros2", "pkg", "prefix", "roboter_task_planning"], text=True).strip()
import yaml  # noqa: E402
welt = yaml.safe_load(open(f"{share}/share/roboter_task_planning/missions/{mission}.yaml"))["welt"]
json.dump({"mission": mission, "welt": welt, "slam": not einfach, "realistisch": not einfach, "lidar": "3d"},
          open(f"{out}/lauf.json", "w"), indent=1)
args = ["ros2", "launch", "roboter_system", "system.launch.py", f"mission:={mission}", "navigation:=nav2",
        "lidar_modus:=3d", "headless:=true",
        f"record_path:={out}/pose.npz"]
if not einfach:
    args += ["slam:=true", "realistic_sensors:=true", "odom_quelle:=beinodometrie"]
proc = subprocess.Popen(args, stdout=open(f"{out}/launch.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
events = []
rclpy.init()
n = Node("demo_regie")
lat = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String, "/mission/status", lambda m: events.append({"wall": time.time(), **json.loads(m.data)}), lat)
latq = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                  durability=DurabilityPolicy.TRANSIENT_LOCAL)
notaus = n.create_publisher(Bool, "/not_aus", latq)
start = n.create_client(Trigger, "/mission/start")
odom = {}
n.create_subscription(Odometry, "/odom", lambda m: odom.__setitem__("p", (m.pose.pose.position.x, m.pose.pose.position.y)), 10)


def spin(t):
    e = time.time() + t
    while time.time() < e:
        rclpy.spin_once(n, timeout_sec=0.02)


def note(txt):
    events.append({"wall": time.time(), "phase": "regie", "text": txt})
    print(f"[{time.strftime('%H:%M:%S')}] {txt}", flush=True)


def wait_end(timeout):
    e = time.time() + timeout
    while time.time() < e:
        spin(0.2)
        if events and events[-1].get("phase") == "ende":
            return events[-1]
    return None


try:
    while "p" not in odom or not start.service_is_ready():
        spin(0.3)
    spin(30 if not einfach else 15)          # Nav2/SLAM hochfahren, Roboter steht in Bereitschaft
    note("Akt 1: Kontrollrundgang")
    r = start.call_async(Trigger.Request())
    while not r.done():
        spin(0.05)
    note(f"Mission gestartet: {r.result().message}")
    end = wait_end(420)
    note(f"Mission beendet: {end}")
    spin(4)
    note("Akt 2: NotAus")
    r = start.call_async(Trigger.Request())
    while not r.done():
        spin(0.05)
    spin(11)
    note("NOT-AUS ausgeloest")
    notaus.publish(Bool(data=True))
    wait_end(30)
    spin(8)
    note("Ende der Aufnahme")
finally:
    json.dump(events, open(f"{out}/ereignisse.json", "w"), ensure_ascii=False, indent=1)
    rclpy.shutdown()
    try:
        os.killpg(proc.pid, signal.SIGINT); proc.wait(timeout=25)
    except Exception:
        try: os.killpg(proc.pid, signal.SIGKILL)
        except Exception: pass
