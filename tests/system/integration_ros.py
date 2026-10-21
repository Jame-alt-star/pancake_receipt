#!/usr/bin/env python3
"""Integrationstest der echten ROS2-Pipeline (mujoco_node + steuereinheit_node).

Voraussetzung: ROS2 und Workspace gesourct, WSLg/Display fuer das MuJoCo-Fenster.
Aufruf:   python3 tests/system/integration_ros.py [--realistisch] [--beinodometrie] [--headless] [--keep-log DIR]
          [--aufnahme DIR]   (Pose, Schritte und Ergebnisse fuer das Testvideo, testaufnahme.py)

Startet die Pipeline selbst (ros2 launch), fuehrt die Szenarien nacheinander in
EINEM persistenten ROS-Prozess aus (viele kurzlebige ros2-CLI-Aufrufe stoeren die
Simulation) und beendet die Pipeline am Ende. Exit-Code 0 = alle bestanden.
"""
import argparse, os, signal, subprocess, sys, tempfile, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Bool

from roboter_beschreibung.geometrie import yaw_aus_quat
from testaufnahme import Aufnahme

RESULTS = []
AUF = Aufnahme(None, "")        # in main gesetzt


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    AUF.check(name, ok, detail)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}  {detail}", flush=True)


class Probe(Node):
    def __init__(self):
        super().__init__("integration_probe")
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.hand = self.create_publisher(Twist, "/cmd_vel/hand", 10)
        q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.not_aus = self.create_publisher(Bool, "/not_aus", q)
        self.freigabe_pub = self.create_publisher(Bool, "/freigabe", 10)
        self.p = None; self.quat = None; self.dq = None
        self.odom = None; self.tf_ok = False
        self.odom_vx = []        # twist.linear.x aller /odom-Nachrichten (fuer Auswertungen ueber eine Phase)
        self.sim_zeit = None     # Simulationszeit aus /clock [s]
        # /clock und /joint_states nur abonnieren, solange sie gebraucht werden (S3 bzw. S5): die Sonde soll das System
        # moeglichst wenig belasten - mit Viewer-Fenster unter WSL ist die CPU-Reserve knapp (siehe /tf unten).
        self._clock_abo = self._gelenk_abo = None
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        # /tf nur bis odom->base_link einmal gesehen ist: seit robot_state_publisher laufen dort 100x/s die Transformationen
        # aller ~35 Glieder; die Sonde packte sie dauerhaft in Python aus - unnoetige Last neben der Simulation mit Viewer.
        self._tf_abo = self.create_subscription(TFMessage, "/tf", self._tf, 10)
        self.create_subscription(Point, "/base_position", lambda m: setattr(self, "p", (m.x, m.y, m.z)), 10)
        self.create_subscription(Imu, "/imu", lambda m: setattr(self, "quat", (
            m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z)), 10)

    def uhr_abo(self, an: bool):
        if an and self._clock_abo is None:
            self._clock_abo = self.create_subscription(
                Clock, "/clock", lambda m: setattr(self, "sim_zeit", m.clock.sec + m.clock.nanosec * 1e-9), 10)
        elif not an and self._clock_abo is not None:
            self.destroy_subscription(self._clock_abo); self._clock_abo = None

    def gelenk_abo(self, an: bool):
        if an and self._gelenk_abo is None:
            self._gelenk_abo = self.create_subscription(
                JointState, "/joint_states", lambda m: setattr(self, "dq", np.abs(np.array(m.velocity[:12])).mean()), 10)
        elif not an and self._gelenk_abo is not None:
            self.destroy_subscription(self._gelenk_abo); self._gelenk_abo = None

    def _odom(self, m):
        self.odom = m
        self.odom_vx.append(m.twist.twist.linear.x)

    def _tf(self, m):
        for t in m.transforms:
            if t.header.frame_id == "odom" and t.child_frame_id == "base_link" and not self.tf_ok:
                self.tf_ok = True
                self.destroy_subscription(self._tf_abo)
                return

    def spin(self, secs):
        # Monotone Uhr: die Systemzeit der WSL springt alle ~35 s um 2-4 s vor (Hyper-V stellt sie nach, 21.10.2026);
        # mit time.time() wurde eine Testphase dann um diese Zeit kuerzer (S7b: 0,99 statt ~1,6 m).
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def yaw(self):
        return float(np.degrees(yaw_aus_quat(self.quat)))

    def freigabe(self, frei=True):
        self.freigabe_pub.publish(Bool(data=bool(frei)))

    def spin_mit_cmd(self, secs, vx, pub=None):
        """spin, dabei /cmd_vel (oder pub) mit 10 Hz senden (wie Nav2/Joystick)."""
        end = time.monotonic() + secs
        while time.monotonic() < end:
            self.twist(vx, pub=pub)
            self.spin(0.1)

    def twist(self, vx=0.0, vy=0.0, wz=0.0, pub=None):
        m = Twist(); m.linear.x = float(vx); m.linear.y = float(vy); m.angular.z = float(wz)
        (pub or self.cmd).publish(m)

    def wait_data(self, timeout=60):
        end = time.monotonic() + timeout
        while time.monotonic() < end and (self.p is None or self.quat is None):
            self.spin(0.1)
        return self.p is not None


def read_log(path):
    try:
        return open(path, errors="replace").read()
    except OSError:
        return ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--keep-log", default=None)
    ap.add_argument("--realistisch", action="store_true", help="realistic_sensors:=true (Rauschen, Latenz)")
    ap.add_argument("--beinodometrie", action="store_true", help="odom_quelle:=beinodometrie (Schaetzer statt Ground-Truth)")
    ap.add_argument("--headless", action="store_true", help="Simulation ohne Viewer-Fenster")
    ap.add_argument("--aufnahme", default=None, help="Ordner fuer das Testvideo")
    a = ap.parse_args()
    global AUF
    AUF = Aufnahme(a.aufnahme, "Pipeline realistisch + Beinodometrie" if a.realistisch else "Pipeline (Layer 3 + 4)",
                   welt=None, mission=None, navigation="aus", realistisch=a.realistisch, beinodometrie=a.beinodometrie)
    has_clip = True        # 29-DOF-Policy mit Kommandobegrenzung (12-DOF entfernt)
    # Strenger Stillstand nur fuer die 29-DOF-Policy; die 12-DOF-Policy trippelt bei
    # cmd=0 dauerhaft auf der Stelle (dokumentiertes Verhalten) - dort nur "kein Sturz, kaum Drift".
    strict = has_clip
    tol = 0.05 if strict else 0.3
    logdir = a.keep_log or tempfile.mkdtemp(prefix="itest_")
    os.makedirs(logdir, exist_ok=True)
    logfile = os.path.join(logdir, "launch.log")
    proc = subprocess.Popen(["ros2", "launch", "roboter_system", "system.launch.py"]
                            + (["realistic_sensors:=true"] if a.realistisch else [])
                            + (["odom_quelle:=beinodometrie"] if a.beinodometrie else [])
                            + (["headless:=true"] if a.headless else []) + AUF.launch_args(), stdout=open(logfile, "w"),
                            stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    pr = Probe()
    try:
        print(f"== Pipeline gestartet, Log: {logfile}")
        ok = pr.wait_data(90)
        check("S0 Sensordaten kommen an", ok)
        if not ok:
            raise SystemExit(1)

        AUF.schritt("== S1 Start: Stehen in Bereitschaft ohne Kommando (Startrennen)")
        pr.spin(10); pb = pr.p; yb = pr.yaw(); pr.spin(30)
        check("S1 steht nach Hochfahren (z>0.7)", pr.p[2] > 0.7, f"z={pr.p[2]:.3f}")
        drift = float(np.hypot(pr.p[0] - pb[0], pr.p[1] - pb[1]))
        check("S1 Drift in Bereitschaft < 5 cm in 30 s", drift < 0.05, f"drift={drift*100:.1f} cm, dyaw={pr.yaw()-yb:.1f}")
        check("S1 Regler verbunden/Physik gestartet", "Regler verbunden" in read_log(logfile))

        AUF.schritt("== S1b Odometrie fuer Nav2 (/odom, TF odom->base_link)")
        check("S1b /odom kommt an", pr.odom is not None)
        check("S1b TF odom->base_link", pr.tf_ok)
        if pr.odom is not None:
            dpos = float(np.hypot(pr.odom.pose.pose.position.x - pr.p[0], pr.odom.pose.pose.position.y - pr.p[1]))
            check("S1b /odom-Pose passt zu /base_position (<5 cm)", dpos < 0.05, f"d={dpos*100:.1f} cm")
            check("S1b Frames odom/base_link", pr.odom.header.frame_id == "odom" and pr.odom.child_frame_id == "base_link")

        AUF.schritt("== S2 Freigabe + Stehen")
        pr.twist(0.4); pr.spin(4)
        check("S2a /cmd_vel ohne Freigabe bewegt den Roboter nicht", pr.p[2] > 0.7 and "Bewegung freigegeben" not in read_log(logfile))
        pr.twist(0, 0, 0); pr.freigabe(True); pr.spin(8)
        check("S2 steht nach Freigabe", pr.p[2] > 0.7, f"z={pr.p[2]:.3f}")
        check("S2 Zustand Balancieren", "Bereitschaft -> Balancieren" in read_log(logfile))

        AUF.schritt("== S2b Positionshaltung: 30 s Stehen ohne Wegdriften")
        pp = pr.p; pr.spin(30)
        drift = float(np.hypot(pr.p[0] - pp[0], pr.p[1] - pp[1]))
        check("S2b Drift im Stand < 5 cm in 30 s", drift < 0.05, f"drift={drift*100:.1f} cm")

        AUF.schritt("== S3 Gehen 0.4 m/s, 12 s (Kurshaltung)")
        pr.uhr_abo(True); pr.spin(1)
        p0 = pr.p; y0 = pr.yaw(); pr.twist(0.4); t_sim0, t_wand0 = pr.sim_zeit, time.monotonic()
        pr.spin(6); pr.odom_vx.clear(); pr.spin(6)          # Geschwindigkeit ueber die zweite Haelfte auswerten
        dx = float(np.hypot(pr.p[0] - p0[0], pr.p[1] - p0[1])); dyaw = pr.yaw() - y0
        dt_sim = (pr.sim_zeit - t_sim0) if (pr.sim_zeit is not None and t_sim0 is not None) else float("nan")
        echtzeit = dt_sim / (time.monotonic() - t_wand0)
        pr.uhr_abo(False)
        check("S3 kein Sturz", pr.p[2] > 0.7, f"z={pr.p[2]:.3f}")
        # In SIMULATIONSZEIT gemessen (Review 2): laeuft die Simulation unter Last langsamer als Echtzeit, legt der
        # Roboter in 12 s Wanduhr weniger Strecke zurueck, ohne langsamer zu gehen (beobachtet: 2.85 statt ~4.1 m).
        v_sim = dx / dt_sim if dt_sim > 0 else 0.0
        check("S3 Gehgeschwindigkeit >= 0.25 m/s (Simulationszeit)", v_sim >= 0.25,
              f"v={v_sim:.2f} m/s, dx={dx:.2f} m in {dt_sim:.1f} s Simzeit, Echtzeitfaktor {echtzeit:.2f}")
        check("S3 geradeaus (|dyaw|<8 Grad)", abs(dyaw) < 8, f"dyaw={dyaw:.1f}")
        check("S3 Zustand Gehen", "-> Gehen" in read_log(logfile))
        # Median ueber 6 s statt eines Einzelwerts: am 14.10.2026 lag ein einzelner Wert der Beinodometrie bei
        # 0.05 m/s; in 2400 Werten gleichmaessigen Gehens (Diagnose) war das Minimum 0.28. Das Minimum wird
        # mit ausgegeben, damit Ausreisser sichtbar bleiben.
        vx = np.array(pr.odom_vx)
        check("S3 /odom Twist linear.x > 0.2 (Koerper-Frame, Median ueber 6 s)", len(vx) > 0 and np.median(vx) > 0.2,
              f"median={np.median(vx):.2f} min={vx.min():.2f} n={len(vx)}" if len(vx) else "keine /odom-Nachricht")

        if has_clip:
            AUF.schritt("== S4 zu grosses cmd (vx=2.0, wz=1.0) -> Begrenzung")
            pr.twist(2.0, 0.0, 1.0); pr.spin(8)
            check("S4 kein Sturz", pr.p[2] > 0.7, f"z={pr.p[2]:.3f}")
            check("S4 Warnung ausserhalb Trainingsbereich", "ausserhalb des Trainingsbereichs" in read_log(logfile))

        AUF.schritt("== S5 NOT-AUS waehrend Gehen")
        pr.twist(0.4); pr.spin(6)
        AUF.ereignis("moment", text="NOT-AUS ausgeloest")
        pr.not_aus.publish(Bool(data=True)); pr.spin(6)
        pr.gelenk_abo(True); px = pr.p[0]; pr.spin(5); pr.gelenk_abo(False)
        check("S5 kein Sturz", pr.p[2] > 0.7, f"z={pr.p[2]:.3f}")
        check(f"S5 steht (x-Aenderung < {tol*100:.0f} cm in 5 s)", abs(pr.p[0] - px) < tol, f"dx={pr.p[0]-px:.3f}")
        if strict:
            check("S5 Beine still (|dq| < 0.05 rad/s)", pr.dq is not None and pr.dq < 0.05, f"dq={pr.dq:.4f}")
        check("S5 NOT-AUS im Log", "NOT-AUS ausgeloest" in read_log(logfile))

        AUF.schritt("== S5b /cmd_vel und Freigabe-Versuch WAEHREND NotAus (Review B1)")
        pr.freigabe(True)                          # muss abgelehnt werden
        px = pr.p[0]; pr.spin_mit_cmd(3, 0.4)      # automatische Quelle sendet weiter
        check("S5b Freigabe waehrend NOT-AUS abgelehnt", "abgelehnt: NOT-AUS ist aktiv" in read_log(logfile))
        check("S5b steht trotz /cmd_vel", abs(pr.p[0] - px) < tol, f"dx={pr.p[0]-px:.3f}")

        AUF.schritt("== S6 Entwarnung ohne Freigabe, /cmd_vel laeuft weiter -> bleibt stehen")
        pr.not_aus.publish(Bool(data=False)); px = pr.p[0]; pr.spin_mit_cmd(6, 0.4)
        check("S6 laeuft trotz /cmd_vel nicht von selbst an", abs(pr.p[0] - px) < tol, f"dx={pr.p[0]-px:.3f}")
        check("S6 wartet auf Freigabe", "NOT-AUS aufgehoben" in read_log(logfile))

        AUF.schritt("== S7 neue Freigabe -> Gehen, dann Stopp")
        pp = pr.p; pr.freigabe(True); pr.spin_mit_cmd(8, 0.4)
        dist = float(np.hypot(pr.p[0] - pp[0], pr.p[1] - pp[1]))   # Richtung egal (S4 hat gedreht)
        check("S7 geht wieder (>1.5 m)", dist > 1.5, f"dist={dist:.2f}")
        AUF.ereignis("moment", text="Stopp")
        pr.twist(0, 0, 0); pr.spin(8); px = pr.p[0]; pr.spin(4)
        check("S7 steht nach Stopp", pr.p[2] > 0.7 and abs(pr.p[0] - px) < tol, f"z={pr.p[2]:.3f}")

        AUF.schritt("== S7b Handsteuerung (/cmd_vel/hand) hat Vorrang vor /cmd_vel, danach gilt wieder /cmd_vel (= 0)")
        pp = pr.p; pr.spin_mit_cmd(5, 0.4, pub=pr.hand)
        dist = float(np.hypot(pr.p[0] - pp[0], pr.p[1] - pp[1]))
        check("S7b geht mit Handbefehl, obwohl /cmd_vel 0 ist (>0.8 m)", dist > 0.8, f"dist={dist:.2f}")
        pr.spin(8); pp = pr.p; pr.spin(4)
        dist = float(np.hypot(pr.p[0] - pp[0], pr.p[1] - pp[1]))
        check("S7b steht wieder, sobald die Hand nicht mehr sendet", pr.p[2] > 0.7 and dist < tol,
              f"z={pr.p[2]:.3f} d={dist:.3f}")

        if pr.odom is not None:
            o = pr.odom.pose.pose.position
            err = float(np.hypot(o.x - pr.p[0], o.y - pr.p[1]))
            check("S8 /odom stimmt mit wahrer Position ueberein (< 0.5 m nach allen Szenarien, ~3 % der Strecke)", err < 0.5,
                  f"Fehler={err*100:.1f} cm")
        log = read_log(logfile)
        check("S8 keine MuJoCo-Instabilitaetswarnung", "simulation is unstable" not in log)
        check("S8 keine Python-Tracebacks", "Traceback" not in log)
        check("S8 Sprunglimit griff nie (Normalbetrieb)", "Zielsprung begrenzt" not in log)

        AUF.schritt("== S9 Steuerung faellt aus -> Simulation daempft (Review 2)")
        pids = subprocess.run(["pgrep", "-f", "lib/roboter_steuerung/steuereinheit_node"], capture_output=True,
                              text=True).stdout.split()
        check("S9 Steuerknoten gefunden", len(pids) == 1, f"pids={pids}")
        AUF.ereignis("moment", text="Steuerung beendet")
        for pid in pids:
            # SIGINT statt SIGKILL: fuer die Simulation zaehlt nur, dass keine Kommandos mehr kommen. Ein per SIGKILL
            # beendeter Knoten hinterlaesst gesperrte Fast-DDS-Speichersegmente (/dev/shm), die nachfolgende Laeufe
            # ausbremsten (Parameter kamen erst nach ~14 s an, IMU-Nachrichten blieben aus).
            os.kill(int(pid), signal.SIGINT)
        pr.spin(3)
        check("S9 Simulation meldet Daempfungsmodus", "Daempfungsmodus" in read_log(logfile))
    finally:
        pr.destroy_node(); rclpy.shutdown()
        try:
            os.killpg(proc.pid, signal.SIGINT); proc.wait(timeout=15)
        except Exception:
            try: os.killpg(proc.pid, signal.SIGKILL)
            except Exception: pass
        AUF.speichern(sum(1 for r in RESULTS if r[1]), len(RESULTS))

    fails = [r for r in RESULTS if not r[1]]
    print(f"\n== {len(RESULTS)-len(fails)}/{len(RESULTS)} bestanden")
    for n, _, d in fails:
        print(f"   FAIL: {n} {d}")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
