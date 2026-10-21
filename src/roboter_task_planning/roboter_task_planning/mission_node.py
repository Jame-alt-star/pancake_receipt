"""Missions-Knoten (Layer 1): laedt eine Mission (YAML), plant mit Fast Downward und fuehrt den
Plan ueber die Bridges aus. NotAus (/not_aus) bricht die laufende Mission ab.

Schnittstellen:
  Service  /mission/start   (std_srvs/Trigger)  startet die Mission aus Parameter mission_file
  Service  /mission/starte  (roboter_interfaces/MissionStarten)  startet eine Mission nach Name oder Pfad
  Service  /mission/abort   (std_srvs/Trigger)
  Topic    /mission/status  (std_msgs/String, JSON; TRANSIENT_LOCAL)
Parameter: mission_file, nav_backend ('nav2'|'gehaktion'), auto_start (bool), wegkosten_nav2 (bool: Wegkosten fuer
den Planer ueber den Nav2-Planer statt Luftlinie)

Thread-Modell: der Knoten laeuft in einem MultiThreadedExecutor (alle Callbacks reentrant). Eine
Mission laeuft in einem eigenen Thread (MissionExecutor.run blockiert, Bridges pollen ihre Action-
Futures); Executor-Threads bedienen parallel Status, Services, /not_aus und die Action-Clients.
Abbruch (Service oder NotAus) ueber ein threading.Event, das die Bridges regelmaessig pruefen.
Beim Start gibt der Knoten die Bewegung frei (/freigabe): der Missionsstart ist die Bedieneraktion. Am Ende (Erfolg,
Fehlschlag, Abbruch) entzieht er sie wieder - danach steht der Roboter in Bereitschaft, ein verirrtes /cmd_vel bewegt
ihn nicht. Meldet die Steuerung eine Stoerung (/steuerung/zustand), bricht die Mission ab.
auto_start nur mit Simulation (erkannt am /clock-Sender): auf echter Hardware wuerde der Roboter damit ohne
Bedieneraktion losfahren, dort wird er abgelehnt.
"""
import json
import threading
import time

import os

import rclpy
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from rclpy.time import Time
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from roboter_interfaces.srv import MissionStarten
from roboter_task_planning.deliberative.pddl_planner import FastDownwardPlanner
from roboter_task_planning.executive.action_bridges import BalanceBridge, NavigationBridge
from roboter_task_planning.executive.action_dispatcher import MissionExecutor
from roboter_task_planning.executive.simple_bridges import WarteBridge
from roboter_task_planning.executive.world_knowledge import Mission


class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_node")
        self.declare_parameter("mission_file", "")
        self.declare_parameter("nav_backend", "nav2")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("wegkosten_nav2", True)
        cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)
        status_qos = QoSProfile(depth=50, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)
        self.pub_status = self.create_publisher(String, "/mission/status", status_qos)
        # Bewegungsfreigabe fuer Layer 3: der Start einer Mission ist die bewusste Bedieneraktion
        # (seit 14.10.2026 gibt /cmd_vel nicht mehr frei, siehe steuereinheit_node).
        self.pub_freigabe = self.create_publisher(Bool, "/freigabe", 10)
        self._cb = cb
        self.create_subscription(Bool, "/not_aus", self._not_aus, latched, callback_group=cb)
        self._stoerung = None
        self.create_subscription(String, "/steuerung/zustand", self._zustand, latched, callback_group=cb)
        self.create_service(Trigger, "/mission/start", self._start, callback_group=cb)
        self.create_service(Trigger, "/mission/abort", self._abort_srv, callback_group=cb)
        self.create_service(MissionStarten, "/mission/starte", self._starte, callback_group=cb)
        self._not_aus_aktiv = False
        self._abort = threading.Event()
        self._thread = None
        backend = self.get_parameter("nav_backend").value
        self.bridges = [NavigationBridge(self, backend), BalanceBridge(self), WarteBridge()]
        self._publish({"phase": "bereit"})
        self.get_logger().info(f"Missions-Knoten bereit (nav_backend={backend}).")
        if self.get_parameter("auto_start").value:
            self.get_logger().warn("auto_start: Mission startet ohne Bedieneraktion - nur fuer Simulation und Tests.")
            self.create_timer(1.0, self._auto_start_once, callback_group=cb)
            self._auto_done = False
            self._auto_t0 = time.monotonic()

    # ---- Eingaenge ----
    def _locate(self, timeout_s: float = 1.5):
        """Aktuelle Roboterposition im map-Frame (wie die Missionsorte), auf Abruf.

        Review U4: vorher kam die Position aus /odom (odom-Frame). Mit SLAM liegen map und odom
        bis ~0,75 m auseinander, damit waren naechster Ort und pos_aktuell falsch. Jetzt per TF
        map->base_link; nur wenn das nicht verfuegbar ist, Rueckfall auf /odom.
        Abos nur auf Abruf: ein dauerhafter TF-Listener bzw. ein Dauerabo auf 100-Hz-Topics
        kostete im Leerlauf ~30 % CPU (Python-Deserialisierung).
        """
        buf = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buf, self)
        try:
            # lookup_transform(Time()) wartet in rclpy nicht auf einen frisch angelegten Listener,
            # sondern scheitert sofort -> selbst warten, bis die Kette map->base_link vorliegt.
            t_end = time.monotonic() + timeout_s
            while not buf.can_transform("map", "base_link", Time()) and time.monotonic() < t_end:
                time.sleep(0.05)
            t = buf.lookup_transform("map", "base_link", Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception as e:      # TF fehlt (z. B. ohne Nav2) -> /odom
            self.get_logger().info(f"Position per TF map->base_link nicht verfuegbar ({type(e).__name__}), nutze /odom")
        finally:
            listener.unregister()
        got = {}
        ev = threading.Event()

        def cb(m: Odometry):
            got["p"] = (m.pose.pose.position.x, m.pose.pose.position.y)
            ev.set()
        sub = self.create_subscription(Odometry, "/odom", cb, 10, callback_group=self._cb)
        try:
            ev.wait(timeout_s)
        finally:
            self.destroy_subscription(sub)
        return got.get("p")

    def _not_aus(self, m: Bool):
        self._not_aus_aktiv = bool(m.data)
        if m.data and self._thread and self._thread.is_alive():
            self.get_logger().warn("NOT-AUS: Mission wird abgebrochen.")
            self._abort.set()

    def _zustand(self, m: String):
        self._stoerung = m.data if m.data.startswith("Stoerung") else None
        if self._stoerung and self._thread and self._thread.is_alive():
            self.get_logger().warn(f"Steuerung meldet {self._stoerung} - Mission wird abgebrochen.")
            self._abort.set()

    def _publish(self, d: dict):
        self.pub_status.publish(String(data=json.dumps(d, ensure_ascii=False)))

    # auto_start nur mit Simulation: sie sendet /clock, die echte Hardware nicht (Review 2, B9). So lange auf die
    # Simulation warten, wie sie zum Start braucht (Startreihenfolge ist frei).
    AUTO_START_WARTE_S = 30.0

    def _auto_start_once(self):
        if self._auto_done:
            return
        if self.count_publishers("/clock") == 0:
            if time.monotonic() - self._auto_t0 < self.AUTO_START_WARTE_S:
                return
            self._auto_done = True
            self.get_logger().error("auto_start abgelehnt: keine Simulation (/clock) gefunden. Ohne Simulation startet "
                                    "ein Bediener die Mission (/mission/start).")
            return
        self._auto_done = True
        ok, msg = self._start_mission()
        self.get_logger().info(f"auto_start: {msg}")

    # ---- Steuerung ----
    @staticmethod
    def mission_pfad(mission: str) -> str:
        """Name aus missions/ (mit oder ohne .yaml) oder Pfad."""
        if os.path.sep in mission:
            return mission
        name = mission if mission.endswith(".yaml") else mission + ".yaml"
        return os.path.join(get_package_share_directory("roboter_task_planning"), "missions", name)

    def _start_mission(self, path: str = ""):
        if self._thread and self._thread.is_alive():
            return False, "Mission laeuft bereits"
        if self._not_aus_aktiv:
            return False, "NOT-AUS aktiv - Mission nicht gestartet"
        if self._stoerung:
            return False, f"Steuerung in {self._stoerung} - erst /steuerung/quittieren"
        path = path or self.get_parameter("mission_file").value
        try:
            mission = Mission.from_yaml(path)
        except Exception as e:      # Datei fehlt, YAML/Validierung
            return False, f"Mission nicht ladbar: {e}"
        self._abort.clear()
        self.pub_freigabe.publish(Bool(data=True))
        nav = self.bridges[0]
        schaetzer = nav.weglaenge if self.get_parameter("wegkosten_nav2").value else None
        ex = MissionExecutor(FastDownwardPlanner(), self.bridges, locator=self._locate,
                             log=lambda m: self.get_logger().info(m), status_cb=self._publish, wegschaetzer=schaetzer)
        self._thread = threading.Thread(target=self._run_mission, args=(ex, mission), daemon=True)
        self._thread.start()
        return True, f"Mission gestartet ({path})"

    def _run_mission(self, ex, mission):
        """Thread-Rumpf. MissionExecutor.run faengt selbst alle Ausnahmen ab; dieser Rahmen ist die
        letzte Sicherung, damit /mission/status immer einen Endstatus bekommt (Review B3)."""
        try:
            ex.run(mission, self._abort)
        except Exception as e:
            self.get_logger().error(f"Mission abgestuerzt: {type(e).__name__}: {e}")
            self._publish({"phase": "ende", "status": "fehlgeschlagen", "meldung": f"interner Fehler: {e}"})
        finally:
            self.pub_freigabe.publish(Bool(data=False))
            self.get_logger().info("Mission beendet - Bewegungsfreigabe entzogen.")

    def _start(self, req, resp):
        resp.success, resp.message = self._start_mission()
        return resp

    def _starte(self, req, resp):
        resp.erfolg, resp.meldung = self._start_mission(self.mission_pfad(req.mission) if req.mission else "")
        return resp

    def _abort_srv(self, req, resp):
        running = bool(self._thread and self._thread.is_alive())
        self._abort.set()
        resp.success, resp.message = running, "abgebrochen" if running else "keine Mission aktiv"
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._abort.set()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
