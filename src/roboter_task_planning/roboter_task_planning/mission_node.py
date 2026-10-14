"""Missions-Knoten (Layer 1): laedt eine Mission (YAML), plant mit Fast Downward und fuehrt den
Plan ueber die Bridges aus. NotAus (/not_aus) bricht die laufende Mission ab.

Schnittstellen:
  Service  /mission/start   (std_srvs/Trigger)  startet die Mission aus Parameter mission_file
  Service  /mission/abort   (std_srvs/Trigger)
  Topic    /mission/status  (std_msgs/String, JSON; TRANSIENT_LOCAL)
Parameter: mission_file, nav_backend ('nav2'|'gehaktion'), auto_start (bool)

Thread-Modell: der Knoten laeuft in einem MultiThreadedExecutor (alle Callbacks reentrant). Eine
Mission laeuft in einem eigenen Thread (MissionExecutor.run blockiert, Bridges pollen ihre Action-
Futures); Executor-Threads bedienen parallel Status, Services, /not_aus und die Action-Clients.
Abbruch (Service oder NotAus) ueber ein threading.Event, das die Bridges regelmaessig pruefen.
Beim Start gibt der Knoten die Bewegung frei (/freigabe): der Missionsstart ist die Bedieneraktion.
"""
import json
import threading
import time

import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

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
        self.create_service(Trigger, "/mission/start", self._start, callback_group=cb)
        self.create_service(Trigger, "/mission/abort", self._abort_srv, callback_group=cb)
        self._not_aus_aktiv = False
        self._abort = threading.Event()
        self._thread = None
        backend = self.get_parameter("nav_backend").value
        self.bridges = [NavigationBridge(self, backend), BalanceBridge(self), WarteBridge()]
        self._publish({"phase": "bereit"})
        self.get_logger().info(f"Missions-Knoten bereit (nav_backend={backend}).")
        if self.get_parameter("auto_start").value:
            self.create_timer(1.0, self._auto_start_once, callback_group=cb)
            self._auto_done = False

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

    def _publish(self, d: dict):
        self.pub_status.publish(String(data=json.dumps(d, ensure_ascii=False)))

    def _auto_start_once(self):
        if not self._auto_done:
            self._auto_done = True
            ok, msg = self._start_mission()
            self.get_logger().info(f"auto_start: {msg}")

    # ---- Steuerung ----
    def _start_mission(self):
        if self._thread and self._thread.is_alive():
            return False, "Mission laeuft bereits"
        if self._not_aus_aktiv:
            return False, "NOT-AUS aktiv - Mission nicht gestartet"
        path = self.get_parameter("mission_file").value
        try:
            mission = Mission.from_yaml(path)
        except Exception as e:      # Datei fehlt, YAML/Validierung
            return False, f"Mission nicht ladbar: {e}"
        self._abort.clear()
        self.pub_freigabe.publish(Bool(data=True))
        ex = MissionExecutor(FastDownwardPlanner(), self.bridges, locator=self._locate,
                             log=lambda m: self.get_logger().info(m), status_cb=self._publish)
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

    def _start(self, req, resp):
        resp.success, resp.message = self._start_mission()
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
