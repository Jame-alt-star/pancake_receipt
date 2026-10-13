"""Missions-Knoten (Layer 1): laedt eine Mission (YAML), plant mit Fast Downward und fuehrt den
Plan ueber die Bridges aus. NotAus (/not_aus) bricht die laufende Mission ab.

Schnittstellen:
  Service  /mission/start   (std_srvs/Trigger)  startet die Mission aus Parameter mission_file
  Service  /mission/abort   (std_srvs/Trigger)
  Topic    /mission/status  (std_msgs/String, JSON; TRANSIENT_LOCAL)
Parameter: mission_file, nav_backend ('nav2'|'gehaktion'), auto_start (bool)
"""
import json
import threading

import rclpy
from geometry_msgs.msg import Point
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
        self.create_subscription(Point, "/base_position", self._pos, 10, callback_group=cb)
        self.create_subscription(Bool, "/not_aus", self._not_aus, latched, callback_group=cb)
        self.create_service(Trigger, "/mission/start", self._start, callback_group=cb)
        self.create_service(Trigger, "/mission/abort", self._abort_srv, callback_group=cb)
        self._pos_last = None
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
    def _pos(self, m: Point):
        self._pos_last = (m.x, m.y)

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
        ex = MissionExecutor(FastDownwardPlanner(), self.bridges, locator=lambda: self._pos_last,
                             log=lambda m: self.get_logger().info(m), status_cb=self._publish)
        self._thread = threading.Thread(target=ex.run, args=(mission, self._abort), daemon=True)
        self._thread.start()
        return True, f"Mission gestartet ({path})"

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
