"""ROS2-Bridges: Dispatcher-Schritte -> Action-Clients (Layer 2 Navigation, Layer 3 Balancieren)."""
import threading
import time

from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from roboter_interfaces.action import BalancierAktion, GehAktion
from roboter_task_planning.deliberative.i_planner import PlanSchritt
from roboter_task_planning.executive.i_action_bridge import BridgeResult, IActionBridge
from roboter_task_planning.executive.world_knowledge import Mission

GEH_ACTION = "/geh_aktion"           # GehAktion (Layer 3 / Testdouble)
BALANCIER_ACTION = "/balanciere"     # BalancierAktion (balance_dienst_node)
NAV2_ACTION = "navigate_to_pose"     # nav2_msgs/NavigateToPose (Layer 2, Kollege)
NAV2_ACTIVE_SRV = "/lifecycle_manager_navigation/is_active"   # std_srvs/Trigger


def run_action(client: ActionClient, goal, cancel: threading.Event, timeout_s: float,
               server_wait_s: float = 5.0):
    """Sendet ein Goal und wartet blockierend (Executor laeuft in einem anderen Thread).
    Rueckgabe: (status_ok, result_msg_or_None, text)."""
    if not client.wait_for_server(timeout_sec=server_wait_s):
        return False, None, "Action-Server nicht erreichbar"
    gf = client.send_goal_async(goal)
    t0 = time.monotonic()
    while not gf.done():
        if cancel.is_set() or time.monotonic() - t0 > 10.0:
            return False, None, "abgebrochen" if cancel.is_set() else "Goal nicht angenommen"
        time.sleep(0.02)
    gh = gf.result()
    if not gh.accepted:
        return False, None, "Goal abgelehnt"
    rf = gh.get_result_async()
    t0 = time.monotonic()
    while not rf.done():
        if cancel.is_set() or time.monotonic() - t0 > timeout_s:
            gh.cancel_goal_async()
            t1 = time.monotonic()
            while not rf.done() and time.monotonic() - t1 < 5.0:
                time.sleep(0.02)
            return False, None, "abgebrochen" if cancel.is_set() else f"Zeitueberschreitung {timeout_s:.0f} s"
        time.sleep(0.02)
    r = rf.result()
    from action_msgs.msg import GoalStatus
    return r.status == GoalStatus.STATUS_SUCCEEDED, r.result, f"Status {r.status}"


class NavigationBridge(IActionBridge):
    """'gehe_zu(von, nach)': Ziel = Koordinaten des Ortes 'nach'.
    backend='gehaktion': roboter_interfaces/GehAktion (Layer 3 bzw. Testdouble)
    backend='nav2':      nav2_msgs/NavigateToPose (Layer 2 des Kollegen, frame 'map') - hier
                         NICHT getestet (nav2_msgs nicht installiert)."""
    aktionen = ("gehe_zu",)

    def __init__(self, node: Node, backend: str = "nav2", timeout_s: float = 180.0):
        self.node, self.backend, self.timeout_s = node, backend, timeout_s
        if backend == "gehaktion":
            self.client = ActionClient(node, GehAktion, GEH_ACTION)
        elif backend == "nav2":
            from nav2_msgs.action import NavigateToPose
            self._nav2_type = NavigateToPose
            self.client = ActionClient(node, NavigateToPose, NAV2_ACTION)
            self._active_cli = node.create_client(Trigger, NAV2_ACTIVE_SRV)
        else:
            raise ValueError(f"unbekanntes nav_backend {backend!r} (gehaktion|nav2)")

    def _wait_nav2_active(self, cancel: threading.Event, timeout_s: float = 90.0) -> bool:
        """Nav2-Action-Server existiert schon nach 'configure'; Ziele werden aber erst nach
        'activate' bearbeitet (sonst sofort Status ABORTED). Daher auf is_active warten."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s and not cancel.is_set():
            if self._active_cli.wait_for_service(timeout_sec=0.5):
                f = self._active_cli.call_async(Trigger.Request())
                while not f.done() and time.monotonic() - t0 < timeout_s and not cancel.is_set():
                    time.sleep(0.05)
                if f.done() and f.result() is not None and f.result().success:
                    return True
            time.sleep(0.5)
        return False

    def execute(self, schritt: PlanSchritt, mission: Mission, cancel: threading.Event) -> BridgeResult:
        x, y = mission.orte[schritt.args[1]]
        if self.backend == "nav2" and not self._wait_nav2_active(cancel):
            return BridgeResult(False, "Nav2 nicht aktiv" if not cancel.is_set() else "abgebrochen")
        if self.backend == "gehaktion":
            goal = GehAktion.Goal()
            goal.ziel_position_x_m, goal.ziel_position_y_m = float(x), float(y)
            goal.ziel_geschwindigkeit_m_pro_s = float(mission.param("nav_geschwindigkeit"))
            ok, res, text = run_action(self.client, goal, cancel, self.timeout_s)
            if res is not None:
                ok = ok and res.erfolgreich
                text = res.ergebnis or text
        else:
            goal = self._nav2_type.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
            goal.pose.pose.orientation.w = 1.0
            ok, res, text = self._send_nav2(goal, cancel)
        return BridgeResult(bool(ok), text)

    def _send_nav2(self, goal, cancel: threading.Event):
        """Nav2-Ziel senden; ein sofortiger Abbruch (< 6 s, z.B. Planner/TF gerade noch nicht
        bereit, CPU-Spitze beim Missionsstart) wird EINMAL still wiederholt, statt eine ganze
        Neuplanung auszuloesen."""
        t0 = time.monotonic()
        ok, res, text = run_action(self.client, goal, cancel, self.timeout_s)
        if not ok and not cancel.is_set() and time.monotonic() - t0 < 6.0 and "abgebrochen" not in text:
            time.sleep(2.0)
            ok, res, text = run_action(self.client, goal, cancel, self.timeout_s)
        return ok, res, text


class BalanceBridge(IActionBridge):
    """'balanciere(ort)': BalancierAktion (anhalten, stabil stehen)."""
    aktionen = ("balanciere",)

    def __init__(self, node: Node, timeout_s: float = 40.0):
        self.client = ActionClient(node, BalancierAktion, BALANCIER_ACTION)
        self.timeout_s = timeout_s

    def execute(self, schritt, mission: Mission, cancel: threading.Event) -> BridgeResult:
        goal = BalancierAktion.Goal()
        goal.toleranz_grad = float(mission.param("balancier_toleranz_grad"))
        ok, res, text = run_action(self.client, goal, cancel, self.timeout_s)
        if res is not None:
            ok = ok and res.erfolgreich
            text = res.ergebnis or text
        return BridgeResult(bool(ok), text)
