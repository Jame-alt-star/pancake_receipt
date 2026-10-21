"""Layer-3-Dienste fuer die obere Schicht (SysML: GleichgewichtServicePort, BalancierAktionPort).

Eigener, schlanker Knoten neben steuereinheit_node - bewusst ohne Zugriff auf dessen
Regelschleife, nur ueber Topics:
  - Service  /gleichgewicht   (roboter_interfaces/GleichgewichtService): Neigung aus /imu
  - Action   /balanciere      (roboter_interfaces/BalancierAktion): Roboter anhalten (Vorrang-Eingang
    /cmd_vel/balance = 0)
    und melden, sobald er stabil steht (Neigung <= toleranz_grad UND Basisgeschwindigkeit klein
    fuer mind. STABIL_ZEIT_S).

Hinweis: Geschwindigkeit stammt aus /odom (Ground-Truth oder Beinodometrie, je nach odom_quelle).
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu

from roboter_beschreibung.geometrie import neigung_grad
from roboter_interfaces.action import BalancierAktion
from roboter_interfaces.srv import GleichgewichtService

STABIL_ZEIT_S = 1.0      # so lange muessen Neigung und Geschwindigkeit ununterbrochen im Rahmen sein
MAX_GESCHW_M_S = 0.03    # "steht": im Stand gemessen < 0.01 m/s Drift, Gehen >= 0.3 m/s
TIMEOUT_S = 20.0         # Bremsen aus voller Fahrt dauert ~2-3 s; 20 s mit grosser Reserve


class _Tap:
    """Abo auf Abruf: abonniert nur zwischen acquire() und release(). Spart CPU, weil die 100-Hz-Topics
    (/imu, /odom) im Leerlauf nicht dauerhaft in Python deserialisiert werden muessen."""

    def __init__(self, node, msg_type, topic, callback, group):
        self.node, self.msg_type, self.topic, self.cb, self.group = node, msg_type, topic, callback, group
        self.sub = None
        self.refs = 0
        self.lock = threading.Lock()

    def acquire(self):
        with self.lock:
            self.refs += 1
            if self.sub is None:
                self.sub = self.node.create_subscription(self.msg_type, self.topic, self.cb, 10,
                                                         callback_group=self.group)

    def release(self):
        with self.lock:
            self.refs = max(0, self.refs - 1)
            if self.refs == 0 and self.sub is not None:
                self.node.destroy_subscription(self.sub)
                self.sub = None


class BalanceDienstNode(Node):
    def __init__(self):
        super().__init__("balance_dienst_node")
        self.declare_parameter("stabil_grenze_grad", 15.0)
        self.grenze = float(self.get_parameter("stabil_grenze_grad").value)
        self._lock = threading.Lock()
        self._quat = None
        self._hist = []                      # (t, x, y) der letzten ~1 s
        cb = ReentrantCallbackGroup()
        self._tap_imu = _Tap(self, Imu, "/imu", self._imu, cb)
        self._tap_pos = _Tap(self, Odometry, "/odom", self._pos, cb)
        # Eigener Vorrang-Eingang der Steuerung: frueher auf /cmd_vel, dort konkurrierte der Haltebefehl mit Nav2.
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel/balance", 10)
        self.create_service(GleichgewichtService, "/gleichgewicht", self._service, callback_group=cb)
        self.action = ActionServer(
            self, BalancierAktion, "/balanciere", execute_callback=self._execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=cb)
        self.get_logger().info("Balance-Dienste bereit (/gleichgewicht, /balanciere).")

    def _imu(self, m: Imu):
        o = m.orientation
        with self._lock:
            self._quat = (o.w, o.x, o.y, o.z)

    def _pos(self, m: Odometry):
        now = time.monotonic()
        p = m.pose.pose.position
        with self._lock:
            self._hist.append((now, p.x, p.y))
            self._hist = [h for h in self._hist if now - h[0] <= 1.0]

    def _tilt(self):
        with self._lock:
            q = self._quat
        return None if q is None else neigung_grad(q)

    def _speed(self):
        with self._lock:
            h = list(self._hist)
        if len(h) < 2 or h[-1][0] - h[0][0] < 0.3:
            return None
        return math.hypot(h[-1][1] - h[0][1], h[-1][2] - h[0][2]) / (h[-1][0] - h[0][0])

    def _service(self, req, resp):
        with self._lock:
            self._quat = None
        self._tap_imu.acquire()
        try:
            t0 = time.monotonic()
            while self._tilt() is None and time.monotonic() - t0 < 1.0:
                time.sleep(0.02)
            t = self._tilt()
        finally:
            self._tap_imu.release()
        resp.neigung_grad = float(t) if t is not None else 0.0
        resp.stabil = bool(t is not None and t <= self.grenze)
        return resp

    def _execute(self, goal_handle):
        tol = float(goal_handle.request.toleranz_grad) or 5.0
        t0 = time.monotonic()
        stabil_seit = None
        fb = BalancierAktion.Feedback()
        res = BalancierAktion.Result()
        self.get_logger().info(f"Balancieren gestartet (Toleranz {tol:.1f} Grad)")
        with self._lock:
            self._quat, self._hist = None, []
        self._tap_imu.acquire()
        self._tap_pos.acquire()
        try:
            return self._balance_loop(goal_handle, tol, t0, stabil_seit, fb, res)
        finally:
            self._tap_imu.release()
            self._tap_pos.release()

    def _balance_loop(self, goal_handle, tol, t0, stabil_seit, fb, res):
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                res.erfolgreich, res.ergebnis = False, "abgebrochen"
                return res
            self.pub_cmd.publish(Twist())          # anhalten
            tilt, v = self._tilt(), self._speed()
            ok = tilt is not None and tilt <= tol and v is not None and v <= MAX_GESCHW_M_S
            now = time.monotonic()
            stabil_seit = (stabil_seit or now) if ok else None
            gehalten = (now - stabil_seit) if stabil_seit else 0.0
            fb.fortschritt = min(1.0, gehalten / STABIL_ZEIT_S)
            fb.status = f"neigung={tilt if tilt is None else round(tilt, 1)} v={v if v is None else round(v, 3)}"
            goal_handle.publish_feedback(fb)
            if gehalten >= STABIL_ZEIT_S:
                goal_handle.succeed()
                res.erfolgreich, res.ergebnis = True, "steht stabil"
                return res
            if now - t0 > TIMEOUT_S:
                goal_handle.abort()
                res.erfolgreich, res.ergebnis = False, "Zeitueberschreitung: nicht stabil"
                return res
            time.sleep(0.1)
        res.erfolgreich, res.ergebnis = False, "beendet"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = BalanceDienstNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
