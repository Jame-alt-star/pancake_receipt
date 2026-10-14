"""TESTDOUBLE fuer Layer 2 - KEINE Navigation fuer den Produktivbetrieb.

Ersetzt Nav2 nur in Tests/Entwicklung von Layer 1: ein GehAktion-Server, der (ohne Karte,
ohne Hindernisse) auf das Ziel ausrichtet, geradeaus geht und anhaelt. Layer 2 liefert der
Kollege mit Nav2.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from roboter_interfaces.action import GehAktion

WZ_MAX = 0.2        # Trainingsbereich der Policy
TIMEOUT_S = 150.0


class NavTestdouble(Node):
    def __init__(self):
        super().__init__("nav_testdouble")
        self.declare_parameter("toleranz_m", 0.3)
        self.pos = None
        self.yaw = None
        cb = ReentrantCallbackGroup()
        self.create_subscription(Odometry, "/odom", self._odom, 10, callback_group=cb)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.server = ActionServer(self, GehAktion, "/geh_aktion", execute_callback=self._execute,
                                   goal_callback=lambda g: GoalResponse.ACCEPT,
                                   cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=cb)
        self.get_logger().info("Nav-TESTDOUBLE aktiv (ersetzt Nav2 nur fuer Tests).")

    def _odom(self, m):
        self.pos = (m.pose.pose.position.x, m.pose.pose.position.y)
        o = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))

    def _cmd(self, vx=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.angular.z = float(vx), float(wz)
        self.pub.publish(t)

    def _execute(self, gh):
        g = gh.request
        tol = float(self.get_parameter("toleranz_m").value)
        v = max(0.3, min(float(g.ziel_geschwindigkeit_m_pro_s) or 0.4, 0.8))
        res, fb = GehAktion.Result(), GehAktion.Feedback()
        t0 = time.monotonic()
        d0 = None
        while rclpy.ok():
            if gh.is_cancel_requested:
                self._cmd()
                gh.canceled()
                res.erfolgreich, res.ergebnis = False, "abgebrochen"
                return res
            if self.pos is None or self.yaw is None:
                time.sleep(0.1)
                continue
            dx, dy = g.ziel_position_x_m - self.pos[0], g.ziel_position_y_m - self.pos[1]
            dist = math.hypot(dx, dy)
            d0 = d0 or max(dist, 1e-6)
            if dist < tol:
                self._cmd()
                gh.succeed()
                res.erfolgreich, res.ergebnis = True, f"Ziel erreicht (Abstand {dist:.2f} m)"
                return res
            err = (math.atan2(dy, dx) - self.yaw + math.pi) % (2 * math.pi) - math.pi
            if abs(err) > 0.35:
                self._cmd(0.0, math.copysign(WZ_MAX, err))          # erst ausrichten
            else:
                self._cmd(min(v, max(0.3, dist)), max(-WZ_MAX, min(WZ_MAX, 1.0 * err)))
            fb.fortschritt = max(0.0, min(1.0, 1.0 - dist / d0))
            fb.status = f"Abstand {dist:.2f} m"
            gh.publish_feedback(fb)
            if time.monotonic() - t0 > TIMEOUT_S:
                self._cmd()
                gh.abort()
                res.erfolgreich, res.ergebnis = False, "Zeitueberschreitung"
                return res
            time.sleep(0.1)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = NavTestdouble()
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
