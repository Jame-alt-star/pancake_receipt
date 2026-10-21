import time
from typing import Tuple

import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray

from roboter_interfaces.msg import Antriebsparameter
from roboter_steuerung.hardware.i_hardware_bridge import IRobotHardwareBridge


class TopicBridge(IRobotHardwareBridge):
    """
    Hardware-Bridge ueber ROS2-Topics (bis 14.10.2026 "MujocoBridge" im Simulationspaket).

    Kennt MuJoCo nicht: liest /joint_states und /imu, schreibt /joint_commands. Auf der anderen Seite
    steht heute mujoco_node (Layer 4); ein Hardware-Knoten mit denselben Topics koennte ihn ersetzen,
    oder eine SDK-Bridge implementiert IRobotHardwareBridge direkt.

    Die Antriebsparameter (kp, kd, Startpose) gehoeren zur Policy und werden einmal latched auf
    /antrieb/parameter gesendet; die Simulation startet erst, wenn sie da sind (wie ein Motortreiber,
    der seine Verstaerkungen vom Regler bekommt).
    """

    def __init__(self, node: Node, num_joints: int, kp=None, kd=None, startpose=None):
        self.node = node

        # Platzhalter-Werte, bevor die erste echte Sensor-Nachricht angekommen ist. WICHTIG: [0,0,0,0] ist
        # absichtlich KEINE gueltige Quaternion - genau deshalb eignet sie sich als "noch keine echten
        # Daten"-Erkennungswert fuer das Start-Gate in steuereinheit_node.py.
        self.last_quat = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.last_omega = np.zeros(3, dtype=np.float32)

        # num_joints kommt aus num_actions der Policy-Config (eine Stelle fuer die Gelenkanzahl).
        self.last_joint_pos = np.zeros(num_joints, dtype=np.float32)
        self.last_joint_vel = np.zeros(num_joints, dtype=np.float32)
        self._t_gelenke = self._t_imu = None          # Empfangszeit (monotonic) der letzten Nachricht
        # Optional: wird aufgerufen, sobald seit dem letzten Aufruf BEIDE Quellen neu sind (Takt der Steuerung aus den
        # Sensordaten). Frueher nach jeder Gelenknachricht: die Simulation sendet erst Gelenke, dann IMU - die Policy
        # bekam dadurch systematisch eine 10 ms aeltere IMU als die Gelenkwinkel. Im realistischen Modus (zusaetzlich
        # 10 ms Latenz) fiel der Roboter danach beim Anhalten in 2 von 4 Pipeline-Laeufen (21.10.2026).
        self.bei_sensordaten = None
        self._neu_gelenke = self._neu_imu = False

        self.cmd_pub = self.node.create_publisher(Float32MultiArray, "/joint_commands", 10)
        self.node.create_subscription(JointState, "/joint_states", self._joint_callback, 10)
        self.node.create_subscription(Imu, "/imu", self._imu_callback, 10)

        if kp is not None:
            latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)
            self.antrieb_pub = self.node.create_publisher(Antriebsparameter, "/antrieb/parameter", latched)
            msg = Antriebsparameter()
            msg.kp = [float(x) for x in kp]
            msg.kd = [float(x) for x in kd]
            msg.startpose = [float(x) for x in startpose]
            self.antrieb_pub.publish(msg)

    def _joint_callback(self, msg: JointState):
        # Wird von ROS2 automatisch bei jeder neuen /joint_states-
        # Nachricht aufgerufen (nicht manuell aufgerufen!). Prueft erst,
        # ob die Nachricht ueberhaupt Werte enthaelt, bevor ueberschrieben
        # wird - schuetzt vor leeren/kaputten Nachrichten.
        if len(msg.position) > 0:
            self.last_joint_pos = np.array(msg.position, dtype=np.float32)
            self.last_joint_vel = np.array(msg.velocity, dtype=np.float32)
            self._t_gelenke = time.monotonic()
            self._neu_gelenke = True
            self._melde_takt()

    def _imu_callback(self, msg: Imu):
        # Wird automatisch bei jeder neuen /imu-Nachricht aufgerufen.
        # Quaternion-Reihenfolge bewusst (w, x, y, z) - muss so zur
        # Erwartung in rl_locomotion_controller_29dof.py passen.
        self.last_quat = np.array(
            [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
            dtype=np.float32
        )
        self.last_omega = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            dtype=np.float32
        )
        self._t_imu = time.monotonic()
        self._neu_imu = True
        self._melde_takt()

    def _melde_takt(self):
        if self._neu_gelenke and self._neu_imu:
            self._neu_gelenke = self._neu_imu = False
            if self.bei_sensordaten is not None:
                self.bei_sensordaten()

    def sensor_detail(self) -> str:
        """Alter je Quelle, fuer die Diagnose einer Stoerung."""
        jetzt = time.monotonic()
        alter = lambda t: "nie" if t is None else f"{jetzt - t:.2f} s"     # noqa: E731
        return f"Gelenke {alter(self._t_gelenke)}, IMU {alter(self._t_imu)}"

    def sensor_alter_s(self) -> float:
        if self._t_gelenke is None or self._t_imu is None:
            return float("inf")
        return time.monotonic() - min(self._t_gelenke, self._t_imu)

    def read_imu(self) -> Tuple[np.ndarray, np.ndarray]:
        # .copy() ist wichtig: ohne Kopie koennte der Aufrufer (z.B. der
        # RL-Controller) versehentlich diese internen Merker-Werte
        # veraendern, was dann beim naechsten echten Sensor-Update zu
        # verwirrenden Fehlern fuehren wuerde.
        return self.last_quat.copy(), self.last_omega.copy()

    def read_joint_states(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.last_joint_pos.copy(), self.last_joint_vel.copy()

    def send_joint_commands(self, target_positions: np.ndarray) -> None:
        # tolist(): ROS2-Nachrichten verstehen keine NumPy-Arrays direkt,
        # nur normale Python-Listen.
        msg = Float32MultiArray()
        msg.data = target_positions.tolist()
        self.cmd_pub.publish(msg)
