from typing import Tuple
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

from r1_bringup.bridges.i_hardware_bridge import IRobotHardwareBridge


class MujocoBridge(IRobotHardwareBridge):
    """
    Konkrete Hardware-Bridge fuer die MuJoCo-Simulation (Layer 4).

    Kommuniziert NICHT direkt mit MuJoCo, sondern ausschliesslich ueber
    ROS2-Topics mit mujoco_node.py (zwei getrennte Prozesse/Knoten).
    Das ist bewusst so: steuereinheit_node.py (und damit die RL-Policy)
    kennt nur diese Klasse und das IRobotHardwareBridge-Interface -
    welche Simulation/Hardware dahinter tatsaechlich steckt, ist fuer den
    Rest des Codes unsichtbar. Fuer echte Hardware wuerde spaeter eine
    andere Klasse mit derselben Schnittstelle diese hier ersetzen.
    """

    def __init__(self, node: Node, num_joints: int):
        self.node = node

        # Platzhalter-Werte, bevor die erste echte Sensor-Nachricht von
        # mujoco_node.py angekommen ist. WICHTIG: [0,0,0,0] ist absichtlich
        # KEINE gueltige Quaternion (eine echte Drehung kann das nie sein) -
        # genau deshalb eignet sie sich als "noch keine echten Daten"-
        # Erkennungswert fuer den Warte-Check in steuereinheit_node.py.
        self.last_quat = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.last_omega = np.zeros(3, dtype=np.float32)
        # NEU (12.10.2026): fuer read_base_position() - siehe dort und
        # i_hardware_bridge.py fuer den Vorbehalt (reiner Simulations-
        # Ground-Truth).
        self.last_base_position = np.zeros(3, dtype=np.float32)

        # num_joints kommt von aussen (steuereinheit_node.py uebergibt
        # self.config["num_actions"] aus g1_policy.yaml) - so steht die
        # Gelenkanzahl nur an EINER Stelle (single source of truth) und
        # nicht zusaetzlich hier nochmal fest codiert.
        self.last_joint_pos = np.zeros(num_joints, dtype=np.float32)
        self.last_joint_vel = np.zeros(num_joints, dtype=np.float32)

        # Publisher fuer Zielwinkel: steuereinheit_node.py liest nie
        # direkt aus MuJoCo, sondern ruft send_joint_commands() auf,
        # was hier landet und per ROS2-Topic an mujoco_node.py weitergeht.
        self.cmd_pub = self.node.create_publisher(Float32MultiArray, "/joint_commands", 10)

        # Subscriber fuer Sensorwerte: mujoco_node.py veroeffentlicht laufend
        # /joint_states, /imu und /base_position, hier wird nur der jeweils
        # letzte Wert gemerkt (siehe _joint_callback/_imu_callback/
        # _base_position_callback unten).
        self.node.create_subscription(JointState, "/joint_states", self._joint_callback, 10)
        self.node.create_subscription(Imu, "/imu", self._imu_callback, 10)
        self.node.create_subscription(Point, "/base_position", self._base_position_callback, 10)

    def _joint_callback(self, msg: JointState):
        # Wird von ROS2 automatisch bei jeder neuen /joint_states-
        # Nachricht aufgerufen (nicht manuell aufgerufen!). Prueft erst,
        # ob die Nachricht ueberhaupt Werte enthaelt, bevor ueberschrieben
        # wird - schuetzt vor leeren/kaputten Nachrichten.
        if len(msg.position) > 0:
            self.last_joint_pos = np.array(msg.position, dtype=np.float32)
            self.last_joint_vel = np.array(msg.velocity, dtype=np.float32)

    def _imu_callback(self, msg: Imu):
        # Wird automatisch bei jeder neuen /imu-Nachricht aufgerufen.
        # Quaternion-Reihenfolge bewusst (w, x, y, z) - muss so zur
        # Erwartung in rl_locomotion_controller.py passen.
        self.last_quat = np.array(
            [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
            dtype=np.float32
        )
        self.last_omega = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            dtype=np.float32
        )

    def _base_position_callback(self, msg: Point):
        # NEU (12.10.2026): wird automatisch bei jeder neuen /base_position-
        # Nachricht aufgerufen (siehe mujoco_node.py/_publish_states()).
        self.last_base_position = np.array([msg.x, msg.y, msg.z], dtype=np.float32)

    def read_imu(self) -> Tuple[np.ndarray, np.ndarray]:
        # .copy() ist wichtig: ohne Kopie koennte der Aufrufer (z.B. der
        # RL-Controller) versehentlich diese internen Merker-Werte
        # veraendern, was dann beim naechsten echten Sensor-Update zu
        # verwirrenden Fehlern fuehren wuerde.
        return self.last_quat.copy(), self.last_omega.copy()

    def read_joint_states(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.last_joint_pos.copy(), self.last_joint_vel.copy()

    def read_base_position(self) -> np.ndarray:
        return self.last_base_position.copy()

    def send_joint_commands(self, target_positions: np.ndarray) -> None:
        # tolist(): ROS2-Nachrichten verstehen keine NumPy-Arrays direkt,
        # nur normale Python-Listen.
        msg = Float32MultiArray()
        msg.data = target_positions.tolist()
        self.cmd_pub.publish(msg)
