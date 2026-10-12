import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from r1_bringup.bridges.mujoco_bridge import MujocoBridge
from roboter_steuerung.control.rl_locomotion_controller import RLLocomotionController


class SteuereinheitNode(Node):
    """
    Zentraler Regelungs-Knoten (Layer 3): die Regelschleife laeuft NICHT
    mehr ueber einen eigenen Wall-Clock-Timer (fruehere Version, siehe
    Git-Historie), sondern event-getrieben - sie wird direkt ausgeloest,
    wenn die Bridge eine neue /joint_states-Nachricht von mujoco_node.py
    bekommt (siehe on_state_update unten bzw. mujoco_bridge.py). Dadurch
    ist die Anzahl der Physikschritte zwischen zwei Regelaufrufen immer
    exakt control_decimation, egal wie schnell/langsam die Physik real
    laeuft - vorher liefen Physik-Schleife und 50-Hz-Timer unabhaengig
    voneinander und drifteten gegeneinander weg, was den Roboter zum
    Umfallen gebracht hat. Kennt selbst weder MuJoCo noch die neuronale
    Netz-Architektur - beides steckt hinter den beiden Objekten
    self.bridge/self.controller, austauschbar ueber IRobotHardwareBridge
    bzw. den RL-Controller selbst.
    """

    def __init__(self):
        super().__init__("steuereinheit_node")

        # 1. Konfiguration laden - dieselbe g1_policy.yaml wie
        # mujoco_node.py, damit z.B. simulation_dt/control_decimation
        # an beiden Stellen zusammenpassen (siehe Taktung unten).
        bringup_share = get_package_share_directory("r1_bringup")
        config_path = os.path.join(bringup_share, "config", "g1_policy.yaml")
        with open(config_path, "r") as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

        # 2. Hardware-Bridge und RL-Controller initialisieren. num_joints
        # kommt aus num_actions in der YAML (Single Source of Truth) statt
        # hartkodiert zu sein - siehe mujoco_bridge.py. on_state_update=
        # self._control_loop koppelt die Regelschleife direkt an den
        # Empfang neuer Sensordaten (siehe Klassendocstring oben) - KEIN
        # eigener Timer mehr.
        self.bridge = MujocoBridge(
            self,
            num_joints=self.config["num_actions"],
            on_state_update=self._control_loop
        )
        self.controller = RLLocomotionController(self.config)

        # 3. Start-/Default-Geschwindigkeitskommando aus der YAML.
        # Offener Punkt: self.cmd wird hier nur EINMAL beim Start gesetzt
        # und danach nie mehr veraendert - es gibt noch keinen /cmd_vel-
        # Subscriber. Der Roboter laeuft also aktuell dauerhaft mit dem
        # festen Start-Kommando, eine Steuerung "von aussen" (Joystick,
        # Nav2, Tastatur) ist noch nicht angebunden.
        self.cmd = np.array(self.config.get("cmd_init", [0.5, 0.0, 0.0]), dtype=np.float32)

        # 4. KEIN eigener Timer mehr (siehe Klassendocstring) - die
        # Regelschleife wird jetzt ueber on_state_update oben ausgeloest,
        # sobald mujoco_node.py eine neue, bereits korrekt dezimierte
        # /joint_states-Nachricht sendet.
        self.get_logger().info("Steuereinheit erfolgreich gestartet (event-getriebener Regelkreis aktiv).")

    def _control_loop(self):
        # Wird jetzt direkt von der Bridge aufgerufen, sobald eine neue
        # /joint_states-Nachricht ankommt (siehe on_state_update oben) -
        # nicht mehr von einem eigenen Timer. read_imu()/read_joint_states()
        # liefern trotzdem weiterhin nur den zuletzt gespeicherten Wert
        # zurueck (siehe mujoco_bridge.py); da /imu und /joint_states fast
        # gleichzeitig in mujoco_node.py veroeffentlicht werden, ist der
        # IMU-Wert hier in der Praxis minimal (hoechstens einen ROS2-
        # Callback) aelter als die gerade angekommenen Gelenkwerte.
        quat, omega = self.bridge.read_imu()
        joint_pos, joint_vel = self.bridge.read_joint_states()

        # Start-Gate: direkt nach dem Start dieses Knotens sind die
        # Bridge-Werte noch auf ihrem Initialwert (alles 0.0), weil noch
        # keine /joint_states- bzw. /imu-Nachricht von mujoco_node.py
        # angekommen ist. Eine echte Quaternion ist nie komplett 0 (siehe
        # last_quat-Default in mujoco_bridge.py) - der Vergleich erkennt
        # diesen Fall also zuverlaessig und ueberspringt den Takt, statt
        # die Policy mit bedeutungslosen Nullwerten laufen zu lassen.
        if np.all(joint_pos == 0.0) and np.all(quat == 0.0):
            return

        # Soll-Gelenkwinkel ueber das neuronale Netz (RL-Policy) berechnen.
        target_positions = self.controller.compute_joint_targets(
            joint_positions=joint_pos,
            joint_velocities=joint_vel,
            base_quat=quat,
            base_ang_vel=omega,
            cmd=self.cmd
        )

        # Zielwinkel zurueck an die Simulation schicken (per /joint_commands,
        # dort uebernimmt die Low-Level-PD-Regelung in mujoco_node.py).
        self.bridge.send_joint_commands(target_positions)


def main(args=None):
    rclpy.init(args=args)
    node = SteuereinheitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
