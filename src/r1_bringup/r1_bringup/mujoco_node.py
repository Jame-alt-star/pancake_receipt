import os
import threading
import time
import yaml
import numpy as np

import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
from ament_index_python.packages import get_package_share_directory


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """
    Low-Level-Drehmoment-Regelung je Gelenk (Layer 4), uebernommen aus
    unitree_rl_gym's deploy_mujoco.py (dort Zeile 25-28): Drehmoment =
    Steifigkeit * Positionsfehler + Daempfung * Geschwindigkeitsfehler.
    target_dq ist hier immer 0 (siehe Aufruf unten) - es wird nur eine
    Zielposition angesteuert, keine Zielgeschwindigkeit.
    """
    return (target_q - q) * kp + (target_dq - dq) * kd


class MujocoBridgeNode(Node):
    """
    Simulations-Knoten (Layer 4): laedt die MuJoCo-Szene, rechnet die
    Physik + Low-Level-PD-Regelung in einem Hintergrund-Thread, und
    kommuniziert mit steuereinheit_node.py AUSSCHLIESSLICH ueber ROS2-
    Topics (/joint_states, /imu, /base_position raus, /joint_commands
    rein). Kennt selbst keine RL-Policy, kein neuronales Netz - reine
    Physik.
    """

    def __init__(self):
        super().__init__("mujoco_bridge_node")

        # 1. Config laden (gleiche g1_policy.yaml wie steuereinheit_node.py,
        # damit z.B. simulation_dt/kps/kds an beiden Stellen zusammenpassen)
        pkg_share = get_package_share_directory("r1_bringup")
        config_path = os.path.join(pkg_share, "config", "g1_policy.yaml")

        with open(config_path, "r") as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

        # Pfade aufloesen (relativ zum Share-Verzeichnis)
        xml_rel_path = self.config["xml_path"]
        xml_path = os.path.join(pkg_share, xml_rel_path)
        if not os.path.exists(xml_path):
            # Fallback, falls colcon den Share-Ordner (noch) nicht gefuellt
            # hat. ACHTUNG: geht hart davon aus, dass der Workspace exakt
            # unter ~/humanoid_robot liegt - offener Punkt, falls der
            # Workspace-Ordner mal anders benannt/liegt, bricht das hier.
            xml_path = os.path.expanduser(f"~/humanoid_robot/src/r1_bringup/{xml_rel_path}")

        # Parameter initialisieren
        self.simulation_dt = float(self.config["simulation_dt"])
        self.kps = np.array(self.config["kps"], dtype=np.float32)
        self.kds = np.array(self.config["kds"], dtype=np.float32)
        self.default_angles = np.array(self.config["default_angles"], dtype=np.float32)
        self.target_dof_pos = self.default_angles.copy()

        # 2. MuJoCo Modell & Daten laden
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.simulation_dt

        # Ruhelage zuweisen, damit der Roboter nicht in der MuJoCo-
        # Standardpose (meist alle Gelenke auf 0) startet
        self.data.qpos[7:] = self.default_angles
        mujoco.mj_forward(self.model, self.data)

        # 3. ROS-2 Publisher & Subscriber
        self.pub_joint_states = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_imu = self.create_publisher(Imu, "/imu", 10)
        # NEU (12.10.2026): /base_position - die /imu-Nachricht (sensor_msgs/
        # Imu) hat bewusst kein Positionsfeld (ein echtes IMU misst nie
        # absolute Position, nur Orientierung/Drehrate/Beschleunigung) -
        # deshalb ein eigenes, einfaches Topic dafuer. WICHTIG: das ist
        # reiner Simulations-Ground-Truth (direkt aus qpos[0:3], siehe
        # _publish_states()) - auf echter Hardware gaebe es das nicht
        # einfach so, dafuer braeuchte es echte Odometrie/Lokalisierung.
        # Bewusste Vereinfachung fuer den aktuellen Stand, kein Ersatz dafuer.
        self.pub_base_position = self.create_publisher(Point, "/base_position", 10)
        self.sub_joint_commands = self.create_subscription(
            Float32MultiArray,
            "/joint_commands",
            self._joint_commands_callback,
            10
        )

        # 4. Simulationsschleife im Hintergrundthread starten - als eigener
        # Thread, weil mujoco.viewer.launch_passive() blockierend laeuft;
        # ROS2 (rclpy.spin in main()) muss parallel weiterlaufen koennen,
        # damit Subscriber/Publisher nicht einfrieren. daemon=True, damit
        # der Thread beim Beenden des Knotens nicht den Prozess offen haelt.
        self.is_running = True
        self.sim_thread = threading.Thread(target=self._run_simulation)
        self.sim_thread.daemon = True
        self.sim_thread.start()

    def _joint_commands_callback(self, msg: Float32MultiArray):
        # Laengen-Check als Sicherheitsnetz: eine falsch dimensionierte
        # Nachricht (z.B. durch einen Konfigurationsfehler) wird ignoriert,
        # statt die Simulation mit falsch vielen Werten zum Absturz zu bringen.
        cmd_data = np.array(msg.data, dtype=np.float32)
        if len(cmd_data) == len(self.target_dof_pos):
            self.target_dof_pos = cmd_data

    def _run_simulation(self):
        # BUGFIX (gemessen: /joint_states kam nur mit ~90 Hz statt der
        # angenommenen 500 Hz an): viewer.sync() - das Rendern des Fensters -
        # kostet pro Aufruf relativ viel Zeit und wurde vorher bei JEDEM
        # einzelnen Physikschritt aufgerufen. Dadurch schaffte die Schleife
        # real nur ~90 Hz statt 500 Hz, obwohl die Zeitmessung unten
        # (time_until_next) das nicht verhindern konnte - sie drosselt nur,
        # wenn die Schleife SCHNELLER als Echtzeit waere, nicht wenn sie
        # langsamer ist. Weil steuereinheit_node.py unabhaengig davon mit
        # einem eigenen 50-Hz-Wall-Clock-Timer laeuft und intern davon
        # ausgeht, dass pro Regeltakt 10 Physikschritte (control_decimation)
        # vergangen sind, driftete die angenommene Simulationszeit (fuer das
        # Phasensignal) weit von der tatsaechlichen Physik auseinander -
        # das hat den Roboter instabil werden lassen und umfallen lassen.
        # Fix: nur noch alle RENDER_DECIMATION Physikschritte rendern (~50 Hz
        # Bildwiederholung reicht fuers Auge voellig aus), mj_step()/
        # _publish_states() bleiben bei jedem Schritt - das entlastet die
        # Schleife so weit, dass sie near-Echtzeit auf ~500 Hz kommen sollte.
        RENDER_DECIMATION = 10
        render_counter = 0

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # NEU: freie Kamera mit groesserem Abstand, direkt von hinten
            # (Laufrichtung +X, siehe cmd_init in g1_policy.yaml), leicht
            # erhoeht - damit der ganze Roboter UND ein Stueck Laufweg
            # sichtbar sind, ohne jedes Mal manuell nachstellen zu muessen.
            # lookat wird unten im Render-Takt auf die Pelvis-Position
            # aktualisiert, Abstand/Winkel bleiben unberuehrt (per Maus im
            # Viewer weiter frei verstellbar).
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.distance = 5.0
            viewer.cam.azimuth = 180.0
            viewer.cam.elevation = -20.0
            viewer.cam.lookat[:] = self.data.qpos[0:3]

            while viewer.is_running() and self.is_running:
                step_start = time.time()

                # Low-Level-PD-Regelung an Aktuatoren anwenden. qpos[7:]/
                # qvel[6:]: die ersten 7 bzw. 6 Werte gehoeren zur
                # "Floating Base" (qpos: 3 Position + 4 Quaternion = 7;
                # qvel: 3 lineare + 3 Winkelgeschwindigkeit = 6) - danach
                # erst kommen die eigentlichen Gelenkwerte.
                q = self.data.qpos[7:]
                dq = self.data.qvel[6:]
                tau = pd_control(
                    self.target_dof_pos, q, self.kps, np.zeros_like(self.kds), dq, self.kds
                )
                self.data.ctrl[:] = tau

                # Physikschritt berechnen
                mujoco.mj_step(self.model, self.data)

                # Sensor-Zustaende publizieren - bei JEDEM Physikschritt,
                # damit steuereinheit_node.py moeglichst aktuelle Daten
                # bekommt (siehe Bugfix-Kommentar oben).
                self._publish_states()

                render_counter += 1
                if render_counter >= RENDER_DECIMATION:
                    # Kamera folgt dem Roboter (nur Position - Abstand/
                    # Winkel bleiben manuell verstellbar, siehe oben).
                    viewer.cam.lookat[:] = self.data.qpos[0:3]
                    viewer.sync()
                    render_counter = 0

                # Taktung drosseln (auf Physikzeitschritt), damit die
                # Simulation nicht schneller als Echtzeit durchlaeuft
                time_until_next = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next > 0:
                    time.sleep(time_until_next)

    def _publish_states(self):
        now = self.get_clock().now().to_msg()

        # Joint States
        js = JointState()
        js.header.stamp = now
        js.position = self.data.qpos[7:].tolist()
        js.velocity = self.data.qvel[6:].tolist()
        self.pub_joint_states.publish(js)

        # IMU: direkt aus dem simulierten "Floating Base"-Zustand abgeleitet
        # (qpos[3:7] = Orientierungs-Quaternion [w,x,y,z], qvel[3:6] =
        # Winkelgeschwindigkeit) - KEIN simuliertes Sensormodell an einer
        # konkreten Koerperstelle. Offener Punkt: der aeltere Stand hatte
        # zwei getrennte IMUs (Torso + Becken); hier ist es bewusst nur
        # eine, idealisierte - fuer die aktuelle 47-Werte-Policy ausreichend,
        # aber weniger realitaetsnah als zwei echte Sensor-Sites.
        imu = Imu()
        imu.header.stamp = now
        imu.orientation.w = float(self.data.qpos[3])
        imu.orientation.x = float(self.data.qpos[4])
        imu.orientation.y = float(self.data.qpos[5])
        imu.orientation.z = float(self.data.qpos[6])
        imu.angular_velocity.x = float(self.data.qvel[3])
        imu.angular_velocity.y = float(self.data.qvel[4])
        imu.angular_velocity.z = float(self.data.qvel[5])
        self.pub_imu.publish(imu)

        # NEU (12.10.2026): Basis-Position (qpos[0:3] = x, y, z der
        # "Floating Base") - fuer die Positions-Haltung beim Stehen
        # (siehe steuereinheit_node.py/_apply_position_hold()). Siehe
        # Kommentar beim Publisher oben: reiner Simulations-Ground-Truth.
        pos = Point()
        pos.x = float(self.data.qpos[0])
        pos.y = float(self.data.qpos[1])
        pos.z = float(self.data.qpos[2])
        self.pub_base_position.publish(pos)

    def destroy_node(self):
        # Hintergrund-Thread aktiv stoppen, bevor der Knoten zerstoert wird -
        # sonst liefe die Simulationsschleife theoretisch im Hintergrund weiter.
        self.is_running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MujocoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
