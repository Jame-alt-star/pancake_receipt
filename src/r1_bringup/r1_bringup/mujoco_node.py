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
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
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


class _NullViewer:
    """Platzhalter fuer headless=True: gleiche Schnittstelle wie der passive MuJoCo-Viewer,
    tut aber nichts (kein Fenster, kein Rendering)."""

    def __init__(self):
        from types import SimpleNamespace
        self.cam = SimpleNamespace(type=None, distance=0.0, azimuth=0.0, elevation=0.0,
                                   lookat=np.zeros(3))

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def is_running(self):
        return True

    def sync(self):
        pass


def rotate_inverse(q, v):
    """v (Weltkoordinaten) in das Koerper-Koordinatensystem mit Orientierung q=[w,x,y,z]."""
    w, x, y, z = q
    # R^T v  mit R aus Quaternion
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return r.T @ np.asarray(v, dtype=float)


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

        # 1. Config laden (gleiche Datei wie steuereinheit_node.py, damit
        # z.B. simulation_dt/kps/kds an beiden Stellen zusammenpassen).
        # NEU (DOF-Migration): Dateiname ueber den ROS2-Parameter
        # "config_file" waehlbar (Default bleibt "g1_policy.yaml" - exakt
        # das bisherige 12-DOF-Verhalten unveraendert), siehe
        # steuereinheit_node.py fuer denselben Mechanismus und
        # steuereinheit.launch.py fuer das Launch-Argument, das beide
        # Knoten synchron umschaltet.
        self.declare_parameter("config_file", "g1_policy.yaml")
        config_file = self.get_parameter("config_file").get_parameter_value().string_value

        # NEU (13.10.2026): Physik erst starten, wenn der Regler das erste
        # /joint_commands gesendet hat. Gemessen: steuereinheit_node
        # braucht ~7 s Wandzeit zum Hochfahren (Imports, ONNX-Laden); in
        # der Zeit simulierte dieser Knoten schon mit reiner PD-Haltung auf
        # default_angles, was nachweislich umkippt (siehe
        # balancieren_state.py "Variante 2") - der Roboter lag schon, bevor
        # die Policy ihr erstes Kommando senden konnte (z=0.096 bei
        # simtime 1.9 s, ncmd=0). Zustaende werden waehrend der Pause
        # weiter publiziert (der Regler braucht sie fuer sein Start-Gate).
        self.declare_parameter("wait_for_controller", True)
        self.wait_for_controller = self.get_parameter("wait_for_controller").get_parameter_value().bool_value
        self.controller_connected = not self.wait_for_controller
        self.declare_parameter("profile", False)
        self.profile = self.get_parameter("profile").get_parameter_value().bool_value

        pkg_share = get_package_share_directory("r1_bringup")
        config_path = os.path.join(pkg_share, "config", config_file)

        with open(config_path, "r") as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)
        self.get_logger().info(f"[mujoco_bridge_node] aktive Config: {config_file} (xml_path={self.config.get('xml_path')})")

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
        # NEU (13.10.2026): /odom + TF (odom -> base_link) fuer Nav2 (Layer 2). Reiner
        # Simulations-Ground-Truth (perfekte Odometrie, kein Drift/Rauschen). Abschaltbar, falls
        # eine andere Quelle (z.B. robot_localization) Odometrie/TF liefert.
        # NEU (13.10.2026): headless=True -> kein Viewer-Fenster. Spart bei Tests/Nav2-Betrieb
        # viel CPU und Speicher (Rendering war der groesste Posten der Sim-Schleife).
        self.declare_parameter("headless", False)
        self.headless = self.get_parameter("headless").value
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.publish_odom = self.get_parameter("publish_odom").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10) if self.publish_odom else None
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
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
        _now = time.perf_counter()
        if getattr(self, "_prof_last_cmd", None) is not None:
            self._prof_gap = max(getattr(self, "_prof_gap", 0.0), _now - self._prof_last_cmd)
        self._prof_last_cmd = _now
        self._prof_ncmd = getattr(self, "_prof_ncmd", 0) + 1
        if len(cmd_data) == len(self.target_dof_pos):
            self.target_dof_pos = cmd_data
            if not self.controller_connected:
                self.controller_connected = True
                self.get_logger().info("[mujoco_bridge_node] Regler verbunden - Physik gestartet.")

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
        # NEU (13.10.2026): Publish nur alle PUBLISH_DECIMATION Schritte
        # (500/5 = 100 Hz). Gemessen: _publish_states() kostete ~1.1 ms pro
        # Aufruf (3 Nachrichten, 29 Gelenke) - das war der groesste Posten
        # der Schleife und liess die Simulation nur mit ~0.5x Echtzeit
        # laufen. Die Policy laeuft mit 50 Hz (control_decimation=10), 100 Hz
        # Sensordaten reichen also; zusaetzliche Latenz <= 10 ms.
        PUBLISH_DECIMATION = 5
        publish_counter = 0
        # Absolute Zeitplanung: Verzoegerungen eines Schritts (Viewer-Sync,
        # sleep-Overshoot) werden durch kuerzere Wartezeit der folgenden
        # Schritte aufgeholt statt sich zu summieren.
        next_deadline = time.perf_counter()
        # TEMP-PROFILING (13.10.2026): Zeitanteile der Schleife, alle 2s geloggt
        prof = {"step": 0.0, "pub": 0.0, "sync": 0.0, "sleep": 0.0, "n": 0}
        prof_t0 = time.perf_counter()

        viewer_ctx = (_NullViewer() if self.headless
                      else mujoco.viewer.launch_passive(self.model, self.data))
        with viewer_ctx as viewer:
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

                if not self.controller_connected:
                    # Pausiert: Pose einfrieren, aber Sensorwerte (100 Hz)
                    # weiter senden und Fenster aktualisieren.
                    self._publish_states()
                    viewer.sync()
                    time.sleep(0.01)
                    next_deadline = time.perf_counter()
                    continue

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
                _t0 = time.perf_counter()
                mujoco.mj_step(self.model, self.data)
                _t1 = time.perf_counter()

                # Sensor-Zustaende publizieren - bei JEDEM Physikschritt,
                # damit steuereinheit_node.py moeglichst aktuelle Daten
                # bekommt (siehe Bugfix-Kommentar oben).
                publish_counter += 1
                if publish_counter >= PUBLISH_DECIMATION:
                    self._publish_states()
                    publish_counter = 0
                _t2 = time.perf_counter()
                prof["step"] += _t1 - _t0
                prof["pub"] += _t2 - _t1

                render_counter += 1
                if render_counter >= RENDER_DECIMATION:
                    # Kamera folgt dem Roboter (nur Position - Abstand/
                    # Winkel bleiben manuell verstellbar, siehe oben).
                    viewer.cam.lookat[:] = self.data.qpos[0:3]
                    _ts = time.perf_counter()
                    viewer.sync()
                    prof["sync"] += time.perf_counter() - _ts
                    render_counter = 0

                # Taktung drosseln (auf Physikzeitschritt), damit die
                # Simulation nicht schneller als Echtzeit durchlaeuft
                next_deadline += self.model.opt.timestep
                time_until_next = next_deadline - time.perf_counter()
                if time_until_next < -0.05:
                    # >50 ms im Rueckstand (z.B. Fenster verschoben): nicht
                    # endlos im Schnelldurchlauf aufholen
                    next_deadline = time.perf_counter()
                if time_until_next > 0:
                    _tsl = time.perf_counter()
                    time.sleep(time_until_next)
                    prof["sleep"] += time.perf_counter() - _tsl
                prof["n"] += 1
                if self.profile and time.perf_counter() - prof_t0 >= 2.0:
                    el = time.perf_counter() - prof_t0
                    n = prof["n"]
                    self.get_logger().info(
                        f"PROF {n/el:.0f} Hz | simtime={self.data.time:.2f}s | pro Schritt us: step={prof['step']/n*1e6:.0f} pub={prof['pub']/n*1e6:.0f} sync={prof['sync']/n*1e6:.0f} sleep={prof['sleep']/n*1e6:.0f} gesamt={el/n*1e6:.0f} | z={self.data.qpos[2]:.3f} ncmd={getattr(self,'_prof_ncmd',0)} maxgap={getattr(self,'_prof_gap',0.0)*1e3:.0f}ms")
                    self._prof_ncmd = 0
                    self._prof_gap = 0.0
                    prof = {"step": 0.0, "pub": 0.0, "sync": 0.0, "sleep": 0.0, "n": 0}
                    prof_t0 = time.perf_counter()

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

        # Odometrie (Ground-Truth): Pose im odom-Frame, Twist im Koerper-Frame (ROS-Konvention)
        quat = self.data.qpos[3:7]
        if self.pub_odom is not None:
            od = Odometry()
            od.header.stamp = now
            od.header.frame_id = self.odom_frame
            od.child_frame_id = self.base_frame
            od.pose.pose.position.x = float(self.data.qpos[0])
            od.pose.pose.position.y = float(self.data.qpos[1])
            od.pose.pose.position.z = float(self.data.qpos[2])
            od.pose.pose.orientation.w = float(quat[0])
            od.pose.pose.orientation.x = float(quat[1])
            od.pose.pose.orientation.y = float(quat[2])
            od.pose.pose.orientation.z = float(quat[3])
            v_body = rotate_inverse(quat, self.data.qvel[0:3])
            od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = map(float, v_body)
            od.twist.twist.angular.x = float(self.data.qvel[3])    # MuJoCo: Winkelgeschw. im Koerper-Frame
            od.twist.twist.angular.y = float(self.data.qvel[4])
            od.twist.twist.angular.z = float(self.data.qvel[5])
            self.pub_odom.publish(od)
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(self.data.qpos[0])
            t.transform.translation.y = float(self.data.qpos[1])
            t.transform.translation.z = float(self.data.qpos[2])
            t.transform.rotation.w = float(quat[0])
            t.transform.rotation.x = float(quat[1])
            t.transform.rotation.y = float(quat[2])
            t.transform.rotation.z = float(quat[3])
            self.tf_broadcaster.sendTransform(t)

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
