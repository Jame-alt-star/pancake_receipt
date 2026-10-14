import math
import os
import threading
import time
from collections import deque
import yaml
import numpy as np

import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
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
    Simulations-Knoten (Layer 4): laedt die MuJoCo-Szene, rechnet Physik + Low-Level-PD-Regelung
    in einem Hintergrund-Thread und kommuniziert ausschliesslich ueber ROS2-Topics:
      raus: /joint_states, /imu (100 Hz), /odom + TF odom->base_link (Ground-Truth, abschaltbar),
            /base_position (Ground-Truth, nur fuer Tests/Auswertung), /scan bzw. /points (Lidar),
            TF base_link->lidar_link (statisch)
      rein: /joint_commands
    Kennt selbst keine RL-Policy - reine Physik und Sensormodell.
    """

    def __init__(self):
        super().__init__("mujoco_bridge_node")

        # 1. Config laden (gleiche Datei wie steuereinheit_node.py, damit
        # z.B. simulation_dt/kps/kds an beiden Stellen zusammenpassen).
        # NEU (DOF-Migration): Dateiname ueber den ROS2-Parameter
        # "config_file" waehlbar (Standard seit 14.10.2026 g1_policy_29dof.yaml; 12-DOF: g1_policy.yaml - vorher
        # das bisherige 12-DOF-Verhalten unveraendert), siehe
        # steuereinheit_node.py fuer denselben Mechanismus und
        # steuereinheit.launch.py fuer das Launch-Argument, das beide
        # Knoten synchron umschaltet.
        self.declare_parameter("config_file", "g1_policy_29dof.yaml")
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
        # NEU (13.10.2026): scene_file ueberschreibt xml_path der Config (z.B. Testwelt mit Waenden)
        self.declare_parameter("scene_file", "")
        scene_file = self.get_parameter("scene_file").value
        xml_rel_path = scene_file or self.config["xml_path"]
        xml_path = os.path.join(pkg_share, xml_rel_path)
        if not os.path.exists(xml_path):
            # Kein hartkodierter Rueckfall auf ~/humanoid_robot mehr (Review Kat. 6): klare Meldung.
            raise FileNotFoundError(f"MuJoCo-Szene nicht gefunden: {xml_path} - 'colcon build' ausgefuehrt?")

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
        # /base_position: wahre Basisposition (Simulations-Ground-Truth, qpos[0:3]). Seit 14.10.2026
        # nur noch fuer Tests/Auswertung - alle Positionsnutzer lesen /odom (Ground-Truth oder
        # Beinodometrie). Auf echter Hardware gibt es dieses Topic nicht.
        self.pub_base_position = self.create_publisher(Point, "/base_position", 10)
        # Realistische Sensorik (Schalter realistic_sensors, 14.10.2026): Rauschen auf Gelenken/IMU,
        # konstanter Gyro-Bias, Gier-Drift, Latenz in beide Richtungen (Zustaende und Kommandos),
        # Lidar-Rauschen und (2D-Scan) Ausfaelle. Die Werte sind ANNAHMEN in plausibler Groesse
        # (typische Encoder/MEMS-IMU/DDS), nicht aus einem G1-Datenblatt. Gemessen (synchroner Kreis):
        # die Policy bleibt bis zum 10-Fachen dieser Werte plus 20 ms Latenz stabil.
        # Die Position (/odom) ist davon unabhaengig: Ground-Truth oder Beinodometrie (odom_quelle).
        self.declare_parameter("realistic_sensors", False)
        self.declare_parameter("noise_joint_pos", 0.001)     # rad
        self.declare_parameter("noise_joint_vel", 0.02)      # rad/s
        self.declare_parameter("noise_gyro", 0.01)           # rad/s
        self.declare_parameter("gyro_bias", 0.005)           # rad/s (Std. des konstanten Bias)
        self.declare_parameter("noise_orientation", 0.005)   # rad
        self.declare_parameter("sensor_latency_ms", 10.0)
        self.declare_parameter("command_latency_ms", 10.0)
        self.declare_parameter("lidar_noise_m", 0.02)
        self.declare_parameter("lidar_dropout", 0.01)
        self.declare_parameter("noise_seed", 0)
        self.declare_parameter("yaw_drift_deg_per_min", 0.5)   # langsame Gier-Drift der IMU-Orientierung
        self.realistic = bool(self.get_parameter("realistic_sensors").value)
        self._rng = np.random.default_rng(int(self.get_parameter("noise_seed").value))
        gp = lambda n: float(self.get_parameter(n).value)
        self.n_q, self.n_dq, self.n_gyro = gp("noise_joint_pos"), gp("noise_joint_vel"), gp("noise_gyro")
        self.n_ori, self.n_lidar, self.p_drop = gp("noise_orientation"), gp("lidar_noise_m"), gp("lidar_dropout")
        self.gyro_bias = self._rng.normal(0.0, gp("gyro_bias"), 3) if self.realistic else np.zeros(3)
        self.sensor_latency = gp("sensor_latency_ms") / 1000.0 if self.realistic else 0.0
        self.command_latency = gp("command_latency_ms") / 1000.0 if self.realistic else 0.0
        self.yaw_drift_rate = np.radians(gp("yaw_drift_deg_per_min")) / 60.0 if self.realistic else 0.0
        self._state_buf = deque()      # (sim_time, q, dq, quat, omega)
        self._cmd_buf = deque()        # (sim_time_ankunft, cmd)

        # NEU (14.10.2026): Aufzeichnung der Roboterpose fuer Demo-Videos (Offline-Rendering). Schreibt
        # (Wandzeit, Simzeit, qpos) mit 25 Hz in eine .npz-Datei; die Simulation bleibt in Echtzeit.
        self.declare_parameter("record_path", "")
        self.record_path = str(self.get_parameter("record_path").value)
        self._rec = {"wall": [], "t": [], "qpos": []}
        self._rec_next = 0.0
        self._rec_flushed = time.time()
        # Viewer-Bildrate: alle render_decimation Physikschritte (25 -> 20 Hz). headless=True ->
        # kein Fenster (spart CPU/RAM, Pflicht mit Nav2+SLAM bei ~2,8 GB RAM).
        self.declare_parameter("render_decimation", 25)
        self.render_decimation = max(1, int(self.get_parameter("render_decimation").value))
        self.declare_parameter("headless", False)
        self.headless = self.get_parameter("headless").value
        # /odom + TF odom->base_link fuer Nav2 (Ground-Truth, perfekt). Abschaltbar, wenn eine andere
        # Quelle sie liefert (odom_quelle:=beinodometrie -> beinodometrie_node).
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

        # Simuliertes Lidar (Raycast auf MuJoCo-Geoms der Gruppe 2 = Welt-Hindernisse). Frame lidar_link
        # an base_link (0,0,lidar_z), kippt mit dem Roboter wie ein am Torso montierter Sensor.
        # lidar_2d: eine Scanebene /scan (alter Modus); lidar_3d: Punktwolke /points (Standard in
        # mission.launch, /scan wird dann von roboter_navigation/punktwolke_zu_scan abgeleitet).
        # Takt: der Lidar-Zaehler laeuft in _publish_states (100 Hz bei simulation_dt 0.002 und
        # PUBLISH_DECIMATION 5); lidar_rate_hz bezieht sich auf diese 100 Hz.
        self.declare_parameter("lidar", False)   # an per Launch-Argument lidar:=true (Nav2/SLAM)
        self.declare_parameter("lidar_rate_hz", 10.0)
        self.declare_parameter("lidar_rays", 360)
        self.declare_parameter("lidar_range_max", 10.0)
        self.declare_parameter("lidar_z", 0.45)           # ueber dem Becken [m]
        self.declare_parameter("lidar_geomgroup", 2)
        self.lidar_on = bool(self.get_parameter("lidar").value)
        self.lidar_period = max(1, int(round(100.0 / float(self.get_parameter("lidar_rate_hz").value))))
        self.lidar_n = int(self.get_parameter("lidar_rays").value)
        self.lidar_rmax = float(self.get_parameter("lidar_range_max").value)
        self.lidar_z = float(self.get_parameter("lidar_z").value)
        self.lidar_mask = np.zeros(6, dtype=np.uint8)
        self.lidar_mask[int(self.get_parameter("lidar_geomgroup").value)] = 1
        self._lidar_count = 0
        # NEU (14.10.2026): optionales 3D-Lidar als PointCloud2 (/points, Frame lidar_link): 24 Ringe
        # (-40..+10 Grad) x 360 Azimutwerte, 5 Hz. Ground-Truth-Raycast wie der 2D-Scan.
        self.declare_parameter("lidar_3d", False)
        # lidar_2d=False: KEIN eigener 2D-Scan (wie ein echtes 3D-Lidar); /scan wird dann von
        # roboter_navigation/punktwolke_zu_scan aus der Punktwolke abgeleitet.
        self.declare_parameter("lidar_2d", True)
        self.lidar2d_on = bool(self.get_parameter("lidar_2d").value)
        self.declare_parameter("lidar_3d_rings", 24)
        self.declare_parameter("lidar_3d_elev_min_deg", -40.0)
        self.declare_parameter("lidar_3d_elev_max_deg", 10.0)
        self.lidar3d_on = bool(self.get_parameter("lidar_3d").value) and self.lidar_on
        self._cloud_count = 0
        if self.lidar3d_on:
            nr = int(self.get_parameter("lidar_3d_rings").value)
            el = np.radians(np.linspace(float(self.get_parameter("lidar_3d_elev_min_deg").value),
                                        float(self.get_parameter("lidar_3d_elev_max_deg").value), nr))
            az = np.linspace(-np.pi, np.pi, self.lidar_n, endpoint=False)
            E, A = np.meshgrid(el, az, indexing="ij")
            self.cloud_dirs_local = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)],
                                             axis=-1).reshape(-1, 3)
            self.pub_cloud = self.create_publisher(PointCloud2, "/points", 5)
        a = np.linspace(-np.pi, np.pi, self.lidar_n, endpoint=False)
        self.lidar_dirs_local = np.stack([np.cos(a), np.sin(a), np.zeros_like(a)], axis=1)
        if self.lidar_on and bool(self.get_parameter("lidar_2d").value):
            self.pub_scan = self.create_publisher(LaserScan, "/scan", 10)
        if self.lidar_on:
            # Lidar-Montage (base_link -> lidar_link) IMMER publizieren - im reinen 3D-Modus ebenso
            # (fehlte dort zunaechst: Nav2 konnte die Punktwolke nicht einordnen) und auch, wenn
            # odom->base_link vom Schaetzer kommt.
            st = TransformStamped()
            st.header.stamp = self.get_clock().now().to_msg()
            st.header.frame_id = self.base_frame
            st.child_frame_id = "lidar_link"
            st.transform.translation.z = self.lidar_z
            st.transform.rotation.w = 1.0
            self._static_tf = StaticTransformBroadcaster(self)
            self._static_tf.sendTransform(st)
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
            if self.command_latency > 0.0 and self.controller_connected:
                self._cmd_buf.append((float(self.data.time), cmd_data))   # wird im Sim-Thread verzoegert angewendet
            else:
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
        # Fix (07.10.): nur alle RENDER_DECIMATION Physikschritte rendern. Seit 14.10. 25 statt 10
        # (20 Hz Bild): viewer.sync() kostet ~16 ms (Software-GL unter WSLg) und loeste sporadische
        # Stuerze im Pipeline-Test aus. Weitere Massnahmen (13.10.): Publish nur 100 Hz, absolute
        # Zeitplanung (siehe unten).
        RENDER_DECIMATION = self.render_decimation
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
        # Profiling der Schleife (Parameter profile:=true): Zeitanteile Physik/Publish/Rendern/Warten,
        # alle 2 s geloggt. Bleibt dauerhaft drin, kostet ohne profile nur perf_counter-Aufrufe.
        prof = {"step": 0.0, "pub": 0.0, "sync": 0.0, "sleep": 0.0, "n": 0}
        prof_t0 = time.perf_counter()

        viewer_ctx = (_NullViewer() if self.headless
                      else mujoco.viewer.launch_passive(self.model, self.data))
        with viewer_ctx as viewer:
            # NEU: freie Kamera mit groesserem Abstand, direkt von hinten
            # (Laufrichtung beim Start +X), leicht
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
                while self._cmd_buf and self._cmd_buf[0][0] + self.command_latency <= self.data.time:
                    self.target_dof_pos = self._cmd_buf.popleft()[1]
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
                if self.record_path and self.data.time >= self._rec_next:
                    self._rec_next = self.data.time + 0.04
                    self._rec["wall"].append(time.time())
                    self._rec["t"].append(float(self.data.time))
                    self._rec["qpos"].append(self.data.qpos.copy())
                    if time.time() - self._rec_flushed > 15.0:
                        self._flush_recording()

                # Sensor-Zustaende publizieren: alle PUBLISH_DECIMATION Schritte (100 Hz), siehe oben.
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

        q, dq = self.data.qpos[7:].copy(), self.data.qvel[6:].copy()
        quat_m, omega = self.data.qpos[3:7].copy(), self.data.qvel[3:6].copy()
        if self.realistic:
            q, dq, quat_m, omega = self._sensor_model(q, dq, quat_m, omega)

        # Joint States
        js = JointState()
        js.header.stamp = now
        js.position = q.tolist()
        js.velocity = dq.tolist()
        self.pub_joint_states.publish(js)

        # IMU: direkt aus dem simulierten "Floating Base"-Zustand abgeleitet
        # (qpos[3:7] = Orientierungs-Quaternion [w,x,y,z], qvel[3:6] =
        # Winkelgeschwindigkeit) - KEIN simuliertes Sensormodell an einer
        # konkreten Koerperstelle (eine idealisierte IMU an der Basis; die 29-DOF-Policy erwartet
        # Basis-Drehrate und Schwerkraftrichtung). Mit realistic_sensors verrauscht (_sensor_model).
        imu = Imu()
        imu.header.stamp = now
        imu.orientation.w = float(quat_m[0])
        imu.orientation.x = float(quat_m[1])
        imu.orientation.y = float(quat_m[2])
        imu.orientation.z = float(quat_m[3])
        imu.angular_velocity.x = float(omega[0])
        imu.angular_velocity.y = float(omega[1])
        imu.angular_velocity.z = float(omega[2])
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

        if self.lidar_on:
            self._lidar_count += 1
            if self._lidar_count >= self.lidar_period:
                self._lidar_count = 0
                if self.lidar2d_on:
                    self._publish_scan(now, quat)
                if self.lidar3d_on:
                    self._cloud_count += 1
                    if self._cloud_count >= 2:                  # 5 Hz
                        self._cloud_count = 0
                        self._publish_cloud(now, quat)

    def _sensor_model(self, q, dq, quat, omega):
        """Latenz (Ringpuffer nach Sim-Zeit) + Rauschen/Bias, wie sie ein echter Sensor liefert."""
        t = float(self.data.time)
        self._state_buf.append((t, q, dq, quat, omega))
        while len(self._state_buf) > 1 and self._state_buf[1][0] <= t - self.sensor_latency:
            self._state_buf.popleft()
        _, q, dq, quat, omega = self._state_buf[0]
        r = self._rng
        q = q + r.normal(0.0, self.n_q, q.shape)
        dq = dq + r.normal(0.0, self.n_dq, dq.shape)
        omega = omega + self.gyro_bias + r.normal(0.0, self.n_gyro, 3)
        if self.yaw_drift_rate:
            # Gier-Drift um die Welt-z-Achse (vorne multipliziert): q_drift * q
            h = 0.5 * self.yaw_drift_rate * t
            w1, x1, y1, z1 = math.cos(h), 0.0, 0.0, math.sin(h)
            w2, x2, y2, z2 = quat
            quat = np.array([w1 * w2 - z1 * z2, w1 * x2 - z1 * y2, w1 * y2 + z1 * x2, w1 * z2 + z1 * w2])
        if self.n_ori > 0.0:
            v = r.normal(0.0, self.n_ori, 3)
            d = np.concatenate([[1.0], 0.5 * v])
            d /= np.linalg.norm(d)
            w1, x1, y1, z1 = quat
            w2, x2, y2, z2 = d
            quat = np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                             w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])
        return q, dq, quat, omega

    def _publish_cloud(self, now, quat):
        """3D-Lidar: ein Raycast je Ring x Azimut (mj_multiRay, nur Welt-Geoms der Gruppe 2), Treffer
        als PointCloud2 im lidar_link-Frame. Mit realistic_sensors Entfernungsrauschen."""
        w, x, y, z = quat
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        origin = self.data.qpos[0:3] + R @ np.array([0.0, 0.0, self.lidar_z])
        dirs = self.cloud_dirs_local @ R.T
        n = len(dirs)
        dist = np.zeros(n, dtype=np.float64)
        geomid = np.zeros(n, dtype=np.int32)
        mujoco.mj_multiRay(self.model, self.data, origin, dirs.reshape(-1), self.lidar_mask, True, -1,
                           geomid, dist, None, n, self.lidar_rmax)
        ok = (geomid >= 0) & (dist > 0.15) & (dist < self.lidar_rmax)
        d_ok = dist[ok]
        if self.realistic and len(d_ok):
            d_ok = d_ok + self._rng.normal(0.0, self.n_lidar, len(d_ok))
        pts = (self.cloud_dirs_local[ok] * d_ok[:, None]).astype(np.float32)     # im lidar_link-Frame
        msg = PointCloud2()
        msg.header.stamp = now
        msg.header.frame_id = "lidar_link"
        msg.height, msg.width = 1, int(len(pts))
        msg.fields = [PointField(name=nme, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, nme in enumerate(("x", "y", "z"))]
        msg.is_bigendian, msg.point_step, msg.row_step = False, 12, 12 * len(pts)
        msg.data = pts.tobytes()
        msg.is_dense = True
        self.pub_cloud.publish(msg)

    def _publish_scan(self, now, quat):
        """2D-Lidar (eine Ebene in der Koerper-xy-Ebene des Sensors) als LaserScan; kein Treffer = inf.
        Mit realistic_sensors Rauschen und Ausfaelle (inf)."""
        w, x, y, z = quat
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        origin = self.data.qpos[0:3] + R @ np.array([0.0, 0.0, self.lidar_z])
        dirs = self.lidar_dirs_local @ R.T
        n = self.lidar_n
        dist = np.zeros(n, dtype=np.float64)
        geomid = np.zeros(n, dtype=np.int32)
        mujoco.mj_multiRay(self.model, self.data, origin, dirs.reshape(-1), self.lidar_mask, True, -1,
                           geomid, dist, None, n, self.lidar_rmax)
        ranges = np.where((geomid >= 0) & (dist >= 0) & (dist < self.lidar_rmax), dist, np.inf)
        if self.realistic:
            ranges = ranges + self._rng.normal(0.0, self.n_lidar, n)
            ranges[self._rng.random(n) < self.p_drop] = np.inf     # Ausfaelle (keine Rueckmeldung)
        sc = LaserScan()
        sc.header.stamp = now
        sc.header.frame_id = "lidar_link"
        sc.angle_min = -float(np.pi)
        sc.angle_max = float(np.pi) - 2.0 * float(np.pi) / n
        sc.angle_increment = 2.0 * float(np.pi) / n
        sc.time_increment = 0.0
        sc.scan_time = self.lidar_period * 0.01
        sc.range_min = 0.15
        sc.range_max = self.lidar_rmax
        sc.ranges = ranges.astype(np.float32)   # numpy-Array statt Python-Liste (1.6 ms -> 0.06 ms)
        self.pub_scan.publish(sc)

    def _flush_recording(self):
        """Schreibt die Posenaufzeichnung (record_path) als .npz; alle 15 s und beim Beenden."""
        if not self.record_path or not self._rec["t"]:
            return
        self._rec_flushed = time.time()
        np.savez_compressed(self.record_path, wall=np.array(self._rec["wall"]), t=np.array(self._rec["t"]),
                            qpos=np.array(self._rec["qpos"]))

    def destroy_node(self):
        self._flush_recording()
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
