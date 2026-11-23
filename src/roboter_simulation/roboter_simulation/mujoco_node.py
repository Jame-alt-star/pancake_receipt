"""Simulations-Knoten (Layer 4): MuJoCo-Physik des G1 mit Low-Level-PD-Regelung und Sensorik.

Rechnet in einem Hintergrund-Thread (der Viewer blockiert) und spricht nur ueber ROS2-Topics:
  raus: /joint_states (mit Gelenknamen), /imu (100 Hz), /clock (Simulationszeit), /odom + TF odom->base_link
        (Ground-Truth, abschaltbar), /base_position (Ground-Truth, nur Tests/Auswertung), /scan bzw. /points (Lidar),
        /kamera/color|depth/image_raw + camera_info (RGBD-Kamera, abschaltbar).
        Die uebrigen Frames (Glieder, lidar_link, kamera_optical_frame) kommen aus
        roboter_beschreibung/launch/beschreibung.launch.py.
  rein: /joint_commands (Zielwinkel), /antrieb/parameter (kp, kd, Startpose von der Steuerung, latched)
Kennt keine Policy. Bausteine: szene.py (Roboter + Welt + Kamera), sensormodell.py, lidar.py, kamera.py, aufnahme.py.
Parameter: config/simulation.yaml; Robotermodell und Geometrie: roboter_beschreibung.

Ablauf beim Start: (1) warten auf die Antriebsparameter (nichts senden), (2) Ruhepose einnehmen und
Sensordaten senden, Physik pausiert, (3) mit dem ersten /joint_commands laeuft die Physik in Echtzeit.
Bleiben danach die Kommandos laenger als kommando_timeout_s aus (Steuerung ausgefallen oder in Stoerung), geht die
Simulation wie ein Motortreiber in den Daempfungsmodus (Steifigkeit 0, nur Daempfung): frueher hielt sie fuer immer
starr das letzte Ziel. Mit dem naechsten Kommando regelt sie wieder normal.
Schritt (2) ist noetig, weil der Regler ~7 s zum Hochfahren braucht und reine PD-Haltung auf der Ruhepose
nachweislich umkippt (13.10.2026: z=0.096 bei Simzeit 1.9 s, bevor die Policy ihr erstes Kommando senden konnte).
"""
import threading
import time
from collections import deque

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

from roboter_beschreibung import lade as lade_roboter
from roboter_interfaces.msg import Antriebsparameter
from roboter_simulation.aufnahme import Posenaufnahme
from roboter_simulation.kamera import KameraThread, intrinsik
from roboter_beschreibung.geometrie import rotate_inverse
from roboter_simulation.lidar import R_MIN, Lidar
from roboter_simulation.sensormodell import Sensormodell
from roboter_simulation.szene import lade_welt

# Sensordaten alle 5 Physikschritte (500/5 = 100 Hz). Gemessen 13.10.2026: das Publizieren kostet ~1.1 ms je
# Aufruf und liess die Simulation sonst nur mit ~0.5x Echtzeit laufen; die Policy laeuft mit 50 Hz.
PUBLISH_DECIMATION = 5


# Alle Parameter mit Typ-Standardwerten; die gueltigen Werte stehen in config/simulation.yaml, die
# simulation.launch.py immer laedt (tests/simulation/test_konfiguration.py prueft, dass dort nur Deklariertes steht).
PARAMETER = {
    "welt": "", "simulation_dt": 0.002, "wait_for_controller": True, "kommando_timeout_s": 1.0, "render_decimation": 25,
    "profile": False, "headless": False, "record_path": "",
    "publish_odom": True, "publish_tf": True, "odom_frame": "odom", "base_frame": "base_link",
    "realistic_sensors": False, "noise_joint_pos": 0.001, "noise_joint_vel": 0.02, "noise_gyro": 0.01,
    "gyro_bias": 0.005, "noise_orientation": 0.005, "yaw_drift_deg_per_min": 0.5,
    "sensor_latency_ms": 10.0, "command_latency_ms": 10.0, "lidar_noise_m": 0.02, "lidar_dropout": 0.01,
    "noise_seed": 0,
    "lidar": False, "lidar_2d": True, "lidar_3d": False, "lidar_rate_hz": 10.0, "lidar_rays": 360,
    "lidar_range_max": 10.0, "lidar_geomgroup": 2, "lidar_3d_rings": 24,
    "lidar_3d_elev_min_deg": -40.0, "lidar_3d_elev_max_deg": 10.0,
    "kamera": False, "kamera_rate_hz": 5.0, "kamera_tiefe_min_m": 0.2, "kamera_tiefe_max_m": 10.0,
    "kamera_tiefe_rauschen": 0.01,
}

KAMERA_FRAME = "kamera_optical_frame"      # Bilder im optical frame (z nach vorn), siehe beschreibung.launch.py


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Drehmoment je Gelenk = kp * Positionsfehler + kd * Geschwindigkeitsfehler (wie deploy_mujoco.py aus
    unitree_rl_gym); target_dq ist immer 0."""
    return (target_q - q) * kp + (target_dq - dq) * kd


class _NullViewer:
    """Fuer headless: gleiche Schnittstelle wie der passive MuJoCo-Viewer, ohne Fenster."""

    def __init__(self):
        from types import SimpleNamespace
        self.cam = SimpleNamespace(type=None, distance=0.0, azimuth=0.0, elevation=0.0, lookat=np.zeros(3))

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def is_running(self):
        return True

    def sync(self):
        pass


class MujocoBridgeNode(Node):

    def __init__(self):
        super().__init__("mujoco_bridge_node")
        p = self._parameter_lesen()

        # Modell: Roboter aus roboter_beschreibung + Welt aus roboter_welten ('' = nur Ebene) + Kamera (nur wenn an)
        self.roboter = lade_roboter()
        self.model = lade_welt(self.roboter["modell_xml"], p["welt"], self.roboter["kamera"] if p["kamera"] else None)
        self.model.opt.timestep = p["simulation_dt"]
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        # Gelenknamen in Aktuator-Reihenfolge (= Reihenfolge der Zielwinkel): robot_state_publisher baut daraus den
        # TF-Baum; ohne Namen konnte niemand ausser der Steuerung die Gelenkwerte zuordnen.
        self.gelenknamen = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.model.actuator_trnid[i, 0])
                            for i in range(self.model.nu)]
        # qpos/qvel-Indizes der angetriebenen Robotergelenke (Reihenfolge = gelenknamen). Nicht einfach qpos[7:]:
        # Welten mit Gelenken (Schubladen, Tueren, frei bewegliche Objekte, z. B. libero_kueche) haengen weitere
        # Eintraege hinten an.
        gelenke = self.model.actuator_trnid[:, 0]
        self._q_idx = self.model.jnt_qposadr[gelenke].copy()
        self._v_idx = self.model.jnt_dofadr[gelenke].copy()
        self.get_logger().info(f"[mujoco_bridge_node] Welt: {p['welt'] or '(keine, nur Ebene)'}")

        # Antrieb: kommt von der Steuerung (siehe Modul-Docstring)
        self.kps = self.kds = self.default_angles = None
        self.target_dof_pos = None
        self._antrieb_neu = None
        self.controller_connected = not p["wait_for_controller"]
        self.kommando_timeout_s = float(p["kommando_timeout_s"])
        self._t_kommando = None            # Empfangszeit (monotonic) des letzten Kommandos
        self.daempfung_aktiv = False
        self._cmd_buf = deque()        # (sim_zeit_ankunft, cmd) fuer die Kommando-Latenz

        # Sensorik
        self._rng = np.random.default_rng(p["noise_seed"])
        self.realistic = p["realistic_sensors"]
        self.sensormodell = None
        self.command_latency = 0.0
        if self.realistic:
            self.sensormodell = Sensormodell(
                self._rng, p["noise_joint_pos"], p["noise_joint_vel"], p["noise_gyro"], p["gyro_bias"],
                p["noise_orientation"], p["sensor_latency_ms"] / 1000.0, np.radians(p["yaw_drift_deg_per_min"]) / 60.0)
            self.command_latency = p["command_latency_ms"] / 1000.0
        self.n_lidar, self.p_drop = p["lidar_noise_m"], p["lidar_dropout"]

        self.aufnahme = Posenaufnahme(p["record_path"])
        self.render_decimation = max(1, p["render_decimation"])
        self.headless, self.profile = p["headless"], p["profile"]
        self.odom_frame, self.base_frame = p["odom_frame"], p["base_frame"]

        # Publisher / Subscriber
        self.pub_joint_states = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_imu = self.create_publisher(Imu, "/imu", 10)
        # Simulationszeit auf /clock (100 Hz): Tests messen damit Geschwindigkeiten unabhaengig davon, ob die Simulation
        # unter Last langsamer als Echtzeit laeuft. Die Knoten selbst nutzen weiter die Wanduhr (use_sim_time aus).
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        # /base_position: wahre Basisposition (Ground-Truth) nur fuer Tests/Auswertung; alle Positionsnutzer
        # lesen /odom. Auf echter Hardware gibt es dieses Topic nicht.
        self.pub_base_position = self.create_publisher(Point, "/base_position", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10) if p["publish_odom"] else None
        self.tf_broadcaster = TransformBroadcaster(self) if p["publish_tf"] else None
        self._lidar_einrichten(p)
        self._kamera_einrichten(p)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(Antriebsparameter, "/antrieb/parameter", self._antrieb_callback, latched)
        self.create_subscription(Float32MultiArray, "/joint_commands", self._joint_commands_callback, 10)

        # Simulation im Hintergrundthread (der Viewer blockiert; rclpy.spin muss parallel laufen)
        self.is_running = True
        self.sim_thread = threading.Thread(target=self._run_simulation, daemon=True)
        self.sim_thread.start()

    def _parameter_lesen(self) -> dict:
        for name, wert in PARAMETER.items():
            self.declare_parameter(name, wert)
        return {name: self.get_parameter(name).value for name in PARAMETER}

    def _lidar_einrichten(self, p):
        """Lidar (Raycast auf Welt-Geoms der Gruppe 2). Takt: Zaehler im 100-Hz-Publish; lidar_rate_hz bezieht
        sich darauf, die 3D-Wolke kommt jede zweite Runde (5 Hz bei 10 Hz)."""
        self.lidar = None
        self.lidar2d_on = self.lidar3d_on = False
        if not p["lidar"]:
            return
        self.lidar2d_on, self.lidar3d_on = bool(p["lidar_2d"]), bool(p["lidar_3d"])
        # Montage (Elternglied + Pose) aus roboter_beschreibung - dieselbe Quelle wie der TF-Baum
        self.lidar = Lidar(self.model, self.roboter["lidar"], int(p["lidar_rays"]), float(p["lidar_range_max"]),
                           int(p["lidar_geomgroup"]), ringe=int(p["lidar_3d_rings"]) if self.lidar3d_on else 0,
                           elev_min_deg=p["lidar_3d_elev_min_deg"], elev_max_deg=p["lidar_3d_elev_max_deg"])
        self.lidar_period = max(1, int(round(100.0 / float(p["lidar_rate_hz"]))))
        self._lidar_count = self._cloud_count = 0
        if self.lidar2d_on:
            self.pub_scan = self.create_publisher(LaserScan, "/scan", 10)
        if self.lidar3d_on:
            self.pub_cloud = self.create_publisher(PointCloud2, "/points", 5)

    def _kamera_einrichten(self, p):
        """RGBD-Kamera: rendert in einem eigenen Thread (kamera.py), Takt wie beim Lidar ueber den 100-Hz-Publish."""
        self.kamera = None
        if not p["kamera"]:
            return
        m = self.roboter["kamera"]
        breite, hoehe = int(m["breite"]), int(m["hoehe"])
        fx, fy, cx, cy = intrinsik(breite, hoehe, float(m["fovy_grad"]))
        # CameraInfo: ideale Lochkamera ohne Verzeichnung; Farbe und Tiefe aus derselben Kamera (Tiefe pixelgenau
        # zum Farbbild ausgerichtet, wie aligned_depth_to_color beim RealSense-Treiber).
        self._kamera_info = CameraInfo()
        self._kamera_info.header.frame_id = KAMERA_FRAME
        self._kamera_info.width, self._kamera_info.height = breite, hoehe
        self._kamera_info.distortion_model = "plumb_bob"
        self._kamera_info.d = [0.0] * 5
        self._kamera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self._kamera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self._kamera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        # Bilder sind gross (Farbe 230 KB, Tiefe 300 KB bei 320x240): kurze Warteschlange, alte Bilder sind wertlos.
        self.pub_farbe = self.create_publisher(Image, "/kamera/color/image_raw", 2)
        self.pub_farbe_info = self.create_publisher(CameraInfo, "/kamera/color/camera_info", 2)
        self.pub_tiefe = self.create_publisher(Image, "/kamera/depth/image_raw", 2)
        self.pub_tiefe_info = self.create_publisher(CameraInfo, "/kamera/depth/camera_info", 2)
        self.kamera_period = max(1, int(round(100.0 / float(p["kamera_rate_hz"]))))
        self._kamera_count = 0
        # Eigener Zufallsgenerator: numpy-Generatoren sind nicht threadsicher, _rng nutzt der Simulationsthread.
        rng = np.random.default_rng(int(p["noise_seed"]) + 1) if self.realistic else None
        # Wartet, bis die Kamera angelegt ist (~1 s; haelt den GIL, siehe kamera.py) - vor dem Start der Simulation.
        self.kamera = KameraThread(self.model, self._publish_kamera, float(p["kamera_tiefe_min_m"]),
                                   float(p["kamera_tiefe_max_m"]), rng, float(p["kamera_tiefe_rauschen"]),
                                   log_fehler=lambda t: self.get_logger().error(f"[mujoco_bridge_node] {t}"))
        self.get_logger().info(f"[mujoco_bridge_node] RGBD-Kamera {breite}x{hoehe} an {m['parent']}, "
                               f"{100.0 / self.kamera_period:.0f} Hz.")

    # ---------------------------------------------------------------- Eingaenge
    def _antrieb_callback(self, msg: Antriebsparameter):
        n = self.model.nu
        if not (len(msg.kp) == len(msg.kd) == len(msg.startpose) == n):
            self.get_logger().error(f"Antriebsparameter mit falscher Laenge ignoriert (erwartet {n} Gelenke).")
            return
        # Uebernahme im Simulationsthread (dort wird qpos gesetzt), hier nur ablegen.
        self._antrieb_neu = (np.array(msg.kp, dtype=np.float32), np.array(msg.kd, dtype=np.float32),
                             np.array(msg.startpose, dtype=np.float32))

    def _uebernehme_antrieb(self):
        """Antriebsparameter anwenden; beim ersten Mal den Roboter in die Ruhepose der Policy stellen."""
        kps, kds, start = self._antrieb_neu
        self._antrieb_neu = None
        erstes_mal = self.kps is None
        self.kps, self.kds, self.default_angles = kps, kds, start
        if erstes_mal:
            self.target_dof_pos = start.copy()
            self.data.qpos[self._q_idx] = start
            mujoco.mj_forward(self.model, self.data)
            self.get_logger().info("[mujoco_bridge_node] Antriebsparameter von der Steuerung erhalten.")

    def _joint_commands_callback(self, msg: Float32MultiArray):
        cmd = np.array(msg.data, dtype=np.float32)
        jetzt = time.perf_counter()
        if getattr(self, "_prof_last_cmd", None) is not None:
            self._prof_gap = max(getattr(self, "_prof_gap", 0.0), jetzt - self._prof_last_cmd)
        self._prof_last_cmd = jetzt
        self._prof_ncmd = getattr(self, "_prof_ncmd", 0) + 1
        # Laengen-Check: eine falsch dimensionierte Nachricht wird ignoriert statt die Simulation abzubrechen.
        if self.target_dof_pos is None or len(cmd) != len(self.target_dof_pos):
            return
        self._t_kommando = time.monotonic()
        if self.command_latency > 0.0 and self.controller_connected:
            self._cmd_buf.append((float(self.data.time), cmd))   # im Sim-Thread verzoegert angewendet
        else:
            self.target_dof_pos = cmd
        if not self.controller_connected:
            self.controller_connected = True
            self.get_logger().info("[mujoco_bridge_node] Regler verbunden - Physik gestartet.")

    # ---------------------------------------------------------------- Simulationsschleife
    def _run_simulation(self):
        try:
            self._schleife()
        except Exception:
            # Beim Beenden macht rclpy den Kontext ungueltig, bevor dieser Thread anhaelt; ein Publish wirft dann.
            # Das ist kein Fehler. Waehrend des Laufs dagegen schon - dann weiterreichen.
            if self.is_running and rclpy.ok():
                raise

    def _schleife(self):
        # Viewer nur alle render_decimation Physikschritte (25 -> 20 Hz): viewer.sync() kostet unter WSLg ~16 ms;
        # bei jedem Schritt lief die Schleife nur mit ~90 Hz statt 500 Hz, die Physik driftete gegen den 50-Hz-
        # Regler weg und der Roboter fiel (07.10.); 10 statt 25 loeste noch sporadische Stuerze aus (14.10.).
        render_counter = publish_counter = 0
        # Absolute Zeitplanung: Verzoegerungen eines Schritts (Viewer, sleep-Overshoot) werden durch kuerzeres
        # Warten der folgenden Schritte aufgeholt statt sich zu summieren.
        next_deadline = time.perf_counter()
        prof = {"step": 0.0, "pub": 0.0, "sync": 0.0, "sleep": 0.0, "n": 0}
        prof_t0 = time.perf_counter()

        viewer_ctx = _NullViewer() if self.headless else mujoco.viewer.launch_passive(self.model, self.data)
        with viewer_ctx as viewer:
            # freie Kamera von hinten, leicht erhoeht; lookat folgt dem Roboter (Abstand/Winkel per Maus frei)
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 5.0, 180.0, -20.0
            viewer.cam.lookat[:] = self.data.qpos[0:3]

            while viewer.is_running() and self.is_running:
                if self._antrieb_neu is not None:
                    self._uebernehme_antrieb()
                if self.kps is None or not self.controller_connected:
                    # (1) ohne Antriebsparameter nichts senden; (2) danach Pose halten, Sensordaten senden
                    if self.kps is not None:
                        self._publish_states()
                    viewer.sync()
                    time.sleep(0.01)
                    next_deadline = time.perf_counter()
                    continue

                # PD-Regelung auf die angetriebenen Gelenke (_q_idx/_v_idx: nach der freien Basis, ohne Weltgelenke)
                while self._cmd_buf and self._cmd_buf[0][0] + self.command_latency <= self.data.time:
                    self.target_dof_pos = self._cmd_buf.popleft()[1]
                self._pruefe_kommando_timeout()
                kp = np.zeros_like(self.kps) if self.daempfung_aktiv else self.kps
                self.data.ctrl[:] = pd_control(self.target_dof_pos, self.data.qpos[self._q_idx], kp,
                                               np.zeros_like(self.kds), self.data.qvel[self._v_idx], self.kds)
                _t0 = time.perf_counter()
                mujoco.mj_step(self.model, self.data)
                _t1 = time.perf_counter()
                self.aufnahme.schritt(self.data.time, self.data.qpos)

                publish_counter += 1
                if publish_counter >= PUBLISH_DECIMATION:
                    self._publish_states()
                    publish_counter = 0
                _t2 = time.perf_counter()
                prof["step"] += _t1 - _t0
                prof["pub"] += _t2 - _t1

                render_counter += 1
                if render_counter >= self.render_decimation:
                    viewer.cam.lookat[:] = self.data.qpos[0:3]
                    _ts = time.perf_counter()
                    viewer.sync()
                    prof["sync"] += time.perf_counter() - _ts
                    render_counter = 0

                # Echtzeit: nicht schneller als der Physiktakt; > 50 ms Rueckstand (z. B. Fenster verschoben)
                # nicht im Schnelldurchlauf aufholen
                next_deadline += self.model.opt.timestep
                warten = next_deadline - time.perf_counter()
                if warten < -0.05:
                    next_deadline = time.perf_counter()
                if warten > 0:
                    _tsl = time.perf_counter()
                    time.sleep(warten)
                    prof["sleep"] += time.perf_counter() - _tsl
                prof["n"] += 1
                if self.profile and time.perf_counter() - prof_t0 >= 2.0:
                    self._profil_melden(prof, time.perf_counter() - prof_t0)
                    prof = {"step": 0.0, "pub": 0.0, "sync": 0.0, "sleep": 0.0, "n": 0}
                    prof_t0 = time.perf_counter()

    def _pruefe_kommando_timeout(self):
        """Daempfungsmodus ein/aus je nach Alter des letzten Kommandos (siehe Modul-Docstring)."""
        if self.kommando_timeout_s <= 0.0 or self._t_kommando is None:
            return
        alt = time.monotonic() - self._t_kommando > self.kommando_timeout_s
        if alt and not self.daempfung_aktiv:
            self.get_logger().error(f"[mujoco_bridge_node] Kein Kommando seit > {self.kommando_timeout_s:.1f} s - "
                                    f"Daempfungsmodus (Steifigkeit 0).")
        elif not alt and self.daempfung_aktiv:
            self.get_logger().info("[mujoco_bridge_node] Kommandos wieder da - normale Regelung.")
        self.daempfung_aktiv = alt

    def _profil_melden(self, prof, dauer):
        """Zeitanteile je Physikschritt (Parameter profile:=true), alle 2 s."""
        n = prof["n"]
        self.get_logger().info(
            f"PROF {n / dauer:.0f} Hz | simtime={self.data.time:.2f}s | pro Schritt us: "
            f"step={prof['step'] / n * 1e6:.0f} pub={prof['pub'] / n * 1e6:.0f} sync={prof['sync'] / n * 1e6:.0f} "
            f"sleep={prof['sleep'] / n * 1e6:.0f} gesamt={dauer / n * 1e6:.0f} | z={self.data.qpos[2]:.3f} "
            f"ncmd={getattr(self, '_prof_ncmd', 0)} maxgap={getattr(self, '_prof_gap', 0.0) * 1e3:.0f}ms")
        self._prof_ncmd = 0
        self._prof_gap = 0.0

    # ---------------------------------------------------------------- Ausgaenge
    def _publish_states(self):
        now = self.get_clock().now().to_msg()
        uhr = Clock()
        uhr.clock.sec = int(self.data.time)
        uhr.clock.nanosec = int((self.data.time - int(self.data.time)) * 1e9)
        self.pub_clock.publish(uhr)
        q, dq = self.data.qpos[self._q_idx], self.data.qvel[self._v_idx]      # Indexzugriff liefert Kopien
        quat_m, omega = self.data.qpos[3:7].copy(), self.data.qvel[3:6].copy()
        if self.sensormodell is not None:
            q, dq, quat_m, omega = self.sensormodell.anwenden(float(self.data.time), q, dq, quat_m, omega)

        js = JointState()
        js.header.stamp = now
        js.name = self.gelenknamen
        js.position = q.tolist()
        js.velocity = dq.tolist()
        self.pub_joint_states.publish(js)

        # Idealisierte IMU an der Basis (Orientierung + Drehrate der freien Basis); die Policy erwartet
        # Basis-Drehrate und Schwerkraftrichtung.
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "imu_in_pelvis"          # Glied der Unitree-URDF (gleiche Ausrichtung wie das Becken)
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = map(float, quat_m)
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, omega)
        self.pub_imu.publish(imu)

        pos = Point()
        pos.x, pos.y, pos.z = map(float, self.data.qpos[0:3])
        self.pub_base_position.publish(pos)

        quat = self.data.qpos[3:7]
        if self.pub_odom is not None:
            # Ground-Truth: Pose im odom-Frame, Twist im Koerper-Frame (ROS-Konvention)
            od = Odometry()
            od.header.stamp = now
            od.header.frame_id, od.child_frame_id = self.odom_frame, self.base_frame
            pp, oo = od.pose.pose.position, od.pose.pose.orientation
            pp.x, pp.y, pp.z = map(float, self.data.qpos[0:3])
            oo.w, oo.x, oo.y, oo.z = map(float, quat)
            tl, ta = od.twist.twist.linear, od.twist.twist.angular
            tl.x, tl.y, tl.z = map(float, rotate_inverse(quat, self.data.qvel[0:3]))
            ta.x, ta.y, ta.z = map(float, self.data.qvel[3:6])     # MuJoCo: bereits im Koerper-Frame
            od.pose.covariance[::7] = [1e-6] * 6                   # Ground-Truth: praktisch exakt (nicht 0)
            od.twist.covariance[::7] = [1e-6] * 6
            self.pub_odom.publish(od)
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id, t.child_frame_id = self.odom_frame, self.base_frame
            tt, tr = t.transform.translation, t.transform.rotation
            tt.x, tt.y, tt.z = map(float, self.data.qpos[0:3])
            tr.w, tr.x, tr.y, tr.z = map(float, quat)
            self.tf_broadcaster.sendTransform(t)

        if self.lidar is not None:
            self._lidar_count += 1
            if self._lidar_count >= self.lidar_period:
                self._lidar_count = 0
                if self.lidar2d_on:
                    self._publish_scan(now)
                if self.lidar3d_on:
                    self._cloud_count += 1
                    if self._cloud_count >= 2:
                        self._cloud_count = 0
                        self._publish_cloud(now)

        if self.kamera is not None:
            self._kamera_count += 1
            if self._kamera_count >= self.kamera_period:
                self._kamera_count = 0
                self.kamera.aufnehmen(self.data.qpos, now)      # nur qpos kopieren; gerendert wird im Kamerathread

    def _rauschen(self):
        """(rng, sigma, p_ausfall) fuer das Lidar: nur mit realistic_sensors."""
        return (self._rng, self.n_lidar, self.p_drop) if self.realistic else (None, 0.0, 0.0)

    def _publish_cloud(self, now):
        pts = self.lidar.wolke(self.data, *self._rauschen())
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

    def _publish_scan(self, now):
        ranges = self.lidar.scan(self.data, *self._rauschen())
        n = self.lidar.n
        sc = LaserScan()
        sc.header.stamp = now
        sc.header.frame_id = "lidar_link"
        sc.angle_min = -float(np.pi)
        sc.angle_max = float(np.pi) - 2.0 * float(np.pi) / n
        sc.angle_increment = 2.0 * float(np.pi) / n
        sc.time_increment = 0.0
        sc.scan_time = self.lidar_period * 0.01
        sc.range_min = R_MIN
        sc.range_max = self.lidar.rmax
        sc.ranges = ranges.astype(np.float32)   # numpy-Array statt Python-Liste (1.6 ms -> 0.06 ms)
        self.pub_scan.publish(sc)

    def _bild(self, arr, encoding, bytes_je_pixel, stempel) -> Image:
        msg = Image()
        msg.header.stamp = stempel
        msg.header.frame_id = KAMERA_FRAME
        msg.height, msg.width = int(arr.shape[0]), int(arr.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = msg.width * bytes_je_pixel
        msg.data = arr.tobytes()
        return msg

    def _publish_kamera(self, rgb, tiefe, stempel):
        """Aufruf aus dem Kamerathread. Zeitstempel = Zeitpunkt der Aufnahme (Zustand), nicht das Ende des Renderns.
        Tiefe als 32FC1 in Metern, ungueltig = NaN (REP 118)."""
        self._kamera_info.header.stamp = stempel
        self.pub_farbe.publish(self._bild(rgb, "rgb8", 3, stempel))
        self.pub_farbe_info.publish(self._kamera_info)
        self.pub_tiefe.publish(self._bild(tiefe, "32FC1", 4, stempel))
        self.pub_tiefe_info.publish(self._kamera_info)

    def destroy_node(self):
        # Erst den Simulationsthread anhalten und abwarten (Review B8), dann den Kamerathread (beide publizieren), dann
        # die Aufnahme schreiben (sonst Race) und den Knoten zerstoeren (sonst publiziert ein Thread noch auf einen
        # zerstoerten Knoten).
        self.is_running = False
        if self.sim_thread.is_alive():
            self.sim_thread.join(timeout=2.0)
        if self.kamera is not None:
            self.kamera.stoppen()
        self.aufnahme.schreiben()
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
