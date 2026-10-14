import os
import time
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from roboter_simulation.bridges.mujoco_bridge import MujocoBridge
from roboter_steuerung.states.start_state import StartState
from roboter_steuerung.states.bereitschaft_state import BereitschaftState
from roboter_steuerung.states.not_aus_state import NotAusState


# NEU (12.10.2026): maximale Aenderungsrate von self.cmd pro Sekunde (siehe
# _ramp_cmd() unten), fuer alle drei Kommando-Achsen (vx/vy/yaw_rate)
# gemeinsam verwendet. Bewusst ein einzelner, noch nicht empirisch
# feinabgestimmter Startwert (aehnlich CMD_NULL_SCHWELLE in den
# States) - 1.0 heisst z.B., ein Wechsel von vx 0.5 auf 0 dauert ca. 0.5 s
# statt einem einzigen Regelungstakt. In Bereitschaft/NotAus steht die Rampe auf 0.
CMD_RAMP_RATE = 1.0

# NEU (12.10.2026): Heading-Hold/Positions-Haltung - siehe
# _apply_heading_hold()/_apply_position_hold() fuer die Begruendung
# (Policy haelt selbst keine globale Richtung/Position, driftet daher
# sowohl beim Gehen als auch beim Stehen). *_CMD_SCHWELLE: unterhalb
# dieses Werts im ANGEFORDERTEN Kommando (cmd_target) gilt "keine
# explizite Bewegung in diese Achse gewuenscht" - die jeweilige Haltung
# greift dann ein. Bewusst derselbe Wert wie CMD_NULL_SCHWELLE in den
# States, aus Konsistenzgruenden hier aber separat benannt (konzeptionell
# unabhaengig).
# GEAENDERT (14.10.2026, Review L2): vorher 0.05. Nav2 (RPP) gibt in sanften Kurven oft kleine wz
# (< 0.05) aus; die Kurshaltung hielt dann den alten Kurs und regelte gegen den Pfadfolger. Jetzt
# greift sie nur, wenn praktisch keine Drehung angefordert ist (Handbetrieb, Geradeausfahrt).
YAW_CMD_SCHWELLE = 0.001
LIN_CMD_SCHWELLE = 0.05
# P-Verstaerkungen: wie stark ein Fehler (rad bzw. m) in eine korrigierende
# Rate (rad/s bzw. m/s) uebersetzt wird. Noch nicht empirisch abgestimmt,
# bewusst vorsichtig/klein gewaehlt.
HEADING_HOLD_KP = 1.0
POSITION_HOLD_KP = 1.0
# Begrenzung der Korrekturen selbst, damit ein grosser Fehler (z.B. durch
# einen Mess-Ausreisser) nicht zu einer unrealistisch harten Korrektur
# fuehrt, die die Policy destabilisieren koennte - bewusst klein
# gegenueber echten Bewegungs-Kommandos.
HEADING_HOLD_MAX_YAW_RATE = 0.3
POSITION_HOLD_MAX_SPEED = 0.15


class SteuereinheitNode(Node):
    """
    Zentraler Regelungs-Knoten (Layer 3): taktet die Regelschleife mit
    50 Hz, holt sich bei jedem Takt die aktuellen Sensordaten ueber die
    Hardware-Bridge, laesst die RL-Policy daraus Ziel-Gelenkwinkel
    berechnen und schickt diese zurueck an die Bridge. Kennt selbst
    weder MuJoCo noch die neuronale Netz-Architektur - beides steckt
    hinter den beiden Objekten self.bridge/self.controller, austauschbar
    ueber IRobotHardwareBridge bzw. den RL-Controller selbst. Das
    Geschwindigkeitskommando (self.cmd) kommt zur Laufzeit per
    Standard-ROS2-Topic /cmd_vel (geometry_msgs/Twist) herein, z.B. von
    einem Joystick-Node, einer Tastatursteuerung oder spaeter Nav2 -
    dieser Knoten selbst weiss nicht, WOHER das Kommando stammt.

    NEU (12.10.2026): drei getrennte Kommando-Werte statt einem.
    self.cmd_target ist das zuletzt per /cmd_vel angeforderte Ziel (reiner
    Nutzerwunsch, unveraendert). self.cmd ist der tatsaechlich an die
    Policy uebergebene Wert: zuerst sanft an cmd_target angenaehert
    (_ramp_cmd()), danach ggf. um zwei Korrekturen ergaenzt:
    _apply_heading_hold() (Drehrate, cmd[2]) und _apply_position_hold()
    (vx/vy, cmd[0]/cmd[1]). Hintergrund fuer beide: sowohl beim Gehen als
    auch beim Stehen wurde seitlicher Drift beobachtet (mal als Drehung,
    mal als reine seitliche Verschiebung ohne Drehung) - die Policy
    selbst haelt weder eine globale Ausrichtung noch eine Position,
    cmd gibt ihr nur eine koerperbezogene Soll-Geschwindigkeit vor.
    self.target_yaw/self.target_position sind die jeweils aktuell
    gehaltenen Sollwerte.

    Zustandswechsel-Entscheidungen (siehe gehen_state.py/
    balancieren_state.py) pruefen bewusst gegen self.cmd_target, nicht
    gegen self.cmd - sonst wuerden die Haltungs-Korrekturen faelschlich
    als "Nutzer will sich bewegen" interpretiert und einen
    Zustandswechsel auslösen, obwohl gar kein echtes Kommando vorliegt.

    WAS der Roboter gerade tun soll (Gehen vs. Balancieren) steckt im
    jeweils aktuellen Zustandsobjekt (self.state, State Pattern - siehe
    roboter_steuerung/states/), nicht mehr fest in diesem Knoten. Dieser
    Knoten ruft pro Takt nur noch self.state.execute(self) auf und
    uebernimmt den zurueckgegebenen Zustand fuer den naechsten Takt.

    NEU (12.10.2026): der Zustandsautomat deckt jetzt zusaetzlich
    StartState (noch keine gueltigen Sensordaten), BereitschaftState
    (ein Takt Uebergang) und NotAusState ab (siehe RoboterZustaende.sysml
    fuer den vollen SysML-Automaten). NotAusState wird als einziger
    davon NICHT ueber den normalen execute()-Rueckgabewert erreicht,
    sondern zentral in _control_loop() erzwungen (siehe dort) - Not-Aus
    muss jeden anderen Zustand unterbrechen koennen. AkkuSchwach/Laden
    bleiben bewusst zurueckgestellt (siehe i_robot_state.py).
    """

    def __init__(self):
        super().__init__("steuereinheit_node")

        # 1. Konfiguration laden - dieselbe YAML-Datei wie mujoco_node.py
        # (siehe dort), damit z.B. simulation_dt/control_decimation an
        # beiden Stellen zusammenpassen (siehe Taktung unten).
        # NEU (DOF-Migration): Dateiname jetzt ueber den ROS2-Parameter
        # "config_file" waehlbar (Standard seit 14.10.2026: g1_policy_29dof.yaml; 12-DOF: g1_policy.yaml; also
        # exakt das bisherige 12-DOF-Verhalten unveraendert) - z.B.
        #   ros2 launch roboter_steuerung steuereinheit.launch.py config_file:=g1_policy_29dof.yaml
        # schaltet auf die 29-DOF-Ganzkoerper-Policy um, siehe
        # g1_policy_29dof.yaml. Beide Config-Dateien bleiben nebeneinander
        # bestehen (siehe Projekt-Konvention: Alternativen zurueckstellen,
        # nicht loeschen).
        self.declare_parameter("config_file", "g1_policy_29dof.yaml")
        config_file = self.get_parameter("config_file").get_parameter_value().string_value

        bringup_share = get_package_share_directory("roboter_simulation")
        config_path = os.path.join(bringup_share, "config", config_file)
        with open(config_path, "r") as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)
        self.get_logger().info(
            f"[steuereinheit_node] aktive Config: {config_file} "
            f"(controller_type={self.config.get('controller_type', 'rl_locomotion')}, "
            f"num_actions={self.config.get('num_actions')})"
        )

        # NEU: policy_path in der YAML ist nur ein relativer String
        # ("models/policies/motion.pt"). BUGFIX: RLLocomotionController hat
        # diesen String bisher UNVERAENDERT an torch.jit.load() durchgereicht
        # - das funktionierte nur zufaellig, wenn das Arbeitsverzeichnis beim
        # Start von "ros2 launch" gerade zufaellig gepasst hat (fehlgeschlagen
        # mit "ValueError: The provided filename models/policies/motion.pt
        # does not exist", sobald es nicht passte). Fix nach demselben Muster
        # wie xml_path in mujoco_node.py: erst mit dem Package-Share-
        # Verzeichnis verbinden, mit Fallback auf den Source-Baum, falls der
        # Share-Ordner (noch) nicht gefuellt ist.
        policy_rel_path = self.config["policy_path"]
        policy_path = os.path.join(bringup_share, policy_rel_path)
        if not os.path.exists(policy_path):
            # Kein hartkodierter Rueckfall auf ~/humanoid_robot mehr (Review Kat. 6): klare Meldung.
            raise FileNotFoundError(f"Policy nicht gefunden: {policy_path} - 'colcon build' ausgefuehrt?")
        self.config["policy_path"] = policy_path

        # 2. Hardware-Bridge und RL-Controller initialisieren. num_joints
        # kommt aus num_actions in der YAML (Single Source of Truth) statt
        # hartkodiert zu sein - siehe mujoco_bridge.py.
        self.bridge = MujocoBridge(self, num_joints=self.config["num_actions"])
        # controller_type in der YAML waehlt die ILocomotionController-Implementierung. Der alte
        # 12-DOF-TorchScript-Controller liegt seit 15.10.2026 in archiv/12dof/ (nicht mehr getestet).
        controller_type = self.config.get("controller_type", "rl_locomotion_29dof")
        if controller_type == "rl_locomotion_29dof":
            from roboter_steuerung.control.rl_locomotion_controller_29dof import RLLocomotionController29Dof
            self.controller = RLLocomotionController29Dof(self.config)
        else:
            raise ValueError(f"Unbekannter controller_type in der Config: {controller_type!r}")

        # 3. Kommandowerte: cmd_target = zuletzt per /cmd_vel angefordertes Ziel, cmd_ramped =
        # gerampter Wert, cmd = an die Policy uebergebener Wert (Rampe + Kurshaltung, begrenzt).
        # GEAENDERT (14.10.2026, Review L1): Startwert immer 0. Ein cmd_init != 0 wurde
        # nach der Freigabe als Sprung an die Policy gegeben (die Rampe lief vorher schon
        # in Bereitschaft hoch); cmd_init aus der YAML wird nicht mehr verwendet.
        self.cmd_target = np.zeros(3, dtype=np.float32)
        # BUGFIX (13.10.2026): getrennter Rampen-Zustand. Vorher lief die
        # Rampe direkt auf self.cmd, und _apply_heading_hold() addierte
        # seine Korrektur (bis +-0.3) JEDEN Takt auf self.cmd[2], waehrend
        # die Rampe nur 0.02/Takt zurueckholen konnte -> unbegrenzter
        # Integrator (im synchronen Test: cmd_wz = -572 rad/s nach 60 s
        # Stehen; in Bereitschaft unsichtbar, weil dort cmd=0 an die Policy
        # geht, aber beim Wechsel nach Balancieren sofort Sturz).
        # self.cmd wird jetzt jeden Takt NEU aus cmd_ramped gebildet, die
        # Korrekturen sind damit begrenzt und nicht akkumulierend.
        self.cmd_ramped = self.cmd_target.copy()
        self.cmd = self.cmd_ramped.copy()

        # NEU (13.10.2026): Begrenzung von self.cmd auf den Trainings-
        # Wertebereich der Policy (cmd_training_range in der YAML). Ein
        # Kommando ausserhalb waere "out of distribution". Fehlt der
        # Schluessel (12-DOF-Config), bleibt das Verhalten unveraendert.
        rng = self.config.get("cmd_training_range")
        if rng:
            self.cmd_min = np.array([rng["lin_vel_x"][0], rng["lin_vel_y"][0], rng["ang_vel_z"][0]], dtype=np.float32)
            self.cmd_max = np.array([rng["lin_vel_x"][1], rng["lin_vel_y"][1], rng["ang_vel_z"][1]], dtype=np.float32)
        else:
            self.cmd_min = self.cmd_max = None
        # Kombinierte Grenze: bei hoher Vorwaertsgeschwindigkeit wird die Drehrate zusaetzlich
        # linear auf wz_at_vmax abgesenkt (gemessen: vx 1.0 + wz 1.0 faellt in ~15-20 % der Laeufe,
        # die Linie vx 0.7/wz 1.0 -> vx 1.0/wz 0.5 war in 10 Seeds je Punkt stabil).
        hs = self.config.get("cmd_wz_limit_high_speed")
        self.wz_vx_ref = float(hs["vx_ref"]) if hs else None
        self.wz_at_vmax = float(hs["wz_at_vmax"]) if hs else None
        self._cmd_clip_gemeldet = False

        # NEU (13.10.2026): cmd_vel-Watchdog. Bleibt /cmd_vel laenger als cmd_vel_timeout_s aus
        # (z.B. Nav2 faellt aus), wird cmd_target auf 0 gesetzt, statt dass der Roboter mit dem
        # letzten Kommando weiterlaeuft. 0 = aus. Parameter ueberschreibt den Config-Wert.
        self.declare_parameter("cmd_vel_timeout_s", float(self.config.get("cmd_vel_timeout_s", 0.0)))
        self.cmd_vel_timeout_s = float(self.get_parameter("cmd_vel_timeout_s").value)
        self._last_cmd_vel_t = None
        self._timeout_gemeldet = False

        # Bewegungsfreigabe (Realismus-Fix 12.10.2026): nach dem Hochfahren bleibt der Roboter in
        # Bereitschaft stehen, bis eine bewusste Freigabe kommt (siehe bereitschaft_state.py).
        # NotAus entzieht die Freigabe; nach der Entwarnung ist eine neue noetig.
        self.bewegung_freigegeben = False
        # GEAENDERT (14.10.2026, Review B1/L3): Freigabe ueber einen EIGENEN Kanal /freigabe
        # (std_msgs/Bool, true = freigeben, false = Freigabe entziehen) statt durch irgendein
        # /cmd_vel. Vorher setzte JEDE /cmd_vel-Nachricht die Freigabe - auch automatische
        # Quellen (Nav2-Glaetter beim Abbremsen, collision_monitor, balance_dienst_node), und
        # das sogar waehrend NotAus: nach der Entwarnung lief der Roboter ohne bewusste
        # Freigabe wieder an. Waehrend NotAus wird eine Freigabe immer ignoriert.
        # freigabe_per_cmd_vel=true stellt das alte Verhalten fuer Handbetrieb wieder her
        # (auch dann nie waehrend NotAus).
        self.declare_parameter("freigabe_per_cmd_vel", False)
        self.freigabe_per_cmd_vel = bool(self.get_parameter("freigabe_per_cmd_vel").value)

        # NEU (12.10.2026): Soll-Ausrichtung/Soll-Position fuer die
        # Haltungs-Korrekturen (_apply_heading_hold()/
        # _apply_position_hold()). None bis zur ersten gueltigen Messung -
        # vorher gibt es keine sinnvolle Referenz (siehe last_quat-Default
        # in mujoco_bridge.py).
        self.target_yaw = None
        self.target_position = None
        # NEU (13.10.2026): Positionshaltung im Stand (nur Zustand
        # Balancieren). Schalter in der YAML (position_hold: true). Die
        # Korrektur wird NICHT auf self.cmd addiert, sondern getrennt in
        # self.pos_hold_corr [vx, vy] abgelegt und von BalancierenState
        # an die Policy gegeben. Grund fuer das fruehere Scheitern (12.10.)
        # war sehr wahrscheinlich dieselbe Akkumulation wie bei der
        # Kurshaltung (cmd[0] = cmd[0] + correction jeden Takt).
        # Position aus /odom (Ground-Truth oder Beinodometrie, je nach odom_quelle).
        self.position_hold_aktiv = bool(self.config.get("position_hold", False))
        self.pos_hold_corr = np.zeros(2, dtype=np.float32)
        self._odom_xyz = None
        if self.position_hold_aktiv:
            self.create_subscription(Odometry, "/odom", self._odom_callback, 10)

        # NEU: /cmd_vel-Subscriber (Standard-ROS2-Konvention, geometry_msgs/
        # Twist) - ersetzt das bisherige "self.cmd wird nur einmal beim
        # Start gesetzt und nie mehr veraendert". Mapping auf den vom
        # RL-Controller erwarteten 3er-Kommandovektor [vx, vy, yaw_rate]
        # (velocity_commands, Teil der Policy-Beobachtung): linear.x -> vorwaerts/-rueckwaerts
        # (m/s), linear.y -> seitwaerts (m/s), angular.z -> Drehrate um die
        # Hochachse (rad/s). linear.z/angular.x/angular.y werden bewusst
        # ignoriert, die Policy kennt nur ebene Fortbewegung. Damit kann
        # jetzt ein beliebiger Standard-ROS2-Publisher (Joystick-Node,
        # Tastatursteuerung, "ros2 topic pub", Nav2) die Laufrichtung zur Laufzeit aendern.
        # /cmd_vel gibt seit 14.10.2026 NICHT mehr frei (siehe /freigabe).
        self.sub_cmd_vel = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_callback, 10
        )
        self.sub_freigabe = self.create_subscription(Bool, "/freigabe", self._freigabe_callback, 10)

        # (/balance_mode wurde am 14.10.2026 entfernt - Review U7: immer False, ohne Abonnenten.)

        # NEU (12.10.2026): /not_aus-Subscriber - Not-Aus (SysML:
        # RoboterZustandsautomat, Zustand "NotAus"). Recherche zur
        # ueblichen ROS2-/Industrie-Praxis (siehe Chat-Historie): ein
        # einzelnes, ROS(2)-weit standardisiertes Not-Aus-Topic/-Service
        # gibt es nicht - auf echter Hardware wird Not-Aus ueblicherweise
        # HARDWARE-seitig (Sicherheitsrelais/-schaltkreis) verdrahtet und
        # kappt die Motorversorgung UNABHAENGIG von der Software; die
        # Software bekommt den resultierenden Fehlerzustand nur noch
        # gemeldet. Da hier keine solche physische Verdrahtung existiert
        # (MuJoCo-Simulation), wird Not-Aus rein softwareseitig ausgeloest
        # - als eigenes Topic, aber mit expliziter
        # TRANSIENT_LOCAL-QoS ("gelatcht"): ein spaeter startender oder
        # neu verbindender Subscriber soll den zuletzt gesetzten Zustand
        # sofort kennen, nicht erst auf die naechste Nachricht warten
        # muessen - bei einem Sicherheitssignal darf "ich weiss es nicht"
        # niemals als "alles ok" fehlinterpretiert werden.
        not_aus_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.not_aus_aktiv = False
        self.sub_not_aus = self.create_subscription(
            Bool, "/not_aus", self._not_aus_callback, not_aus_qos
        )

        # NEU: initialer Zustand (State Pattern, siehe roboter_steuerung/
        # states/). Start jetzt im eigenen StartState (vorher implizit
        # als frueher return im Start-Gate unten in _control_loop
        # behandelt, siehe dort) - wechselt automatisch weiter nach
        # BereitschaftState, sobald die erste gueltige Sensormessung da
        # ist, und von dort nach Gehen/Balancieren je nach cmd_target.
        self.state = StartState()

        # 4. Regelungsschleife takten: simulation_dt * control_decimation,
        # z.B. 0.002s * 10 = 0.02s -> 50 Hz. Muss mit der Taktung passen,
        # mit der die Policy trainiert wurde (siehe rl_locomotion_controller.py).
        # NEU (12.10.2026): als self.control_dt gemerkt, weil _ramp_cmd()
        # pro Takt genau diese Zeitspanne braucht (siehe dort).
        self.control_dt = float(self.config["simulation_dt"]) * int(self.config["control_decimation"])
        self.timer = self.create_timer(self.control_dt, self._control_loop)
        self.get_logger().info("Steuereinheit erfolgreich gestartet (50 Hz Regelkreis aktiv).")

    def _cmd_vel_callback(self, msg: Twist):
        # Setzt nur das ZIEL (cmd_target); Rampe, Kurs-/Positionshaltung und Begrenzung
        # passieren in _control_loop(). Twist hat feste Struktur, kein Laengen-Check noetig.
        # Waehrend NotAus wird /cmd_vel komplett ignoriert (Ziel bleibt 0, siehe
        # _not_aus_callback), damit nach der Entwarnung kein altes Kommando ansteht.
        if self.not_aus_aktiv:
            return
        self.cmd_target = np.array(
            [msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32
        )
        self._last_cmd_vel_t = time.monotonic()
        self._timeout_gemeldet = False
        if self.freigabe_per_cmd_vel:
            self._setze_freigabe(True, quelle="/cmd_vel")

    def _freigabe_callback(self, msg: Bool):
        self._setze_freigabe(bool(msg.data), quelle="/freigabe")

    def _setze_freigabe(self, frei: bool, quelle: str):
        """Freigabe setzen/entziehen. Waehrend NotAus wird eine Freigabe immer abgelehnt."""
        if frei and self.not_aus_aktiv:
            self.get_logger().warn(f"Freigabe ueber {quelle} abgelehnt: NOT-AUS ist aktiv.")
            return
        if frei and not self.bewegung_freigegeben:
            # Zielrichtung der Kurshaltung bei der (Neu-)Freigabe auf die aktuelle Ausrichtung
            # setzen (wird im naechsten Takt neu erfasst). Gemessen 13.10.: der Roboter dreht
            # im Stand ~0.5 deg/s; mit dem alten Ziel drehte er beim ersten Kommando zurueck.
            self.target_yaw = None
            self.get_logger().info(f"Bewegung freigegeben ({quelle}).")
        elif not frei and self.bewegung_freigegeben:
            self.get_logger().info(f"Freigabe entzogen ({quelle}) - Roboter bleibt stehen.")
        self.bewegung_freigegeben = frei

    def _odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self._odom_xyz = np.array([p.x, p.y, p.z], dtype=np.float32)

    def _not_aus_callback(self, msg: Bool):
        # Setzt nur das Flag - die eigentliche Reaktion (Zustandswechsel
        # nach NotAusState bzw. zurueck nach BereitschaftState) passiert
        # zentral in _control_loop(), nicht hier, weil Not-Aus JEDEN
        # Zustand unterbrechen kann/muss (nicht nur Gehen/Balancieren) -
        # siehe Kommentar dort.
        self.not_aus_aktiv = bool(msg.data)
        if self.not_aus_aktiv:
            # GEAENDERT (14.10.2026, Review B1/B5): sofort im Callback (nicht erst im naechsten
            # Regeltakt) Freigabe entziehen und Ziel/Rampe auf 0 - kein altes Kommando darf
            # die Entwarnung ueberleben.
            self.bewegung_freigegeben = False
            self.cmd_target = np.zeros(3, dtype=np.float32)
            self.cmd_ramped = np.zeros(3, dtype=np.float32)

    def _ramp_cmd(self):
        """
        NEU (12.10.2026): naehert self.cmd (der tatsaechlich an die Policy
        uebergebene Wert) pro Takt nur um maximal CMD_RAMP_RATE *
        self.control_dt an self.cmd_target (das zuletzt per /cmd_vel
        angeforderte Ziel) an, statt bei jeder neuen /cmd_vel-Nachricht
        sofort zu springen. Begruendung: ein harter Sprung (z.B. 0.5 -> 0
        von einem Takt auf den naechsten) wuerde die Policy aus dem
        Geschwindigkeitsbereich reissen, in dem sie trainiert wurde - ein
        sanftes Abbremsen entspricht eher der tatsaechlichen Vorgehensweise
        von Unitree selbst (siehe Docstring in balancieren_state.py: die
        Policy bleibt auch im Stillstand aktiv, nur das Kommando wird
        weich auf 0 heruntergefahren statt hart gestoppt).
        """
        delta = self.cmd_target - self.cmd_ramped
        max_step = CMD_RAMP_RATE * self.control_dt
        delta = np.clip(delta, -max_step, max_step)
        self.cmd_ramped = self.cmd_ramped + delta
        # Korrekturen (Kurs-/Positionshaltung) kommen danach obendrauf und
        # werden jeden Takt neu berechnet, nicht aufsummiert.
        self.cmd = self.cmd_ramped.copy()

    @staticmethod
    def _yaw_from_quat(quat: np.ndarray) -> float:
        # Standard-Formel fuer die Gier-/Yaw-Komponente (Drehung um die
        # Hochachse) aus einer Quaternion in (w, x, y, z)-Reihenfolge -
        # siehe mujoco_bridge.py fuer die Begruendung dieser Reihenfolge.
        w, x, y, z = quat
        return float(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        # Bildet einen beliebigen Winkel auf den Bereich [-pi, pi] ab -
        # ohne das wuerde z.B. ein Fehler von 359 Grad statt als "-1 Grad,
        # also fast nichts" als riesiger Fehler fehlinterpretiert.
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    def _apply_heading_hold(self, quat: np.ndarray):
        """
        NEU (12.10.2026): Hintergrund - sowohl beim Gehen (cmd=[0.5,0,0],
        yaw_rate=0 angefordert) als auch beim Stehen wurde beobachtet,
        dass sich der Roboter langsam zur Seite DREHT (je nach Testlauf
        mal links, mal rechts - ein Hinweis, dass es kein fester Bias im
        Code ist, sondern dass die Policy selbst keinen geschlossenen
        Regelkreis fuer die globale Ausrichtung hat: cmd gibt nur eine
        koerperbezogene Soll-Geschwindigkeit vor, keine Welt-Richtung).
        Diese Methode ergaenzt genau das als duenne Schicht ueber der
        Policy, nicht als Ersatz: ein einfacher P-Regler auf dem
        Yaw-Fehler, der self.cmd[2] (Drehrate) korrigiert, wenn gerade
        keine explizite Drehung per /cmd_vel angefordert ist.
        """
        current_yaw = self._yaw_from_quat(quat)

        if self.target_yaw is None:
            # Erster Takt mit gueltiger IMU-Messung: die aktuelle
            # Ausrichtung wird zur Soll-Richtung, nichts zu korrigieren.
            self.target_yaw = current_yaw
            return

        if abs(float(self.cmd_target[2])) >= YAW_CMD_SCHWELLE:
            # Explizite Drehung angefordert - Heading-Hold greift hier
            # NICHT ein, sondern verfolgt laufend die aktuelle Ausrichtung,
            # damit nach Ende der Drehung genau DIESE (neue) Richtung
            # gehalten wird, statt zur alten Soll-Richtung zurueckzudrehen.
            self.target_yaw = current_yaw
            return

        yaw_error = self._wrap_angle(self.target_yaw - current_yaw)
        correction = float(np.clip(
            HEADING_HOLD_KP * yaw_error, -HEADING_HOLD_MAX_YAW_RATE, HEADING_HOLD_MAX_YAW_RATE
        ))
        self.cmd[2] = self.cmd[2] + correction

    def _apply_position_hold(self, base_position: np.ndarray, quat: np.ndarray):
        """
        Positionshaltung im Stand: P-Regler auf dem Weltpositionsfehler,
        in Koerperkoordinaten (vorwaerts/seitwaerts) umgerechnet. Ergebnis
        in self.pos_hold_corr (vx, vy), von BalancierenState an die Policy
        uebergeben. Das Ziel wird erst erfasst, wenn kein Kommando mehr
        ansteht UND die Rampe abgeklungen ist (cmd_ramped ~ 0), und bei
        jedem Verlassen von Balancieren verworfen (siehe _control_loop).
        Gemessen (13.10.2026, synchroner Kreis, 29-DOF): Abweichung ~1 cm
        ueber 60 s statt 32 cm Drift; seitlicher Stoss 0.3 m/s -> max 4 cm.
        """
        self.pos_hold_corr[:] = 0.0
        current_xy = np.asarray(base_position[:2], dtype=np.float32)

        if (
            np.any(np.abs(self.cmd_target[:2]) >= LIN_CMD_SCHWELLE)
            or np.any(np.abs(self.cmd_ramped[:2]) >= LIN_CMD_SCHWELLE)
        ):
            self.target_position = None
            return

        if self.target_position is None:
            self.target_position = current_xy.copy()
            return

        dx, dy = self.target_position - current_xy
        yaw = self._yaw_from_quat(quat)
        forward_error = np.cos(yaw) * dx + np.sin(yaw) * dy
        lateral_error = -np.sin(yaw) * dx + np.cos(yaw) * dy
        self.pos_hold_corr[0] = float(np.clip(
            POSITION_HOLD_KP * forward_error, -POSITION_HOLD_MAX_SPEED, POSITION_HOLD_MAX_SPEED))
        self.pos_hold_corr[1] = float(np.clip(
            POSITION_HOLD_KP * lateral_error, -POSITION_HOLD_MAX_SPEED, POSITION_HOLD_MAX_SPEED))

    def _check_cmd_timeout(self, now: float) -> bool:
        """True, wenn der Watchdog ausgeloest hat (cmd_target wurde auf 0 gesetzt)."""
        if self.cmd_vel_timeout_s <= 0.0 or self._last_cmd_vel_t is None:
            return False
        if now - self._last_cmd_vel_t <= self.cmd_vel_timeout_s:
            return False
        if np.any(self.cmd_target != 0.0):
            self.cmd_target = np.zeros(3, dtype=np.float32)
            if not self._timeout_gemeldet:
                self.get_logger().warn(
                    f"/cmd_vel seit > {self.cmd_vel_timeout_s:.1f} s nicht empfangen - Kommando auf 0 gesetzt.")
                self._timeout_gemeldet = True
            return True
        return False

    def _clip_cmd(self):
        """Begrenzt self.cmd auf cmd_training_range (falls konfiguriert) und
        warnt einmalig, wenn das angeforderte /cmd_vel ausserhalb liegt."""
        if self.cmd_min is None:
            return
        if not self._cmd_clip_gemeldet and np.any(
            (self.cmd_target < self.cmd_min) | (self.cmd_target > self.cmd_max)
        ):
            self.get_logger().warn(
                f"cmd_vel {self.cmd_target.tolist()} liegt ausserhalb des Trainingsbereichs "
                f"[{self.cmd_min.tolist()} .. {self.cmd_max.tolist()}] - wird begrenzt."
            )
            self._cmd_clip_gemeldet = True
        self.cmd = np.clip(self.cmd, self.cmd_min, self.cmd_max)
        if self.wz_vx_ref is not None:
            vmax = float(max(abs(self.cmd_min[0]), abs(self.cmd_max[0])))
            wz_hi = float(self.cmd_max[2])
            v = abs(float(self.cmd[0]))
            if v > self.wz_vx_ref and vmax > self.wz_vx_ref:
                a = min(1.0, (v - self.wz_vx_ref) / (vmax - self.wz_vx_ref))
                cap = wz_hi + a * (self.wz_at_vmax - wz_hi)
                self.cmd[2] = float(np.clip(self.cmd[2], -cap, cap))

    def _control_loop(self):
        # Start-Gate bleibt hier im Node und ist bewusst zustands-
        # unabhaengig: direkt nach dem Start dieses Knotens sind die
        # Bridge-Werte noch auf ihrem Initialwert (alles 0.0), weil noch
        # keine /joint_states- bzw. /imu-Nachricht von mujoco_node.py
        # angekommen ist. Eine echte Quaternion ist nie komplett 0 (siehe
        # last_quat-Default in mujoco_bridge.py) - der Vergleich erkennt
        # diesen Fall also zuverlaessig und ueberspringt den Takt, statt
        # die Policy mit bedeutungslosen Nullwerten laufen zu lassen. Das
        # betrifft beide Zustaende gleich, deshalb kein Teil der
        # State-Pattern-Logik selbst.
        quat, omega = self.bridge.read_imu()
        joint_pos, joint_vel = self.bridge.read_joint_states()
        if np.all(joint_pos == 0.0) and np.all(quat == 0.0):
            # Review L6: StartState.execute laeuft dadurch nie mit ungueltigen Daten; ein NotAus in
            # dieser Phase wird im Callback gemerkt und mit den ersten Sensordaten wirksam (vorher kann
            # die Policy ohnehin nicht laufen; die Physik pausiert bis zum ersten Kommando).
            return

        self._check_cmd_timeout(time.monotonic())
        # GEAENDERT (14.10.2026, Review L1/B5): ohne Freigabe (Bereitschaft) und bei NotAus
        # bleibt die Rampe auf 0. Vorher lief sie dort mit dem Nutzerkommando hoch: nach der
        # Freigabe sprang die Policy von 0 auf den fast vollen Wert, und Bereitschaft gab
        # eine alte Drehrate weiter (cmd[2] enthielt cmd_ramped[2]).
        if not self.bewegung_freigegeben or self.not_aus_aktiv:
            self.cmd_ramped = np.zeros(3, dtype=np.float32)
        self._ramp_cmd()
        if not self.bewegung_freigegeben or self.not_aus_aktiv:
            self.cmd_ramped = np.zeros(3, dtype=np.float32)
            self.cmd = np.zeros(3, dtype=np.float32)
        self._apply_heading_hold(quat)
        self._clip_cmd()
        # Positionshaltung nur im Zustand Balancieren (Schalter position_hold in der YAML). Der
        # erste Versuch am 12.10. fiel um; Ursache war die Akkumulation der Korrektur auf
        # self.cmd (wie bei der Kurshaltung). Seit 13.10. getrennt in pos_hold_corr: stabil.
        if self.position_hold_aktiv:
            from roboter_steuerung.states.balancieren_state import BalancierenState
            if isinstance(self.state, BalancierenState) and self._odom_xyz is not None:
                self._apply_position_hold(self._odom_xyz, quat)
            else:
                self.target_position = None
                self.pos_hold_corr[:] = 0.0

        # NEU (12.10.2026): Not-Aus unterbricht JEDEN Zustand, nicht nur
        # Gehen/Balancieren - deshalb zentral hier, VOR dem normalen
        # State-Pattern-Aufruf, statt als Teil des regulaeren
        # execute()-Rueckgabewerts einzelner Zustaende. Erzwingt den
        # Wechsel nach NotAusState, solange self.not_aus_aktiv True ist,
        # und danach zurueck nach BereitschaftState, sobald es wieder
        # False wird - inkl. Ruecksetzen von bewegung_freigegeben, damit
        # nach einer Entwarnung eine bewusste neue Freigabe noetig ist
        # (siehe bewegung_freigegeben oben, schliesst den vorher
        # dokumentierten offenen Punkt).
        if self.not_aus_aktiv:
            if not isinstance(self.state, NotAusState):
                self.get_logger().warn("NOT-AUS ausgeloest - Bewegungsregelung angehalten.")
                self.state = NotAusState()
                # NEU (12.10.2026): schliesst den vorher dokumentierten
                # offenen Punkt - ohne das wuerde die Bewegung nach
                # Entwarnung automatisch wieder anlaufen (letzter
                # cmd_target-Wert war ja schon vorher gueltig). Jetzt
                # muss nach Not-Aus erst wieder eine echte /cmd_vel-
                # Nachricht ankommen (siehe BereitschaftState).
                self.bewegung_freigegeben = False
        elif isinstance(self.state, NotAusState):
            self.get_logger().info("NOT-AUS aufgehoben - Zustandsautomat wird ab Bereitschaft fortgesetzt.")
            self.state = BereitschaftState()
        elif not self.bewegung_freigegeben and not isinstance(self.state, (BereitschaftState, StartState)):
            # Freigabe entzogen (/freigabe false): zurueck in Bereitschaft (Policy aktiv, steht)
            self.get_logger().info("Freigabe entzogen - Zustandswechsel nach Bereitschaft.")
            self.state = BereitschaftState()

        # State Pattern (siehe roboter_steuerung/states/): der aktuelle
        # Zustand liest sich seine Sensordaten selbst erneut (kleine,
        # bewusst in Kauf genommene Redundanz zum Start-Gate oben - beide
        # Lesezugriffe sind nur guenstige Rueckgaben zuletzt gespeicherter
        # Werte, siehe mujoco_bridge.py), fuehrt seine Logik aus und gibt
        # den fuer den naechsten Takt geltenden Zustand zurueck.
        self.state = self.state.execute(self)


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
