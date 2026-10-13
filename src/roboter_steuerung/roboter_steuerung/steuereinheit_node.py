import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from r1_bringup.bridges.mujoco_bridge import MujocoBridge
from roboter_steuerung.control.rl_locomotion_controller import RLLocomotionController
from roboter_steuerung.control.rl_locomotion_controller_29dof import RLLocomotionController29Dof
from roboter_steuerung.states.start_state import StartState
from roboter_steuerung.states.bereitschaft_state import BereitschaftState
from roboter_steuerung.states.not_aus_state import NotAusState


# NEU (12.10.2026): maximale Aenderungsrate von self.cmd pro Sekunde (siehe
# _ramp_cmd() unten), fuer alle drei Kommando-Achsen (vx/vy/yaw_rate)
# gemeinsam verwendet. Bewusst ein einzelner, noch nicht empirisch
# feinabgestimmter Startwert (aehnlich CMD_NULL_SCHWELLE in den
# States) - 1.0 heisst z.B., ein Sprung von cmd_init [0.5, 0, 0] auf
# [0, 0, 0] dauert ca. 0.5s statt einem einzigen Regelungstakt.
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
YAW_CMD_SCHWELLE = 0.05
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
POSITION_HOLD_MAX_SPEED = 0.2


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
        # "config_file" waehlbar (Default bleibt "g1_policy.yaml", also
        # exakt das bisherige 12-DOF-Verhalten unveraendert) - z.B.
        #   ros2 launch roboter_steuerung steuereinheit.launch.py config_file:=g1_policy_29dof.yaml
        # schaltet auf die 29-DOF-Ganzkoerper-Policy um, siehe
        # g1_policy_29dof.yaml. Beide Config-Dateien bleiben nebeneinander
        # bestehen (siehe Projekt-Konvention: Alternativen zurueckstellen,
        # nicht loeschen).
        self.declare_parameter("config_file", "g1_policy.yaml")
        config_file = self.get_parameter("config_file").get_parameter_value().string_value

        bringup_share = get_package_share_directory("r1_bringup")
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
            policy_path = os.path.expanduser(f"~/humanoid_robot/src/r1_bringup/{policy_rel_path}")
        self.config["policy_path"] = policy_path

        # 2. Hardware-Bridge und RL-Controller initialisieren. num_joints
        # kommt aus num_actions in der YAML (Single Source of Truth) statt
        # hartkodiert zu sein - siehe mujoco_bridge.py.
        self.bridge = MujocoBridge(self, num_joints=self.config["num_actions"])
        # NEU (DOF-Migration): controller_type in der YAML waehlt die
        # ILocomotionController-Implementierung (Single Source of Truth,
        # wie num_actions) - fehlt der Schluessel (alte g1_policy.yaml),
        # bleibt es beim bisherigen 12-DOF-TorchScript-Controller.
        controller_type = self.config.get("controller_type", "rl_locomotion")
        if controller_type == "rl_locomotion_29dof":
            self.controller = RLLocomotionController29Dof(self.config)
        elif controller_type == "rl_locomotion":
            self.controller = RLLocomotionController(self.config)
        else:
            raise ValueError(f"Unbekannter controller_type in der Config: {controller_type!r}")

        # 3. Start-/Default-Geschwindigkeitskommando aus der YAML. Wird
        # weiter unten per /cmd_vel-Subscriber zur Laufzeit ueberschrieben,
        # sobald eine erste Nachricht ankommt - bis dahin gilt dieser
        # Default, damit der Roboter nicht ohne jedes Kommando (z.B. mit
        # lauter Nullen) startet. NEU (12.10.2026): cmd_target ist das
        # angeforderte Ziel, cmd der aktuell gerampte+korrigierte Wert -
        # beide starten gleich, es gibt also am Anfang nichts auszurampen.
        self.cmd_target = np.array(self.config.get("cmd_init", [0.5, 0.0, 0.0]), dtype=np.float32)
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

        # NEU (12.10.2026, Realismus-Fix): ohne dieses Flag wuerde
        # BereitschaftState sofort anhand von cmd_target entscheiden -
        # und cmd_target startet mit cmd_init (haeufig != 0), der
        # Roboter wuerde also direkt nach dem Hochfahren von sich aus
        # loslaufen, OHNE dass je ein Bediener ein Kommando gesendet
        # hat. True wird es erst in _cmd_vel_callback(), sobald die
        # ERSTE echte /cmd_vel-Nachricht ankommt (siehe dort) - das ist
        # die eigentliche "Freigabe", nach der BereitschaftState
        # wartet (siehe bereitschaft_state.py). cmd_init bleibt dadurch
        # nur noch ein Platzhalter-Anfangswert, der erst nach der
        # Freigabe tatsaechlich wirksam werden kann.
        self.bewegung_freigegeben = False

        # NEU (12.10.2026): Soll-Ausrichtung/Soll-Position fuer die
        # Haltungs-Korrekturen (_apply_heading_hold()/
        # _apply_position_hold()). None bis zur ersten gueltigen Messung -
        # vorher gibt es keine sinnvolle Referenz (siehe last_quat-Default
        # in mujoco_bridge.py).
        self.target_yaw = None
        self.target_position = None

        # NEU: /cmd_vel-Subscriber (Standard-ROS2-Konvention, geometry_msgs/
        # Twist) - ersetzt das bisherige "self.cmd wird nur einmal beim
        # Start gesetzt und nie mehr veraendert". Mapping auf den vom
        # RL-Controller erwarteten 3er-Kommandovektor [vx, vy, yaw_rate]
        # (siehe cmd_init oben, 3 der 47 Beobachtungswerte in
        # rl_locomotion_controller.py): linear.x -> vorwaerts/-rueckwaerts
        # (m/s), linear.y -> seitwaerts (m/s), angular.z -> Drehrate um die
        # Hochachse (rad/s). linear.z/angular.x/angular.y werden bewusst
        # ignoriert, die Policy kennt nur ebene Fortbewegung. Damit kann
        # jetzt ein beliebiger Standard-ROS2-Publisher (Joystick-Node,
        # Tastatursteuerung, "ros2 topic pub", spaeter Nav2) die
        # Laufrichtung zur Laufzeit aendern, ohne den Knoten neu zu starten.
        self.sub_cmd_vel = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_callback, 10
        )

        # NEU (12.10.2026): /balance_mode-Publisher - teilt mujoco_node.py
        # mit, ob gerade der Stehen-Zustand (BalancierenState) aktiv ist.
        # Aktuell OHNE Wirkung (mujoco_node.py hat (noch) keinen
        # entsprechenden Subscriber/kein eigenes Stand-Gain-Set) - reine
        # Vorbereitung fuer eine spaetere Erweiterung, bewusst schon verdrahtet.
        self.pub_balance_mode = self.create_publisher(Bool, "/balance_mode", 10)

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
        # - nach demselben Muster wie /balance_mode, aber mit expliziter
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
        # NEU (12.10.2026): setzt nur noch das ZIEL (cmd_target), nicht
        # mehr den tatsaechlich an die Policy uebergebenen Wert direkt -
        # das sanfte Nachfuehren + die Haltungs-Korrekturen passieren jetzt
        # in _ramp_cmd()/_apply_heading_hold()/_apply_position_hold(). Kein
        # Laengen-Check noetig wie bei /joint_commands in mujoco_node.py,
        # da Twist immer dieselbe feste Struktur hat.
        self.cmd_target = np.array(
            [msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32
        )

        # NEU (12.10.2026, Realismus-Fix): das Ankommen IRGENDEINER
        # /cmd_vel-Nachricht gilt als bewusste Freigabe durch einen
        # Bediener/eine uebergeordnete Instanz (Joystick, Tastatur,
        # Nav2, "ros2 topic pub") - auch ein explizites "stehen
        # bleiben" (0,0,0) zaehlt als Freigabe, nicht nur ein
        # Bewegungswunsch. Bleibt einmal True (kein automatisches
        # Zuruecksetzen hier) - wird NUR beim Ausloesen von Not-Aus
        # wieder auf False gesetzt (siehe _control_loop()), damit nach
        # einer Entwarnung wieder eine bewusste Freigabe noetig ist.
        self.bewegung_freigegeben = True

    def _not_aus_callback(self, msg: Bool):
        # Setzt nur das Flag - die eigentliche Reaktion (Zustandswechsel
        # nach NotAusState bzw. zurueck nach BereitschaftState) passiert
        # zentral in _control_loop(), nicht hier, weil Not-Aus JEDEN
        # Zustand unterbrechen kann/muss (nicht nur Gehen/Balancieren) -
        # siehe Kommentar dort.
        self.not_aus_aktiv = bool(msg.data)

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
        NEU (12.10.2026): Hintergrund - zusaetzlich zum Drehungs-Drift
        (siehe _apply_heading_hold()) wurde beim Stehen auch reines
        seitliches Wegrutschen OHNE Drehung beobachtet (der Roboter bleibt
        gleich ausgerichtet, "marschiert" aber auf der Stelle leicht nach
        links weg - vermutlich weil der Gang-Phasen-Zaehler auch bei
        cmd=0 weiterlaeuft, siehe balancieren_state.py). Analog zu
        _apply_heading_hold(), aber auf der Position statt der
        Ausrichtung: Fehler zwischen Soll- und Ist-Position (Weltkoordinaten,
        aus base_position) wird mit dem aktuellen Yaw in Koerper-Koordinaten
        (vorwaerts/seitwaerts) umgerechnet und als P-Korrektur auf
        self.cmd[0]/self.cmd[1] angewendet.

        OFFENER PUNKT/RISIKO: anders als beim bisherigen "Variante 4"-
        Ansatz (vx/vy hart auf 0) wird hier jetzt aktiv ein kleines,
        von Null verschiedenes vx/vy an die Policy gegeben. Variante 1/4
        wurde urspruenglich gerade WEIL cmd=[0,0,0] (hart) als einzige
        Variante NIE umgefallen ist. Es ist nicht auszuschliessen, dass
        eine aktive, wenn auch kleine Geschwindigkeitskorrektur die
        Policy wieder naeher an ihre Stabilitaetsgrenze bringt - deshalb
        bewusst kleine Verstaerkung/Begrenzung (POSITION_HOLD_KP/
        POSITION_HOLD_MAX_SPEED) und explizit als zu beobachtender
        Testpunkt markiert, nicht als sichere Verbesserung vorausgesetzt.
        """
        current_xy = base_position[:2]

        if self.target_position is None:
            # Erster Takt mit gueltiger Positions-Messung: die aktuelle
            # Position wird zur Soll-Position, nichts zu korrigieren.
            self.target_position = current_xy.copy()
            return

        if abs(float(self.cmd_target[0])) >= LIN_CMD_SCHWELLE or abs(float(self.cmd_target[1])) >= LIN_CMD_SCHWELLE:
            # Explizite Translation angefordert - Positions-Haltung greift
            # hier NICHT ein, sondern verfolgt laufend die aktuelle
            # Position, damit nach Ende der Bewegung genau DIESE (neue)
            # Position gehalten wird, statt zur alten zurueckzulaufen.
            self.target_position = current_xy.copy()
            return

        # Fehler in Weltkoordinaten (Soll minus Ist) ...
        dx, dy = self.target_position - current_xy
        # ... in Koerper-Koordinaten umrechnen (vorwaerts/seitwaerts
        # relativ zur aktuellen Ausrichtung) - Standard-Rotation um den
        # aktuellen Yaw-Winkel.
        yaw = self._yaw_from_quat(quat)
        forward_error = np.cos(yaw) * dx + np.sin(yaw) * dy
        lateral_error = -np.sin(yaw) * dx + np.cos(yaw) * dy

        correction_vx = float(np.clip(
            POSITION_HOLD_KP * forward_error, -POSITION_HOLD_MAX_SPEED, POSITION_HOLD_MAX_SPEED
        ))
        correction_vy = float(np.clip(
            POSITION_HOLD_KP * lateral_error, -POSITION_HOLD_MAX_SPEED, POSITION_HOLD_MAX_SPEED
        ))
        self.cmd[0] = self.cmd[0] + correction_vx
        self.cmd[1] = self.cmd[1] + correction_vy

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
            return

        self._ramp_cmd()
        self._apply_heading_hold(quat)
        # ZURUECKGEBAUT (12.10.2026): _apply_position_hold() wurde testweise
        # aktiviert, hat beim Stehen-Test aber zum Umfallen gefuehrt - die
        # aktive vx/vy-Korrektur hat die bisher stabile Variante (cmd hart
        # auf [0,0,0]) offenbar wieder naeher an ihre Kippgrenze gebracht
        # (siehe Risiko-Hinweis im Docstring von _apply_position_hold()
        # unten, der sich damit bestaetigt hat). Methode bleibt im Code
        # erhalten (fuer eine spaetere, vorsichtiger abgestimmte Variante
        # oder eine andere Policy), wird aber aktuell NICHT aufgerufen.
        # self._apply_position_hold(self.bridge.read_base_position(), quat)

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
