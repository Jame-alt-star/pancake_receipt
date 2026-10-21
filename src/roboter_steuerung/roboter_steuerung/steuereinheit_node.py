"""Steuereinheit (Layer 3): Regelschleife im Takt der Policy (50 Hz) mit Zustandsautomat.

Je Takt: Sensordaten ueber die Hardware-Bridge lesen, Fahrbefehl aufbereiten (kommando.py: Watchdog, Rampe,
Begrenzung; haltung.py: Kurs- und Positionshaltung), NotAus/Freigabe auswerten und den aktuellen Zustand
ausfuehren lassen (State Pattern, states/). Der Zustand ruft die Policy (control/) und schickt die
Ziel-Gelenkwinkel ueber die Bridge an Layer 4. Der Knoten kennt weder MuJoCo noch die Netzarchitektur.

Kommandowerte:
  cmd_target  zuletzt per /cmd_vel angeforderter Wunsch [vx, vy, wz] (Zustandswechsel pruefen dagegen)
  cmd_ramped  sanft an cmd_target angenaehert (Rampe)
  cmd         an die Policy: cmd_ramped + Kurshaltung, begrenzt auf den Trainingsbereich
  pos_hold_corr  Positionshaltung (vx, vy), nur im Zustand Balancieren

Zustaende (SysML RoboterZustandsautomat): Start, Bereitschaft, Gehen, Balancieren, NotAus, dazu Stoerung (Ergaenzung
21.10.2026). NotAus und Stoerung werden zentral erzwungen (unterbrechen jeden Zustand), nicht ueber den
execute()-Rueckgabewert. AkkuSchwach/Laden fehlen bewusst (keine echten Batteriedaten).

Sicherheitspruefungen je Takt: Sensordaten aktuell (sensor_timeout_s), Roboter aufrecht (sturz_neigung_grad), Policy-
Ausgabe endlich; Zielwinkel werden auf die Gelenkgrenzen des Robotermodells begrenzt.

Topics: /cmd_vel (Twist, Wunsch), /cmd_vel/balance (Twist, Vorrang vor /cmd_vel, z. B. Balance-Dienst), /freigabe
(Bool, eigener Kanal), /not_aus (Bool, latched), /odom (Positionshaltung); raus: /steuerung/zustand (String, latched).
Dienst /steuerung/quittieren (Trigger): Stoerung bewusst verlassen.
Parameter: config/steuerung.yaml; Policy: config/policy_29dof.yaml.
"""
import os
import time

import mujoco
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from roboter_beschreibung import lade as lade_roboter
from roboter_beschreibung.geometrie import neigung_grad

from roboter_steuerung.hardware.topic_bridge import TopicBridge
from roboter_steuerung.haltung import Kurshaltung, Positionshaltung
from roboter_steuerung.kommando import Kommandobegrenzung, Watchdog, rampe
from roboter_steuerung.states.balancieren_state import BalancierenState
from roboter_steuerung.states.bereitschaft_state import BereitschaftState
from roboter_steuerung.states.not_aus_state import NotAusState
from roboter_steuerung.states.start_state import StartState
from roboter_steuerung.states.stoerung_state import StoerungState

# Vorrang-Eingang /cmd_vel/balance gilt so lange nach seiner letzten Nachricht (der Balance-Dienst sendet mit 10 Hz).
BALANCE_VORRANG_S = 0.5


def lade_policy(share: str, datei: str) -> dict:
    """Policy-Config aus config/ von roboter_steuerung; policy_path wird absolut (relativ zum Share-Verzeichnis,
    ein relativer Pfad brach frueher, sobald das Arbeitsverzeichnis beim Start nicht passte)."""
    with open(os.path.join(share, "config", datei)) as f:
        config = yaml.safe_load(f)
    config["policy_path"] = os.path.join(share, config["policy_path"])
    if not os.path.exists(config["policy_path"]):
        raise FileNotFoundError(f"Policy nicht gefunden: {config['policy_path']} - 'colcon build' ausgefuehrt?")
    return config


def gelenkgrenzen(modell_xml: str):
    """Untere/obere Grenze je Aktuator (Gelenkreihenfolge des Modells = Reihenfolge der Zielwinkel)."""
    m = mujoco.MjModel.from_xml_path(modell_xml)
    unten, oben = np.full(m.nu, -np.inf), np.full(m.nu, np.inf)
    for i in range(m.nu):
        j = m.actuator_trnid[i, 0]
        if m.jnt_limited[j]:
            unten[i], oben[i] = m.jnt_range[j]
    return unten.astype(np.float32), oben.astype(np.float32)


def erzeuge_controller(config: dict):
    """ILocomotionController nach controller_type (der 12-DOF-Controller ist entfernt, Tag v1.9-review-abgeschlossen)."""
    typ = config.get("controller_type", "rl_locomotion_29dof")
    if typ == "rl_locomotion_29dof":
        from roboter_steuerung.control.rl_locomotion_controller_29dof import RLLocomotionController29Dof
        return RLLocomotionController29Dof(config)
    raise ValueError(f"Unbekannter controller_type in der Config: {typ!r}")


class SteuereinheitNode(Node):

    def __init__(self):
        super().__init__("steuereinheit_node")
        self.declare_parameter("policy_config", "policy_29dof.yaml")
        self.declare_parameter("cmd_vel_timeout_s", 0.0)
        self.declare_parameter("freigabe_per_cmd_vel", False)
        self.declare_parameter("position_hold", True)
        self.declare_parameter("sensor_timeout_s", 1.0)     # aelter -> Stoerung (Sensorausfall)
        self.declare_parameter("sturz_neigung_grad", 45.0)  # staerker geneigt -> Stoerung (Sturz)
        # Takt: "sensor" = alle regeltakt_s / sensor_periode_s Gelenknachrichten ein Regelschritt. Damit laeuft die
        # Policy im Takt der Simulations- bzw. Messzeit statt der Wanduhr: laeuft die Simulation unter Last
        # langsamer als Echtzeit, bekam die Policy vorher mehr Schritte je Simulationssekunde als im Training.
        # "timer" = alter Betrieb mit ROS-Timer (Wanduhr).
        self.declare_parameter("takt_quelle", "sensor")
        self.declare_parameter("sensor_periode_s", 0.01)    # Abstand der Gelenknachrichten (Simulation: 100 Hz)
        self.sensor_timeout_s = float(self.get_parameter("sensor_timeout_s").value)
        self.sturz_neigung_grad = float(self.get_parameter("sturz_neigung_grad").value)

        # Policy (Layer 3): Netz, Verstaerkungen, Ruhepose, Regeltakt, Kommandobereich. Die Bridge schickt
        # kp/kd/Ruhepose einmal latched an Layer 4 (/antrieb/parameter), wie an einen Motortreiber.
        policy_config = self.get_parameter("policy_config").value
        self.config = lade_policy(get_package_share_directory("roboter_steuerung"), policy_config)
        self.get_logger().info(f"[steuereinheit_node] Policy-Config: {policy_config} "
                               f"(controller_type={self.config.get('controller_type')}, "
                               f"num_actions={self.config.get('num_actions')})")
        self.bridge = TopicBridge(self, num_joints=self.config["num_actions"], kp=self.config["kps"],
                                  kd=self.config["kds"], startpose=self.config["default_angles"])
        self.controller = erzeuge_controller(self.config)
        # Gelenkgrenzen aus dem Robotermodell: in der Simulation begrenzt MuJoCo selbst, auf Hardware ginge ein
        # Ausreisser der Policy sonst direkt an die Motoren.
        self.grenze_unten, self.grenze_oben = gelenkgrenzen(lade_roboter()["modell_xml"])
        self.control_dt = float(self.config["regeltakt_s"])

        # Fahrbefehl (siehe Modul-Docstring). Startwert immer 0 (Review L1). cmd_wunsch kommt von /cmd_vel; cmd_target
        # ist der je Takt gueltige Wunsch (Vorrang /cmd_vel/balance).
        self.cmd_wunsch = np.zeros(3, dtype=np.float32)
        self._balance_cmd = np.zeros(3, dtype=np.float32)
        self._balance_bis = 0.0
        self.cmd_target = np.zeros(3, dtype=np.float32)
        self.cmd_ramped = np.zeros(3, dtype=np.float32)
        self.cmd = np.zeros(3, dtype=np.float32)
        self.begrenzung = Kommandobegrenzung.aus_policy(self.config)
        self._cmd_clip_gemeldet = False
        self.watchdog = Watchdog(float(self.get_parameter("cmd_vel_timeout_s").value))
        self._timeout_gemeldet = False
        self.kurshaltung = Kurshaltung()
        self.positionshaltung = Positionshaltung()
        self.position_hold_aktiv = bool(self.get_parameter("position_hold").value)
        self.pos_hold_corr = np.zeros(2, dtype=np.float32)
        self._odom_xyz = None
        if self.position_hold_aktiv:
            self.create_subscription(Odometry, "/odom", self._odom_callback, 10)

        # Freigabe ueber den eigenen Kanal /freigabe (Review B1/L3): vorher gab JEDES /cmd_vel frei, auch
        # automatische Quellen und sogar waehrend NotAus. freigabe_per_cmd_vel=true stellt den alten
        # Handbetrieb wieder her (auch dann nie waehrend NotAus).
        self.bewegung_freigegeben = False
        self.freigabe_per_cmd_vel = bool(self.get_parameter("freigabe_per_cmd_vel").value)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_callback, 10)
        # Vorrang-Eingang: frueher sendeten Balance-Dienst und Nav2 beide auf /cmd_vel und ueberschrieben sich.
        self.create_subscription(Twist, "/cmd_vel/balance", self._balance_callback, 10)
        self.create_subscription(Bool, "/freigabe", self._freigabe_callback, 10)

        # /not_aus latched (TRANSIENT_LOCAL): ein spaet verbindender Knoten kennt sofort den letzten Zustand -
        # bei einem Sicherheitssignal darf "unbekannt" nie als "alles ok" gelten. Auf echter Hardware kappt der
        # Not-Aus die Motorversorgung per Sicherheitsrelais; hier ist er rein softwareseitig.
        not_aus_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)
        self.not_aus_aktiv = False
        self.create_subscription(Bool, "/not_aus", self._not_aus_callback, not_aus_qos)

        self.pub_zustand = self.create_publisher(String, "/steuerung/zustand", not_aus_qos)
        self._gemeldeter_zustand = None
        self.create_service(Trigger, "/steuerung/quittieren", self._quittieren_callback)

        self.state = StartState()
        self.takt_quelle = str(self.get_parameter("takt_quelle").value)
        if self.takt_quelle == "sensor":
            periode = float(self.get_parameter("sensor_periode_s").value)
            self._teiler = max(1, int(round(self.control_dt / periode)))
            self._zaehler = 0
            self.bridge.bei_gelenken = self._sensor_takt
            # Ueberwachung unabhaengig vom Sensortakt: bleiben die Nachrichten ganz aus, laeuft sonst kein Regelschritt
            self.create_timer(0.1, self._ueberwache_sensoren)
            takt = f"jede {self._teiler}. Gelenknachricht"
        elif self.takt_quelle == "timer":
            self.timer = self.create_timer(self.control_dt, self._control_loop)
            takt = "ROS-Timer"
        else:
            raise ValueError(f"takt_quelle {self.takt_quelle!r} - erlaubt: sensor, timer")
        self.get_logger().info(f"Steuereinheit gestartet ({1.0 / self.control_dt:.0f} Hz Regelkreis, Takt: {takt}).")

    # ---------------------------------------------------------------- Eingaenge
    def _cmd_vel_callback(self, msg: Twist):
        """Setzt nur den Wunsch; waehrend NotAus ignoriert (kein altes Kommando ueberlebt die Entwarnung).
        linear.z/angular.x/y entfallen: die Policy kennt nur ebene Fortbewegung."""
        if self.not_aus_aktiv:
            return
        self.cmd_wunsch = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32)
        self.cmd_target = self.cmd_wunsch.copy()
        self.watchdog.empfangen(time.monotonic())
        self._timeout_gemeldet = False
        if self.freigabe_per_cmd_vel:
            self._setze_freigabe(True, quelle="/cmd_vel")

    def _balance_callback(self, msg: Twist):
        if self.not_aus_aktiv:
            return
        self._balance_cmd = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32)
        self._balance_bis = time.monotonic() + BALANCE_VORRANG_S

    def _freigabe_callback(self, msg: Bool):
        self._setze_freigabe(bool(msg.data), quelle="/freigabe")

    def _setze_freigabe(self, frei: bool, quelle: str):
        """Freigabe setzen/entziehen. Waehrend NotAus wird eine Freigabe immer abgelehnt."""
        if frei and self.not_aus_aktiv:
            self.get_logger().warn(f"Freigabe ueber {quelle} abgelehnt: NOT-AUS ist aktiv.")
            return
        if frei and not self.bewegung_freigegeben:
            self.kurshaltung.zuruecksetzen()
            self.get_logger().info(f"Bewegung freigegeben ({quelle}).")
        elif not frei and self.bewegung_freigegeben:
            self.get_logger().info(f"Freigabe entzogen ({quelle}) - Roboter bleibt stehen.")
        self.bewegung_freigegeben = frei

    def _odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self._odom_xyz = np.array([p.x, p.y, p.z], dtype=np.float32)

    def _not_aus_callback(self, msg: Bool):
        """Sofort (nicht erst im naechsten Takt, Review B1/B5): Freigabe entziehen, Ziel und Rampe auf 0.
        Der Zustandswechsel selbst passiert zentral in _control_loop."""
        self.not_aus_aktiv = bool(msg.data)
        if self.not_aus_aktiv:
            self.bewegung_freigegeben = False
            self.cmd_wunsch = np.zeros(3, dtype=np.float32)
            self.cmd_target = np.zeros(3, dtype=np.float32)
            self.cmd_ramped = np.zeros(3, dtype=np.float32)
            self._balance_bis = 0.0

    def _quittieren_callback(self, req, resp):
        """Stoerung bewusst verlassen - nur wenn die Ursache behoben ist."""
        if not isinstance(self.state, StoerungState):
            resp.success, resp.message = False, "keine Stoerung aktiv"
        elif self.not_aus_aktiv:
            resp.success, resp.message = False, "NOT-AUS aktiv"
        elif self.bridge.sensor_alter_s() > self.sensor_timeout_s:
            resp.success, resp.message = False, "Sensordaten fehlen weiterhin"
        elif neigung_grad(self.bridge.read_imu()[0]) > self.sturz_neigung_grad:
            resp.success, resp.message = False, "Roboter nicht aufrecht"
        else:
            self.get_logger().info(f"Stoerung quittiert ({self.state.grund}) - weiter ab Bereitschaft.")
            self.bewegung_freigegeben = False
            self.state = BereitschaftState()
            resp.success, resp.message = True, "quittiert"
        return resp

    def _stoerung(self, grund: str):
        """In den Zustand Stoerung wechseln (einmal melden), Freigabe und Befehle zuruecksetzen."""
        if isinstance(self.state, StoerungState):
            return
        self.get_logger().error(f"STOERUNG: {grund} - keine Zielwinkel mehr; Layer 4 daempft. "
                                f"Weiter nur ueber /steuerung/quittieren.")
        self.state = StoerungState(grund)
        self.bewegung_freigegeben = False
        self.cmd_wunsch = np.zeros(3, dtype=np.float32)
        self.cmd_target = np.zeros(3, dtype=np.float32)
        self.cmd_ramped = np.zeros(3, dtype=np.float32)

    # ---------------------------------------------------------------- Fahrbefehl
    def _pruefe_watchdog(self, jetzt: float) -> bool:
        """True, wenn der Watchdog ausgeloest hat (der Wunsch von /cmd_vel wurde auf 0 gesetzt)."""
        if not self.watchdog.abgelaufen(jetzt) or not np.any(self.cmd_wunsch != 0.0):
            return False
        self.cmd_wunsch = np.zeros(3, dtype=np.float32)
        self.cmd_target = np.zeros(3, dtype=np.float32)
        if not self._timeout_gemeldet:
            self.get_logger().warn(f"/cmd_vel seit > {self.watchdog.timeout_s:.1f} s nicht empfangen - "
                                   f"Kommando auf 0 gesetzt.")
            self._timeout_gemeldet = True
        return True

    def _bereite_befehl_auf(self, quat: np.ndarray):
        """Rampe + Kurshaltung + Begrenzung -> self.cmd. Ohne Freigabe und bei NotAus bleibt alles auf 0
        (Review L1/B5: sonst lief die Rampe in Bereitschaft schon hoch und die Policy sprang nach der Freigabe)."""
        steht = not self.bewegung_freigegeben or self.not_aus_aktiv
        if steht:
            self.cmd_ramped = np.zeros(3, dtype=np.float32)
        else:
            self.cmd_ramped = rampe(self.cmd_ramped, self.cmd_target, self.control_dt)
        self.cmd = self.cmd_ramped.copy()
        self.cmd[2] += self.kurshaltung.korrektur(quat, float(self.cmd_target[2]))
        if not self._cmd_clip_gemeldet and self.begrenzung.ausserhalb(self.cmd_target):
            self.get_logger().warn(
                f"cmd_vel {self.cmd_target.tolist()} liegt ausserhalb des Trainingsbereichs "
                f"[{self.begrenzung.cmd_min.tolist()} .. {self.begrenzung.cmd_max.tolist()}] - wird begrenzt.")
            self._cmd_clip_gemeldet = True
        self.cmd = self.begrenzung.begrenze(self.cmd)

    def _halte_position(self, quat: np.ndarray):
        """Positionshaltung nur im Zustand Balancieren (Korrektur getrennt in pos_hold_corr, nicht auf cmd
        addiert - der erste Versuch am 12.10. fiel genau an dieser Akkumulation um)."""
        if self.position_hold_aktiv and isinstance(self.state, BalancierenState) and self._odom_xyz is not None:
            self.pos_hold_corr[:] = self.positionshaltung.korrektur(self._odom_xyz, quat, self.cmd_target,
                                                                    self.cmd_ramped)
        else:
            self.positionshaltung.zuruecksetzen()
            self.pos_hold_corr[:] = 0.0

    # ---------------------------------------------------------------- Zustands-API (fuer states/)
    # Unterhalb dieser Groesse (je Achse) im angeforderten Befehl gilt "kein Bewegungswunsch".
    BEWEGUNG_SCHWELLE = 0.05

    def sensordaten_gueltig(self) -> bool:
        """Gelenke UND IMU mindestens einmal empfangen. (Frueher genuegte eine der beiden Quellen - mit der
        Altersbewertung, die beide braucht, meldete die Steuerung dann beim Start sofort einen Sensorausfall.)"""
        return self.bridge.sensor_alter_s() != float("inf")

    def bewegung_gewuenscht(self) -> bool:
        """Steht ein Bewegungswunsch an? Geprueft gegen den Wunsch (cmd_target), nicht gegen den Befehl an die Policy."""
        return bool(np.max(np.abs(self.cmd_target)) >= self.BEWEGUNG_SCHWELLE)

    def befehl_gehen(self) -> np.ndarray:
        """Aufbereiteter Fahrbefehl: Rampe + Kurshaltung, begrenzt."""
        return self.cmd.copy()

    def befehl_stehen(self, mit_positionshaltung: bool) -> np.ndarray:
        """Stehen: abklingende Rampe (weiches Abbremsen) + optional Positionshaltung; Drehrate mit Kurshaltung."""
        vx, vy = self.cmd_ramped[0], self.cmd_ramped[1]
        if mit_positionshaltung:
            vx, vy = vx + self.pos_hold_corr[0], vy + self.pos_hold_corr[1]
        return np.array([vx, vy, self.cmd[2]], dtype=np.float32)

    def sende_gelenkziele(self, ziele: np.ndarray) -> None:
        """Pruefen und senden: nicht endliche Werte -> Stoerung (nichts senden); sonst auf Gelenkgrenzen begrenzt."""
        ziele = np.asarray(ziele, dtype=np.float32)
        if not np.all(np.isfinite(ziele)):
            self._stoerung("Policy-Ausgabe ungueltig (NaN/inf)")
            return
        self.bridge.send_joint_commands(np.clip(ziele, self.grenze_unten, self.grenze_oben))

    # ---------------------------------------------------------------- Regeltakt
    def _sensor_takt(self):
        self._zaehler += 1
        if self._zaehler >= self._teiler:
            self._zaehler = 0
            self._control_loop()

    def _ueberwache_sensoren(self):
        """Sensorausfall auch erkennen, wenn keine Nachricht mehr kommt (im Sensortakt laeuft dann kein Regelschritt)."""
        if isinstance(self.state, (StartState, StoerungState)):
            return
        alter = self.bridge.sensor_alter_s()
        if alter > self.sensor_timeout_s:
            self._stoerung(f"Sensordaten seit {alter:.2f} s ausgeblieben ({self.bridge.sensor_detail()})")
            self._melde_zustand()

    def _control_loop(self):
        # Start-Gate: vor den ersten Sensordaten liefert die Bridge Nullen (eine echte Quaternion ist nie 0).
        # Ein NotAus in dieser Phase wird im Callback gemerkt und mit den ersten Daten wirksam (Review L6).
        if isinstance(self.state, StoerungState):
            self._melde_zustand()
            return
        if not self.sensordaten_gueltig():
            return
        quat, _ = self.bridge.read_imu()
        alter = self.bridge.sensor_alter_s()
        if alter > self.sensor_timeout_s:
            self._stoerung(f"Sensordaten seit {alter:.2f} s ausgeblieben ({self.bridge.sensor_detail()})")
            self._melde_zustand()
            return
        neigung = neigung_grad(quat)
        if neigung > self.sturz_neigung_grad:
            self._stoerung(f"Sturz erkannt (Neigung {neigung:.0f} Grad)")
            self._melde_zustand()
            return

        jetzt = time.monotonic()
        self._pruefe_watchdog(jetzt)
        self.cmd_target = self._balance_cmd.copy() if jetzt < self._balance_bis else self.cmd_wunsch.copy()
        self._bereite_befehl_auf(quat)
        self._halte_position(quat)

        # NotAus unterbricht jeden Zustand; nach der Entwarnung weiter ab Bereitschaft mit neuer Freigabe.
        if self.not_aus_aktiv:
            if not isinstance(self.state, NotAusState):
                self.get_logger().warn("NOT-AUS ausgeloest - Bewegungsregelung angehalten.")
                self.state = NotAusState()
                self.bewegung_freigegeben = False
        elif isinstance(self.state, NotAusState):
            self.get_logger().info("NOT-AUS aufgehoben - Zustandsautomat wird ab Bereitschaft fortgesetzt.")
            self.state = BereitschaftState()
        elif not self.bewegung_freigegeben and not isinstance(self.state, (BereitschaftState, StartState)):
            self.get_logger().info("Freigabe entzogen - Zustandswechsel nach Bereitschaft.")
            self.state = BereitschaftState()

        naechster = self.state.execute(self)
        if not isinstance(self.state, StoerungState):     # Stoerung waehrend execute (z. B. NaN) bleibt bestehen
            self.state = naechster
        self._melde_zustand()

    def _melde_zustand(self):
        """Zustandsname auf /steuerung/zustand (latched), nur bei Aenderung."""
        name = type(self.state).__name__.replace("State", "")
        if isinstance(self.state, StoerungState):
            name += f": {self.state.grund}"
        if name != self._gemeldeter_zustand:
            self._gemeldeter_zustand = name
            self.pub_zustand.publish(String(data=name))


def main(args=None):
    rclpy.init(args=args)
    node = SteuereinheitNode()
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
