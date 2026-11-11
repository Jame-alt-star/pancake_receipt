"""Greif-Dienst (Layer 3): Aktion /greife (roboter_interfaces/GreifAktion) - Gegenstand beidhaendig greifen oder ablegen.

Duenne ROS-Huelle um die Ablaufsteuerung greifen/dienst.py (dort Ablauf und Begruendungen). Je Takt (50 Hz):
Odometrie, Gelenkwinkel und die Lagen aus der Wahrnehmung (/objekte/<name>, Pose relativ zum Becken) einsammeln,
Greifauftrag.takt() rechnen und die Ausgabe senden:
  /cmd_vel/greifen  Anfahrt an den Tisch und Zuruecktreten (Vorrang-Eingang der Steuereinheit)
  /arme/kommando    Armfuehrung (Modus FUEHREN/HALTEN/AUSBLENDEN, 14 Armgelenke, Taillenanteil)
  /hand/kommando    Fingerziele mit kp/kd (wie Unitrees Dex3-HandCmd)
Nach erfolgreichem Greifen haelt der Dienst die Tragepose weiter (sendet FUEHREN mit 20 Hz), bis abgelegt wird; die
Steuereinheit wuerde ohne Kommando ohnehin halten (armfuehrung.py). Abbruch (Cancel, NotAus): Fahrbefehl 0, Arme
HALTEN - nie loslassen.
"""
import dataclasses
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from roboter_beschreibung import lade as lade_roboter
from roboter_interfaces.action import GreifAktion
from roboter_interfaces.msg import Armkommando, Handkommando
from roboter_steuerung.control.rl_locomotion_controller_29dof import MUJOCO_JOINT_ORDER
from roboter_steuerung.greifen import ablauf as greifablauf
from roboter_steuerung.greifen.dienst import AUSBLENDEN, FUEHREN, HALTEN, Eingang, Greifauftrag, Greifparameter
from roboter_steuerung.greifen.kinematik import Armkinematik
from roboter_steuerung.greifen.manipulation import ARM_INDIZES
from roboter_steuerung.steuereinheit_node import lade_policy

TAKT_S = 0.02


def _quat(o):
    return np.array([o.w, o.x, o.y, o.z], dtype=float)


class GreifDienstNode(Node):

    def __init__(self):
        super().__init__("greif_dienst_node")
        # Alle Felder von Greifparameter sind ROS-Parameter (gleicher Name, gleicher Standardwert); so bleiben
        # Dataclass und Knoten nicht auseinander (ein umbenanntes Feld liess den Knoten beim Start abstuerzen).
        d = Greifparameter()
        namen = [f.name for f in dataclasses.fields(Greifparameter)]
        for name in namen:
            self.declare_parameter(name, float(getattr(d, name)))
        self.par = Greifparameter(**{n: float(self.get_parameter(n).value) for n in namen})
        roboter = lade_roboter()
        if not roboter.get("modell_xml_haende"):
            raise RuntimeError("roboter.yaml ohne modell_xml_haende - Greifen braucht das Modell mit Haenden")
        from ament_index_python.packages import get_package_share_directory
        policy = lade_policy(get_package_share_directory("roboter_steuerung"), "policy_29dof.yaml")
        self.ruhe = np.array(policy["default_angles"], dtype=np.float32)
        self.kin = Armkinematik(roboter["modell_xml_haende"], MUJOCO_JOINT_ORDER)
        self.armnamen = [MUJOCO_JOINT_ORDER[i] for i in ARM_INDIZES]

        self._lock = threading.Lock()
        self._odom = None              # (pos, quat)
        self._gelenke = None           # 29 Koerpergelenke in MUJOCO_JOINT_ORDER
        self._lagen = {}               # name -> (t_empfang, pos_rel, quat_rel)
        self._abos = {}
        self.not_aus = False
        self._sim_t = None             # Simulationszeit (/clock); ohne Uhr gilt die Wanduhr
        self.freigabe = None           # None = noch nicht gehoert; False waehrend eines Auftrags bricht ihn ab
        self._reserviert = False       # Auftrag angenommen (zwischen goal_callback und Ende), unter _lock
        # Zustand ueber Auftraege hinweg: gehaltene Arme/Finger nach dem Greifen
        self.gehalten = None           # (arme, haende, taille, objekt) oder None
        self._auftrag_laeuft = False

        cb = ReentrantCallbackGroup()
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10, callback_group=cb)
        self.create_subscription(JointState, "/joint_states", self._gelenke_cb, 10, callback_group=cb)
        # wie in der Steuerung latched: ein NotAus vor dem Start des Dienstes darf nicht "unbekannt = ok" sein
        not_aus_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(Bool, "/not_aus", self._not_aus_cb, not_aus_qos, callback_group=cb)
        self.create_subscription(Bool, "/freigabe", self._freigabe_cb, 10, callback_group=cb)
        # Zeitbasis der Ablaeufe = Simulationszeit: unter Last (Nav2, Lidar) laeuft die Simulation langsamer als die
        # Wanduhr; mit Wanduhr eilten die Arme der Physik voraus (Kiste beim Greifen nicht gehalten, 11.11.2026).
        self.create_subscription(Clock, "/clock", self._clock_cb, 10, callback_group=cb)
        self.create_service(Trigger, "/greife/freigeben", self._freigeben, callback_group=cb)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel/greifen", 10)
        self.pub_arme = self.create_publisher(Armkommando, "/arme/kommando", 10)
        self.pub_hand = self.create_publisher(Handkommando, "/hand/kommando", 10)
        self.action = ActionServer(self, GreifAktion, "/greife", execute_callback=self._execute,
                                   goal_callback=self._goal_ok, cancel_callback=lambda g: CancelResponse.ACCEPT,
                                   callback_group=cb)
        self.create_timer(0.05, self._halten_senden, callback_group=cb)
        self.get_logger().info("Greif-Dienst bereit (/greife).")

    # ------------------------------------------------------------ Eingaenge
    def _odom_cb(self, m: Odometry):
        p = m.pose.pose.position
        with self._lock:
            self._odom = (np.array([p.x, p.y, p.z]), _quat(m.pose.pose.orientation))

    def _gelenke_cb(self, m: JointState):
        if len(m.name) != len(m.position):
            return
        idx = {n: i for i, n in enumerate(m.name)}
        if not all(n in idx for n in MUJOCO_JOINT_ORDER):
            return
        with self._lock:
            self._gelenke = np.array([m.position[idx[n]] for n in MUJOCO_JOINT_ORDER], dtype=np.float32)

    def _not_aus_cb(self, m: Bool):
        self.not_aus = bool(m.data)

    def _clock_cb(self, m: Clock):
        self._sim_t = m.clock.sec + 1e-9 * m.clock.nanosec

    def _jetzt(self) -> float:
        return self._sim_t if self._sim_t is not None else time.monotonic()

    def _freigabe_cb(self, m: Bool):
        self.freigabe = bool(m.data)

    def _freigeben(self, req, res):
        """Notausgang aus 'haelt etwas' (Mission abgebrochen, Auftrag unterbrochen): Haende oeffnen, Arme an die Policy
        zurueck, Merker loeschen. Der Gegenstand faellt dabei, wenn er noch in den Haenden liegt - nur im Stand und
        ohne laufenden Auftrag."""
        with self._lock:
            if self._reserviert:
                res.success, res.message = False, "Auftrag laeuft"
                return res
            self.gehalten = None
        if self.not_aus or self.freigabe is False:
            res.success, res.message = False, "NotAus oder keine Freigabe: Steuerung haelt die Arme"
            return res
        self._sende_haende(greifablauf.HAND_OFFEN)
        self._sende_arme(AUSBLENDEN, None, 0.0)
        res.success, res.message = True, "Haende offen, Arme an die Policy zurueck"
        return res

    def _abo(self, name: str):
        if name and name not in self._abos:
            def cb(m: PoseStamped, n=name):
                p = m.pose.position
                with self._lock:
                    self._lagen[n] = (time.monotonic(), np.array([p.x, p.y, p.z]), _quat(m.pose.orientation))
            self._abos[name] = self.create_subscription(PoseStamped, f"/objekte/{name}", cb, 10)

    def _lage(self, name, max_alter=0.5):
        with self._lock:
            v = self._lagen.get(name)
        if v is None or time.monotonic() - v[0] > max_alter:
            return None
        return v[1], v[2]

    # ------------------------------------------------------------ Ausgaenge
    def _sende(self, a):
        if a.cmd is not None:
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = map(float, a.cmd)
            self.pub_cmd.publish(t)
        if a.arm_modus:
            self._sende_arme(a.arm_modus, a.arme, a.taille)
        if a.haende is not None:
            self._sende_haende(a.haende)

    def _sende_arme(self, modus, arme, taille):
        m = Armkommando()
        m.modus = int(modus)
        if modus == FUEHREN and arme is not None:
            m.namen = self.armnamen
            m.ziele = [float(x) for x in arme]
            m.taille_anteil = float(taille)
        self.pub_arme.publish(m)

    def _sende_haende(self, haende):
        m = Handkommando()
        m.q = [float(x) for x in haende]
        m.kp = [self.par.hand_kp] * len(haende)
        m.kd = [self.par.hand_kd] * len(haende)
        self.pub_hand.publish(m)

    def _halten_senden(self):
        """Zwischen den Auftraegen: Tragepose weiter fuehren, solange ein Objekt gehalten wird."""
        if self._auftrag_laeuft or self._reserviert or self.gehalten is None:
            return
        arme, haende, taille, _ = self.gehalten
        self._sende_arme(HALTEN if self.not_aus else FUEHREN, arme, taille)
        self._sende_haende(haende)

    # ------------------------------------------------------------ Aktion
    def _goal_ok(self, goal):
        if goal.modus not in ("greifen", "ablegen"):
            return GoalResponse.REJECT
        with self._lock:                 # atomar reservieren: zwei schnell aufeinander folgende Ziele
            if self._reserviert:
                return GoalResponse.REJECT
            self._reserviert = True
        return GoalResponse.ACCEPT

    def _eingang(self, objekt, ablage):
        with self._lock:
            odom, gelenke = self._odom, self._gelenke
        if odom is None or gelenke is None:
            return None
        return Eingang(t=self._jetzt(), pos=odom[0], quat=odom[1], gelenke=gelenke,
                       objekt=self._lage(objekt), ablage=self._lage(ablage) if ablage else None)

    def _execute(self, goal_handle):
        try:
            return self._ausfuehren(goal_handle)
        finally:
            with self._lock:
                self._reserviert = False

    def _ausfuehren(self, goal_handle):
        req = goal_handle.request
        res = GreifAktion.Result()
        self._abo(req.objekt)
        self._abo(req.ablage)
        if req.modus == "ablegen" and self.gehalten is None:
            goal_handle.abort()
            res.erfolgreich, res.ergebnis, res.wiederholbar = False, "nichts gehalten - erst greifen", False
            return res
        if req.modus == "greifen" and self.gehalten is not None:
            goal_handle.abort()
            res.erfolgreich, res.ergebnis, res.wiederholbar = False, "haelt schon etwas - erst ablegen", False
            return res
        if self.gehalten is not None:
            arme, haende, taille, _ = self.gehalten
            auftrag = Greifauftrag(req.modus, self.kin, self.ruhe, self.par, arme, haende, taille)
        else:
            abgang = (req.abgang_x, req.abgang_y) if req.hat_abgang else None
            auftrag = Greifauftrag(req.modus, self.kin, self.ruhe, self.par, abgang=abgang)
        self._auftrag_laeuft = True
        self.get_logger().info(f"Auftrag {req.modus} '{req.objekt}'" + (f" auf '{req.ablage}'" if req.ablage else ""))
        fb = GreifAktion.Feedback()
        try:
            t_warten = time.monotonic()
            letzte = None
            while rclpy.ok():
                if goal_handle.is_cancel_requested or self.not_aus or self.freigabe is False:
                    self._anhalten(auftrag)
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    res.erfolgreich, res.ergebnis = False, "abgebrochen" + (
                        " (NotAus)" if self.not_aus else " (Freigabe entzogen)" if self.freigabe is False else "")
                    res.wiederholbar = False
                    return res
                e = self._eingang(req.objekt, req.ablage)
                if e is None:
                    if time.monotonic() - t_warten > 5.0:
                        goal_handle.abort()
                        res.erfolgreich, res.ergebnis, res.wiederholbar = False, "keine Odometrie/Gelenkwinkel", True
                        return res
                    time.sleep(TAKT_S)
                    continue
                a = auftrag.takt(e)
                if (auftrag.phase, a.phase) != letzte:
                    letzte = (auftrag.phase, a.phase)
                    ob = e.objekt[0] if e.objekt is not None else None
                    self.get_logger().info(
                        f"Phase {auftrag.phase}/{a.phase}: Objekt rel. Becken "
                        f"{None if ob is None else [round(float(x), 3) for x in ob]}, Basis "
                        f"{[round(float(x), 2) for x in e.pos[:2]]}")
                self._sende(a)
                fb.status, fb.fortschritt = a.phase, float(a.fortschritt)
                goal_handle.publish_feedback(fb)
                if a.fertig:
                    self._abschliessen(req, auftrag, a)
                    (goal_handle.succeed if a.erfolg else goal_handle.abort)()
                    res.erfolgreich, res.ergebnis, res.wiederholbar = bool(a.erfolg), a.meldung, bool(a.wiederholbar)
                    self.get_logger().info(f"Auftrag {req.modus}: {a.meldung}")
                    return res
                time.sleep(TAKT_S)
            res.erfolgreich, res.ergebnis = False, "beendet"
            return res
        finally:
            self.pub_cmd.publish(Twist())
            self._auftrag_laeuft = False

    def _anhalten(self, auftrag):
        """Abbruch: stehen bleiben, Arme halten (nichts loslassen)."""
        self.pub_cmd.publish(Twist())
        if auftrag.arme is not None and not auftrag.frei:
            self._sende_arme(HALTEN, None, 0.0)
            self.gehalten = (auftrag.arme, auftrag.haende, auftrag.taille, None)
        else:
            # nichts mehr in den Haenden (Ablegen schon losgelassen): kein Dauer-Halten ohne Ausweg
            self.gehalten = None

    def _abschliessen(self, req, auftrag, a):
        """Haelt der Auftrag am Ende noch Arme (gegriffen, oder Ablegen ohne Ablagestelle abgebrochen), wird weiter
        gehalten; sonst sind die Arme an die Policy zurueckgegeben und die Haende offen."""
        if auftrag.arme is None:
            self.gehalten = None
            self._sende_haende(greifablauf.HAND_OFFEN)
        else:
            self.gehalten = (auftrag.arme.copy(), auftrag.haende.copy(), auftrag.taille, req.objekt)


def main(args=None):
    rclpy.init(args=args)
    node = GreifDienstNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
