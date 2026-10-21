"""Logik des Steuerknotens ohne laufendes ROS2: NotAus, Freigabe, Watchdog, Befehlsaufbereitung, Sicherheits-
pruefungen (Sensorausfall, Sturz, ungueltige Policy-Ausgabe, Gelenkgrenzen), Vorrang des Balance-Eingangs.
Benoetigt eine gesourcte ROS2-/Workspace-Umgebung (rclpy, roboter_interfaces)."""
import numpy as np
import pytest

pytest.importorskip("rclpy")
nodemod = pytest.importorskip("roboter_steuerung.steuereinheit_node")
from roboter_steuerung.haltung import Kurshaltung, Positionshaltung  # noqa: E402
from roboter_steuerung.kommando import Kommandobegrenzung, Watchdog  # noqa: E402
from roboter_steuerung.states.bereitschaft_state import BereitschaftState  # noqa: E402
from roboter_steuerung.states.stoerung_state import StoerungState  # noqa: E402

AUFRECHT = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)


class _Log:
    def __init__(self): self.warnungen, self.fehler = [], []
    def warn(self, m): self.warnungen.append(m)
    def error(self, m): self.fehler.append(m)
    def info(self, m): pass


class _Bridge:
    """Attrappe der Hardware-Bridge: feste Messwerte, Alter einstellbar, gesendete Ziele werden gesammelt."""
    def __init__(self, n=3):
        self.quat, self.alter, self.gesendet = AUFRECHT.copy(), 0.01, []
        self.pos = np.full(n, 0.1, dtype=np.float32)
    def read_imu(self): return self.quat.copy(), np.zeros(3, dtype=np.float32)
    def read_joint_states(self): return self.pos.copy(), np.zeros_like(self.pos)
    def sensor_alter_s(self): return self.alter
    def sensor_detail(self): return f"Alter {self.alter}"
    def send_joint_commands(self, z): self.gesendet.append(np.asarray(z))


class _Policy:
    def __init__(self): self.ausgabe = np.zeros(3, dtype=np.float32)
    def compute_joint_targets(self, **kw): return self.ausgabe.copy()


class _Pub:
    def __init__(self): self.daten = []
    def publish(self, m): self.daten.append(m.data)


def make(cmd_target=(0.0, 0.0, 0.0)):
    n = object.__new__(nodemod.SteuereinheitNode)        # ohne Node.__init__ (kein rclpy-Kontext noetig)
    n.cmd_wunsch = np.array(cmd_target, dtype=np.float32)
    n.cmd_target = n.cmd_wunsch.copy()
    n._balance_cmd, n._balance_bis = np.zeros(3, dtype=np.float32), 0.0
    n.cmd_ramped = np.zeros(3, dtype=np.float32)
    n.cmd = np.zeros(3, dtype=np.float32)
    n.control_dt = 0.02
    n.begrenzung = Kommandobegrenzung([-0.5, -0.3, -1.0], [1.0, 0.3, 1.0])
    n._cmd_clip_gemeldet = False
    n.watchdog = Watchdog(0.0)
    n._timeout_gemeldet = False
    n.kurshaltung, n.positionshaltung = Kurshaltung(), Positionshaltung()
    n.position_hold_aktiv, n.pos_hold_corr, n._odom_xyz = False, np.zeros(2, dtype=np.float32), None
    n.bewegung_freigegeben, n.not_aus_aktiv, n.freigabe_per_cmd_vel = False, False, False   # Bereitschaft rechnet jeden Takt
    n.sensor_timeout_s, n.sturz_neigung_grad = 0.3, 45.0
    n.grenze_unten = np.array([-1.0, -1.0, -1.0], dtype=np.float32)
    n.grenze_oben = np.array([1.0, 1.0, 1.0], dtype=np.float32)
    n.bridge, n.controller, n.pub_zustand = _Bridge(), _Policy(), _Pub()
    n._gemeldeter_zustand = None
    n.state = BereitschaftState()
    n._log = _Log()
    n.get_logger = lambda: n._log
    return n


def test_watchdog_setzt_wunsch_auf_null_und_meldet_einmal():
    n = make(cmd_target=(0.4, 0.0, 0.1))
    n.watchdog = Watchdog(1.0)
    assert n._pruefe_watchdog(100.0) is False             # noch nie ein /cmd_vel empfangen
    n.watchdog.empfangen(100.0)
    assert n._pruefe_watchdog(100.5) is False and n.cmd_wunsch[0] == pytest.approx(0.4)
    assert n._pruefe_watchdog(101.5) is True
    assert n.cmd_wunsch.tolist() == [0.0, 0.0, 0.0] and len(n._log.warnungen) == 1
    assert n._pruefe_watchdog(103.0) is False             # nichts mehr zu tun, keine neue Warnung


def test_befehl_bleibt_null_ohne_freigabe_und_warnung_einmal():
    n = make(cmd_target=(2.0, 0.0, 0.0))
    n.bewegung_freigegeben = False
    n._bereite_befehl_auf(AUFRECHT)
    assert n.cmd.tolist() == [0.0, 0.0, 0.0] and n.cmd_ramped.tolist() == [0.0, 0.0, 0.0]
    n.bewegung_freigegeben = True
    for _ in range(100):
        n._bereite_befehl_auf(AUFRECHT)
    assert n.cmd[0] == pytest.approx(1.0) and n.cmd_ramped[0] == pytest.approx(2.0)   # Rampe ja, Policy begrenzt
    assert len(n._log.warnungen) == 1


def test_notaus_freigabe_und_cmd_vel_waehrend_notaus():
    """Review B1: /cmd_vel und Freigabe-Versuche waehrend NotAus duerfen nichts bewirken."""
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    n = make()
    n.freigabe_per_cmd_vel = True            # selbst im Handbetrieb-Modus
    n._not_aus_callback(Bool(data=True))
    assert n.bewegung_freigegeben is False and n.cmd_target.tolist() == [0.0, 0.0, 0.0]
    t = Twist(); t.linear.x = 0.4
    n._cmd_vel_callback(t)
    n._balance_callback(t)
    n._freigabe_callback(Bool(data=True))
    assert n.bewegung_freigegeben is False and n.cmd_wunsch.tolist() == [0.0, 0.0, 0.0] and n._balance_bis == 0.0
    assert any("abgelehnt" in w for w in n._log.warnungen)
    n._not_aus_callback(Bool(data=False))
    n._cmd_vel_callback(t)                    # nach Entwarnung: Ziel ja ...
    assert n.cmd_wunsch[0] == pytest.approx(0.4)
    n.freigabe_per_cmd_vel = False
    n.bewegung_freigegeben = False
    n._cmd_vel_callback(t)                    # ... aber ohne Freigabe-Kanal keine Freigabe
    assert n.bewegung_freigegeben is False
    n._freigabe_callback(Bool(data=True))
    assert n.bewegung_freigegeben is True


def test_policy_ausgabe_begrenzt_und_nan_fuehrt_zur_stoerung():
    n = make()
    n.sende_gelenkziele(np.array([0.5, 2.0, -3.0]))
    assert n.bridge.gesendet[-1].tolist() == pytest.approx([0.5, 1.0, -1.0])     # auf Gelenkgrenzen begrenzt
    n.sende_gelenkziele(np.array([0.1, np.nan, 0.0]))
    assert len(n.bridge.gesendet) == 1                                            # NaN wird nicht gesendet
    assert isinstance(n.state, StoerungState) and "NaN" in n.state.grund
    assert n.bewegung_freigegeben is False


def test_nan_waehrend_zustand_bleibt_stoerung():
    """Ein Zustand gibt nach execute sich selbst zurueck - das darf die Stoerung nicht ueberschreiben."""
    n = make()
    n.controller.ausgabe = np.array([np.nan, 0.0, 0.0], dtype=np.float32)
    n._control_loop()
    assert isinstance(n.state, StoerungState)
    assert n.pub_zustand.daten[-1].startswith("Stoerung")


def test_sensorausfall_fuehrt_zur_stoerung_und_es_wird_nichts_mehr_gesendet():
    n = make()
    n._control_loop()
    assert len(n.bridge.gesendet) == 1 and n.pub_zustand.daten[-1] == "Bereitschaft"
    n.bridge.alter = 0.5                                                          # > sensor_timeout_s
    n._control_loop(); n._control_loop()
    assert isinstance(n.state, StoerungState) and "Sensordaten" in n.state.grund
    assert len(n.bridge.gesendet) == 1 and len(n._log.fehler) == 1                # einmal gemeldet


def test_sturz_fuehrt_zur_stoerung():
    n = make()
    h = np.radians(60) / 2
    n.bridge.quat = np.array([np.cos(h), np.sin(h), 0.0, 0.0], dtype=np.float32)  # 60 Grad gekippt
    n._control_loop()
    assert isinstance(n.state, StoerungState) and "Sturz" in n.state.grund and not n.bridge.gesendet


def test_quittieren_nur_wenn_ursache_behoben():
    from types import SimpleNamespace
    n = make()
    n.bridge.alter = 0.5
    n._control_loop()
    antwort = lambda: n._quittieren_callback(None, SimpleNamespace(success=None, message=""))   # noqa: E731
    assert antwort().success is False                                             # Sensoren fehlen noch
    n.bridge.alter = 0.01
    n.not_aus_aktiv = True
    assert antwort().success is False                                             # NotAus aktiv
    n.not_aus_aktiv = False
    r = antwort()
    assert r.success is True and isinstance(n.state, BereitschaftState) and n.bewegung_freigegeben is False
    assert antwort().success is False                                             # keine Stoerung mehr


def test_balance_eingang_hat_vorrang_vor_cmd_vel():
    from geometry_msgs.msg import Twist
    n = make()
    t = Twist(); t.linear.x = 0.5
    n._cmd_vel_callback(t)
    n._control_loop()
    assert n.cmd_target[0] == pytest.approx(0.5)
    n._balance_callback(Twist())                                                  # Balance-Dienst: anhalten
    n._control_loop()
    assert n.cmd_target.tolist() == [0.0, 0.0, 0.0]                               # Vorrang, obwohl /cmd_vel 0.5 will
    n._balance_bis = 0.0                                                          # Vorrang abgelaufen
    n._control_loop()
    assert n.cmd_target[0] == pytest.approx(0.5)


def test_sensortakt_rechnet_jede_zweite_gelenknachricht():
    n = make()
    n._teiler, n._zaehler = 2, 0
    for _ in range(6):
        n._sensor_takt()
    assert len(n.bridge.gesendet) == 3                                            # 100 Hz Sensoren -> 50 Hz Regelung


def test_ueberwachung_erkennt_ausbleibende_sensoren_ohne_regelschritt():
    n = make()
    n._ueberwache_sensoren()
    assert isinstance(n.state, BereitschaftState)
    n.bridge.alter = 1.0
    n._ueberwache_sensoren()
    assert isinstance(n.state, StoerungState) and n.pub_zustand.daten[-1].startswith("Stoerung")


def test_start_mit_nur_einer_sensorquelle_ist_keine_stoerung():
    """Regression: kamen die Gelenkdaten vor der ersten IMU-Nachricht, meldete die Steuerung 'seit inf s' Stoerung."""
    from roboter_steuerung.states.start_state import StartState
    n = make()
    n.state = StartState()
    n.bridge.alter = float("inf")                       # eine Quelle fehlt noch
    n._control_loop(); n._ueberwache_sensoren()
    assert isinstance(n.state, StartState) and not n.bridge.gesendet
    n.bridge.alter = 0.01                               # beide da
    n._control_loop()
    assert isinstance(n.state, BereitschaftState)


def test_bridge_meldet_takt_erst_wenn_gelenke_und_imu_neu_sind():
    """Regression: der Takt kam nach jeder Gelenknachricht - die Policy rechnete dann mit einer 10 ms aelteren IMU."""
    from types import SimpleNamespace as N
    from roboter_steuerung.hardware.topic_bridge import TopicBridge
    b = object.__new__(TopicBridge)
    b.last_joint_pos = b.last_joint_vel = None
    b._t_gelenke = b._t_imu = None
    b._neu_gelenke = b._neu_imu = False
    takte = []
    b.bei_sensordaten = lambda: takte.append(1)
    gelenk = N(position=[0.1], velocity=[0.0])
    imu = N(orientation=N(w=1.0, x=0.0, y=0.0, z=0.0), angular_velocity=N(x=0.0, y=0.0, z=0.0))
    b._joint_callback(gelenk); assert not takte                 # nur Gelenke: noch kein Takt
    b._imu_callback(imu); assert len(takte) == 1                 # jetzt beide neu
    b._imu_callback(imu); b._imu_callback(imu); assert len(takte) == 1   # nur IMU: kein weiterer Takt
    b._joint_callback(gelenk); assert len(takte) == 2
