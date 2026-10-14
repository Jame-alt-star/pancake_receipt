"""Logik des Steuerknotens ohne laufendes ROS2: NotAus, Freigabe, Watchdog, Befehlsaufbereitung im Knoten.
Benoetigt eine gesourcte ROS2-/Workspace-Umgebung (rclpy, roboter_steuerung)."""
import numpy as np
import pytest

pytest.importorskip("rclpy")
nodemod = pytest.importorskip("roboter_steuerung.steuereinheit_node")
from roboter_steuerung.haltung import Kurshaltung, Positionshaltung  # noqa: E402
from roboter_steuerung.kommando import Kommandobegrenzung, Watchdog  # noqa: E402


class _Log:
    def __init__(self): self.warnungen = []
    def warn(self, m): self.warnungen.append(m)
    def info(self, m): pass


def make(cmd_target=(0.0, 0.0, 0.0)):
    n = object.__new__(nodemod.SteuereinheitNode)        # ohne Node.__init__ (kein rclpy-Kontext noetig)
    n.cmd_target = np.array(cmd_target, dtype=np.float32)
    n.cmd_ramped = np.zeros(3, dtype=np.float32)
    n.cmd = np.zeros(3, dtype=np.float32)
    n.control_dt = 0.02
    n.begrenzung = Kommandobegrenzung([-0.5, -0.3, -1.0], [1.0, 0.3, 1.0])
    n._cmd_clip_gemeldet = False
    n.watchdog = Watchdog(0.0)
    n._timeout_gemeldet = False
    n.kurshaltung, n.positionshaltung = Kurshaltung(), Positionshaltung()
    n.bewegung_freigegeben, n.not_aus_aktiv, n.freigabe_per_cmd_vel = True, False, False
    n._log = _Log()
    n.get_logger = lambda: n._log
    return n


def test_watchdog_setzt_ziel_auf_null_und_meldet_einmal():
    n = make(cmd_target=(0.4, 0.0, 0.1))
    n.watchdog = Watchdog(1.0)
    assert n._pruefe_watchdog(100.0) is False             # noch nie ein /cmd_vel empfangen
    n.watchdog.empfangen(100.0)
    assert n._pruefe_watchdog(100.5) is False and n.cmd_target[0] == pytest.approx(0.4)
    assert n._pruefe_watchdog(101.5) is True
    assert n.cmd_target.tolist() == [0.0, 0.0, 0.0] and len(n._log.warnungen) == 1
    assert n._pruefe_watchdog(103.0) is False             # nichts mehr zu tun, keine neue Warnung


def test_befehl_bleibt_null_ohne_freigabe_und_warnung_einmal():
    q = np.array([1.0, 0, 0, 0], dtype=np.float32)
    n = make(cmd_target=(2.0, 0.0, 0.0))
    n.bewegung_freigegeben = False
    n._bereite_befehl_auf(q)
    assert n.cmd.tolist() == [0.0, 0.0, 0.0] and n.cmd_ramped.tolist() == [0.0, 0.0, 0.0]
    n.bewegung_freigegeben = True
    for _ in range(100):
        n._bereite_befehl_auf(q)
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
    n._freigabe_callback(Bool(data=True))
    assert n.bewegung_freigegeben is False and n.cmd_target.tolist() == [0.0, 0.0, 0.0]
    assert any("abgelehnt" in w for w in n._log.warnungen)
    n._not_aus_callback(Bool(data=False))
    n._cmd_vel_callback(t)                    # nach Entwarnung: Ziel ja ...
    assert n.cmd_target[0] == pytest.approx(0.4)
    n.freigabe_per_cmd_vel = False
    n.bewegung_freigegeben = False
    n._cmd_vel_callback(t)                    # ... aber ohne Freigabe-Kanal keine Freigabe
    assert n.bewegung_freigegeben is False
    n._freigabe_callback(Bool(data=True))
    assert n.bewegung_freigegeben is True
