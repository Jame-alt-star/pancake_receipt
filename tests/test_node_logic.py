"""Tests der Knoten-Logik (Rampe, Kurshaltung, cmd-Begrenzung) ohne laufendes ROS2.
Benoetigt eine gesourcte ROS2-/Workspace-Umgebung (rclpy, roboter_steuerung).
"""
import numpy as np
import pytest

pytest.importorskip("rclpy")
nodemod = pytest.importorskip("roboter_steuerung.steuereinheit_node")
Node = nodemod.SteuereinheitNode


class _Log:
    def __init__(self): self.warnungen = []
    def warn(self, m): self.warnungen.append(m)
    def info(self, m): pass


def make(cmd_target=(0.0, 0.0, 0.0), rng=True):
    n = object.__new__(Node)          # ohne Node.__init__ (kein rclpy-Kontext noetig)
    n.cmd_target = np.array(cmd_target, dtype=np.float32)
    n.cmd_ramped = n.cmd_target.copy()
    n.cmd = n.cmd_ramped.copy()
    n.control_dt = 0.02
    n.target_yaw = None
    n.target_position = None
    n.pos_hold_corr = np.zeros(2, dtype=np.float32)
    n._cmd_clip_gemeldet = False
    if rng:
        n.cmd_min = np.array([-0.5, -0.3, -0.2], dtype=np.float32)
        n.cmd_max = np.array([1.0, 0.3, 0.2], dtype=np.float32)
    else:
        n.cmd_min = n.cmd_max = None
    n.wz_vx_ref = n.wz_at_vmax = None
    n.cmd_vel_timeout_s = 0.0
    n._last_cmd_vel_t = None
    n._timeout_gemeldet = False
    n._log = _Log()
    n.get_logger = lambda: n._log
    return n


def quat_yaw(deg):
    h = np.radians(deg) / 2
    return np.array([np.cos(h), 0, 0, np.sin(h)], dtype=np.float32)


def test_kurshaltung_akkumuliert_nicht():
    """Regression: cmd[2] darf im Stand nicht unbegrenzt anwachsen (war -572 rad/s nach 60 s)."""
    n = make()
    n._apply_heading_hold(quat_yaw(0))            # Zielrichtung erfassen
    for _ in range(50 * 60):                      # 60 s, Roboter dauerhaft 20 Grad verdreht
        n._ramp_cmd()
        n._apply_heading_hold(quat_yaw(20))
        n._clip_cmd()
    assert abs(n.cmd[2]) <= 0.2 + 1e-6, f"cmd[2]={n.cmd[2]}"


def test_rampe_erreicht_ziel_mit_rate():
    n = make(cmd_target=(0.0, 0.0, 0.0))
    n.cmd_target = np.array([0.5, 0, 0], dtype=np.float32)
    n._ramp_cmd()
    assert n.cmd[0] == pytest.approx(0.02, abs=1e-6)      # CMD_RAMP_RATE 1.0 * 0.02 s
    for _ in range(100):
        n._ramp_cmd()
    assert n.cmd[0] == pytest.approx(0.5, abs=1e-6)


def test_cmd_begrenzung_und_einmalige_warnung():
    n = make(cmd_target=(2.0, 0.0, 1.0))
    n.cmd = np.array([2.0, -1.0, 1.0], dtype=np.float32)
    n._clip_cmd(); n._clip_cmd()
    assert n.cmd.tolist() == pytest.approx([1.0, -0.3, 0.2])
    assert len(n._log.warnungen) == 1


def test_ohne_trainingsbereich_keine_begrenzung():
    n = make(cmd_target=(2.0, 0.0, 1.0), rng=False)
    n.cmd = np.array([2.0, 0.0, 1.0], dtype=np.float32)
    n._clip_cmd()
    assert n.cmd.tolist() == pytest.approx([2.0, 0.0, 1.0])


def test_kurshaltung_aus_bei_explizitem_drehkommando():
    n = make(cmd_target=(0.0, 0.0, 0.5))
    n._apply_heading_hold(quat_yaw(0))
    n._ramp_cmd()
    n._apply_heading_hold(quat_yaw(30))
    assert n.target_yaw == pytest.approx(np.radians(30), abs=1e-3)   # Ziel folgt der Ausrichtung


def test_positionshaltung_koerperkoordinaten_und_begrenzung():
    n = make()
    n._apply_position_hold(np.array([0.0, 0.0, 0.79]), quat_yaw(0))     # Ziel erfassen
    assert n.target_position is not None and n.pos_hold_corr.tolist() == [0.0, 0.0]
    # Roboter 0.1 m hinter dem Ziel (Welt -x), Yaw 0 -> soll vorwaerts (vx > 0)
    n._apply_position_hold(np.array([-0.1, 0.0, 0.79]), quat_yaw(0))
    assert n.pos_hold_corr[0] == pytest.approx(0.1, abs=1e-5) and n.pos_hold_corr[1] == pytest.approx(0.0, abs=1e-5)
    # gleiche Weltabweichung, Roboter um 90 Grad gedreht -> Ziel liegt rechts vom Roboter (vy < 0)
    n._apply_position_hold(np.array([-0.1, 0.0, 0.79]), quat_yaw(90))
    assert abs(n.pos_hold_corr[0]) < 1e-5 and n.pos_hold_corr[1] == pytest.approx(-0.1, abs=1e-5)
    # grosse Abweichung wird auf POSITION_HOLD_MAX_SPEED begrenzt
    n._apply_position_hold(np.array([-2.0, 0.0, 0.79]), quat_yaw(0))
    assert n.pos_hold_corr[0] == pytest.approx(nodemod.POSITION_HOLD_MAX_SPEED)


def test_positionshaltung_aus_bei_kommando_oder_laufender_rampe():
    n = make()
    n._apply_position_hold(np.array([0.0, 0.0, 0.79]), quat_yaw(0))
    n.cmd_target = np.array([0.4, 0, 0], dtype=np.float32)          # Nutzer will gehen
    n._apply_position_hold(np.array([-0.1, 0.0, 0.79]), quat_yaw(0))
    assert n.target_position is None and n.pos_hold_corr.tolist() == [0.0, 0.0]
    n.cmd_target = np.zeros(3, dtype=np.float32)                    # Rampe noch nicht abgeklungen
    n.cmd_ramped = np.array([0.3, 0, 0], dtype=np.float32)
    n._apply_position_hold(np.array([-0.1, 0.0, 0.79]), quat_yaw(0))
    assert n.target_position is None


def test_kombigrenze_wz_bei_hoher_geschwindigkeit():
    n = make()
    n.cmd_min = np.array([-0.5, -0.3, -1.0], dtype=np.float32)
    n.cmd_max = np.array([1.0, 0.3, 1.0], dtype=np.float32)
    n.wz_vx_ref, n.wz_at_vmax = 0.7, 0.5
    for vx, wz, erwartet in ((0.4, 1.0, 1.0), (0.7, 1.0, 1.0), (0.85, 1.0, 0.75), (1.0, 1.0, 0.5), (1.0, -1.0, -0.5),
                             (2.0, 0.9, 0.5)):
        n.cmd = np.array([vx, 0.0, wz], dtype=np.float32)
        n._clip_cmd()
        assert n.cmd[2] == pytest.approx(erwartet, abs=1e-5), (vx, wz, n.cmd.tolist())


def test_cmd_vel_watchdog():
    n = make(cmd_target=(0.4, 0.0, 0.1))
    n.cmd_vel_timeout_s = 1.0
    assert n._check_cmd_timeout(100.0) is False          # noch nie ein /cmd_vel empfangen
    n._last_cmd_vel_t = 100.0
    assert n._check_cmd_timeout(100.5) is False and n.cmd_target[0] == pytest.approx(0.4)
    assert n._check_cmd_timeout(101.5) is True            # > 1 s ohne Nachricht
    assert n.cmd_target.tolist() == [0.0, 0.0, 0.0] and len(n._log.warnungen) == 1
    assert n._check_cmd_timeout(103.0) is False           # nichts mehr zu tun, keine neue Warnung
    n.cmd_vel_timeout_s = 0.0
    n.cmd_target = np.array([0.4, 0, 0], dtype=np.float32)
    assert n._check_cmd_timeout(500.0) is False           # Watchdog aus
