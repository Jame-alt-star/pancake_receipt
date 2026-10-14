"""Befehlsaufbereitung (kommando.py) und Haltungsregelung (haltung.py) ohne ROS."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from roboter_steuerung.haltung import POSITION_HOLD_MAX_SPEED, Kurshaltung, Positionshaltung  # noqa: E402
from roboter_steuerung.kommando import Kommandobegrenzung, Watchdog, rampe  # noqa: E402


def quat_yaw(deg):
    h = np.radians(deg) / 2
    return np.array([np.cos(h), 0, 0, np.sin(h)], dtype=np.float32)


def test_kurshaltung_akkumuliert_nicht():
    """Regression: cmd[2] darf im Stand nicht unbegrenzt anwachsen (war -572 rad/s nach 60 s)."""
    k, grenze = Kurshaltung(), Kommandobegrenzung([-0.5, -0.3, -0.2], [1.0, 0.3, 0.2])
    ramped = np.zeros(3, dtype=np.float32)
    k.korrektur(quat_yaw(0), 0.0)                    # Zielrichtung erfassen
    for _ in range(50 * 60):                         # 60 s, Roboter dauerhaft 20 Grad verdreht
        ramped = rampe(ramped, np.zeros(3, dtype=np.float32), 0.02)
        cmd = ramped.copy()
        cmd[2] += k.korrektur(quat_yaw(20), 0.0)
        cmd = grenze.begrenze(cmd)
    assert abs(cmd[2]) <= 0.2 + 1e-6, f"cmd[2]={cmd[2]}"


def test_rampe_erreicht_ziel_mit_rate():
    r, ziel = np.zeros(3, dtype=np.float32), np.array([0.5, 0, 0], dtype=np.float32)
    r = rampe(r, ziel, 0.02)
    assert r[0] == pytest.approx(0.02, abs=1e-6)      # CMD_RAMP_RATE 1.0 * 0.02 s
    for _ in range(100):
        r = rampe(r, ziel, 0.02)
    assert r[0] == pytest.approx(0.5, abs=1e-6)


def test_cmd_begrenzung():
    g = Kommandobegrenzung([-0.5, -0.3, -0.2], [1.0, 0.3, 0.2])
    assert g.ausserhalb(np.array([2.0, 0.0, 1.0])) and not g.ausserhalb(np.array([0.4, 0.0, 0.1]))
    assert g.begrenze(np.array([2.0, -1.0, 1.0], dtype=np.float32)).tolist() == pytest.approx([1.0, -0.3, 0.2])


def test_ohne_trainingsbereich_keine_begrenzung():
    g = Kommandobegrenzung.aus_policy({})
    assert g.begrenze(np.array([2.0, 0.0, 1.0], dtype=np.float32)).tolist() == pytest.approx([2.0, 0.0, 1.0])
    assert not g.ausserhalb(np.array([2.0, 0.0, 1.0]))


def test_kombigrenze_wz_bei_hoher_geschwindigkeit():
    g = Kommandobegrenzung.aus_policy({"cmd_training_range": {"lin_vel_x": [-0.5, 1.0], "lin_vel_y": [-0.3, 0.3],
                                                              "ang_vel_z": [-1.0, 1.0]},
                                       "cmd_wz_limit_high_speed": {"vx_ref": 0.7, "wz_at_vmax": 0.5}})
    for vx, wz, erwartet in ((0.4, 1.0, 1.0), (0.7, 1.0, 1.0), (0.85, 1.0, 0.75), (1.0, 1.0, 0.5), (1.0, -1.0, -0.5),
                             (2.0, 0.9, 0.5)):
        cmd = g.begrenze(np.array([vx, 0.0, wz], dtype=np.float32))
        assert cmd[2] == pytest.approx(erwartet, abs=1e-5), (vx, wz, cmd.tolist())


def test_watchdog():
    w = Watchdog(1.0)
    assert not w.abgelaufen(100.0)                    # noch nie ein /cmd_vel empfangen
    w.empfangen(100.0)
    assert not w.abgelaufen(100.5) and w.abgelaufen(101.5)
    assert not Watchdog(0.0).abgelaufen(1e9)          # 0 = aus


def test_kurshaltung_aus_bei_explizitem_drehkommando():
    k = Kurshaltung()
    k.korrektur(quat_yaw(0), 0.5)
    assert k.korrektur(quat_yaw(30), 0.5) == 0.0
    assert k.ziel_yaw == pytest.approx(np.radians(30), abs=1e-3)   # Ziel folgt der Ausrichtung


def test_positionshaltung_koerperkoordinaten_und_begrenzung():
    p, null = Positionshaltung(), np.zeros(3)
    assert p.korrektur([0.0, 0.0, 0.79], quat_yaw(0), null, null) == (0.0, 0.0)      # Ziel erfassen
    vx, vy = p.korrektur([-0.1, 0.0, 0.79], quat_yaw(0), null, null)                  # 0.1 m hinter dem Ziel
    assert vx == pytest.approx(0.1, abs=1e-5) and vy == pytest.approx(0.0, abs=1e-5)
    vx, vy = p.korrektur([-0.1, 0.0, 0.79], quat_yaw(90), null, null)                 # um 90 Grad gedreht
    assert abs(vx) < 1e-5 and vy == pytest.approx(-0.1, abs=1e-5)
    vx, _ = p.korrektur([-2.0, 0.0, 0.79], quat_yaw(0), null, null)
    assert vx == pytest.approx(POSITION_HOLD_MAX_SPEED)


def test_positionshaltung_aus_bei_kommando_oder_laufender_rampe():
    p, null = Positionshaltung(), np.zeros(3)
    p.korrektur([0.0, 0.0, 0.79], quat_yaw(0), null, null)
    assert p.korrektur([-0.1, 0.0, 0.79], quat_yaw(0), [0.4, 0, 0], null) == (0.0, 0.0) and p.ziel is None
    assert p.korrektur([-0.1, 0.0, 0.79], quat_yaw(0), null, [0.3, 0, 0]) == (0.0, 0.0) and p.ziel is None
