"""Schnelle Tests (kein ROS2, kein Viewer): echter Controller + echtes Modell, synchroner Kreis.
Aufruf: python3 -m pytest tests/simulation/test_closed_loop.py -v
"""
import pytest
from sim_helpers import run_loop

pytestmark = pytest.mark.timeout(600) if hasattr(pytest.mark, "timeout") else []


def kurs(vx, ty=[None]):
    """Kurshaltung wie im Knoten (kp=1, +-0.2, Ziel = Yaw beim ersten Aufruf)."""
    state = {"ty": None}

    def f(t, yaw):
        if state["ty"] is None:
            state["ty"] = yaw
        import numpy as np
        err = (state["ty"] - yaw + 180) % 360 - 180
        wz = float(np.clip(np.radians(err), -0.2, 0.2))
        return [min(vx, t * 1.0), 0.0, wz]
    return f


def test_stehen_30s_stabil():
    r = run_loop(lambda t, y: [0, 0, 0], 30)
    assert r.fell_at is None and r.z > 0.7
    assert abs(r.x) < 1.0 and abs(r.y) < 1.0


def test_stehen_beine_still():
    r = run_loop(lambda t, y: [0, 0, 0], 20)
    assert r.leg_dq < 0.01, f"Beine bewegen sich im Stand: {r.leg_dq:.4f} rad/s"


def test_gehen_vorwaerts_10s():
    r = run_loop(kurs(0.4), 10)
    assert r.fell_at is None and r.z > 0.7
    assert r.x > 2.5, f"zu langsam: x={r.x:.2f}"


def test_gehen_gerade_mit_kurshaltung_20s():
    r = run_loop(kurs(0.6), 20)
    assert r.fell_at is None
    assert abs(r.y) < 0.5, f"seitlicher Versatz y={r.y:.2f}"
    assert abs(r.yaw) < 5, f"Yaw {r.yaw:.1f}"


def test_gehen_dann_stoppen_beine_still():
    r = run_loop(lambda t, y: [0.4 if t < 8 else 0.0, 0, 0], 16)
    assert r.fell_at is None
    assert r.leg_dq < 0.02, f"Beine nach Stopp nicht still: {r.leg_dq:.4f}"


@pytest.mark.parametrize("delay_ms", [20, 40])
def test_robust_gegen_sensorverzoegerung(delay_ms):
    r = run_loop(kurs(0.4), 10, delay_ms=delay_ms)
    assert r.fell_at is None


def test_robust_gegen_takt_jitter():
    r = run_loop(kurs(0.4), 10, jitter=3)
    assert r.fell_at is None


@pytest.mark.parametrize("dec", [8, 12])
def test_robust_gegen_tick_abweichung(dec):
    r = run_loop(lambda t, y: [0, 0, 0], 20, decimation=dec)
    assert r.fell_at is None


def test_ohne_regler_faellt_um():
    """Dokumentiert das Startrennen: reine PD-Haltung ohne Policy kippt (siehe wait_for_controller)."""
    import mujoco, numpy as np
    from sim_helpers import load_config
    cfg = load_config()
    m = mujoco.MjModel.from_xml_path(cfg["xml_path"])
    d = mujoco.MjData(m)
    m.opt.timestep = 0.002
    tgt = np.array(cfg["default_angles"], dtype=np.float32)
    kps = np.array(cfg["kps"], dtype=np.float32); kds = np.array(cfg["kds"], dtype=np.float32)
    d.qpos[7:] = tgt; mujoco.mj_forward(m, d)
    for _ in range(int(8 / 0.002)):
        d.ctrl[:] = (tgt - d.qpos[7:]) * kps - d.qvel[6:] * kds
        mujoco.mj_step(m, d)
    assert d.qpos[2] < 0.4, "PD-Haltung ohne Policy blieb stehen - wait_for_controller waere evtl. unnoetig"


def test_welten_laden_roboter_plus_umgebung():
    """Jede Welt laesst sich mit dem Robotermodell zusammensetzen; Hindernisse liegen in geom group 2 (Lidar).
    Welten duerfen Koerper und Gelenke mitbringen (libero_kueche: Schubladen, Tueren, freie Objekte), aber keine
    Antriebe; die Robotergelenke behalten ihre qpos/qvel-Adressen (mujoco_node liest sie ueber die Aktuatoren)."""
    import os
    import sys
    import mujoco
    import numpy as np
    from sim_helpers import load_config, BRINGUP, WS
    sys.path.insert(0, BRINGUP)
    from roboter_simulation.szene import lade_modell
    roboter = load_config()["xml_path"]
    nur_roboter = mujoco.MjModel.from_xml_path(roboter)
    welten = WS + "roboter_welten/welten/"
    for name in sorted(os.listdir(welten)):
        m = lade_modell(roboter, welten + name + "/szene.xml")
        assert m.nu == nur_roboter.nu, name
        assert m.nbody >= nur_roboter.nbody, name
        for j in range(nur_roboter.njnt):
            assert m.joint(j).name == nur_roboter.joint(j).name, (name, j)
        gelenke = m.actuator_trnid[:, 0]
        assert np.array_equal(m.jnt_qposadr[gelenke], nur_roboter.jnt_qposadr[nur_roboter.actuator_trnid[:, 0]]), name
        assert np.array_equal(m.jnt_dofadr[gelenke], nur_roboter.jnt_dofadr[nur_roboter.actuator_trnid[:, 0]]), name
        if name != "leer":      # Lidar-Ziele: Hindernisse (group 2) oder Ersatzquader importierter Welten (group 4)
            assert np.isin(m.geom_group, (2, 4)).sum() > 0, name


def test_libero_kueche_objekte_bleiben_liegen():
    """libero_kueche: die LIBERO-Objekte stehen nach 2 s Simulation (ohne Roboter) noch auf Tisch bzw. Arbeitsplatte
    (< 1 cm verschoben, nicht umgekippt) und die Gelenke von Schrank, Mikrowelle und Herd bleiben geschlossen."""
    import mujoco
    import numpy as np
    from sim_helpers import WS
    m = mujoco.MjModel.from_xml_path(WS + "roboter_welten/welten/libero_kueche/szene.xml")
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    frei = [j for j in range(m.njnt) if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
    assert len(frei) == 6
    start = {j: d.qpos[m.jnt_qposadr[j]:m.jnt_qposadr[j] + 7].copy() for j in frei}
    for _ in range(int(2.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    for j in frei:
        a = m.jnt_qposadr[j]
        assert np.linalg.norm(d.qpos[a:a + 3] - start[j][:3]) < 0.01, m.joint(j).name
        # Drehung gegenueber dem Start (Ketchup steht absichtlich gekippt: relativ messen)
        q0, q1 = start[j][3:], d.qpos[a + 3:a + 7]
        assert abs(float(np.dot(q0, q1))) > np.cos(np.radians(5) / 2), m.joint(j).name
    for j in range(m.njnt):
        if m.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE:
            assert abs(d.qpos[m.jnt_qposadr[j]]) < 0.01, m.joint(j).name


def test_robocasa_kueche7_tueren_bleiben_zu_und_lidar_sieht_kueche():
    """robocasa_kueche7: alle Tueren und Schubladen (105 Gelenke) bleiben in 2 s Simulation zu, obwohl die
    Kollisionskoerper der Kueche nicht untereinander kollidieren (contype 0); die Ersatzquader (group 4) sind
    unsichtbar und ohne Kollision, und ein Lidar-Strahl vom Start Richtung Insel trifft sie."""
    import mujoco
    import numpy as np
    from sim_helpers import WS
    m = mujoco.MjModel.from_xml_path(WS + "roboter_welten/welten/robocasa_kueche7/szene.xml")
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    assert m.njnt == 105
    q0 = d.qpos.copy()
    for _ in range(int(2.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    assert np.max(np.abs(d.qpos - q0)) < 0.02
    ersatz = m.geom_group == 4
    assert ersatz.sum() > 50
    assert not np.any(m.geom_contype[ersatz] | m.geom_conaffinity[ersatz])
    kollision = m.geom_group == 3
    assert np.all(m.geom_contype[kollision] == 0) and np.all(m.geom_conaffinity[kollision] == 1)
    gid = np.array([-1], dtype=np.int32)
    maske = np.array([0, 0, 1, 0, 1, 0], dtype=np.uint8)            # wie lidar.py: Gruppen 2 und 4
    dist = mujoco.mj_ray(m, d, np.array([0.0, 0.0, 0.5]), np.array([1.0, 0.0, 0.0]), maske, 1, -1, gid)
    assert gid[0] >= 0 and 0.7 < dist < 0.9, (dist, m.geom(gid[0]).name if gid[0] >= 0 else None)   # Insel/Hocker ab x 0.76
