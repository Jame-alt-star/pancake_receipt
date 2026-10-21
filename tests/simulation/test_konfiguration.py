"""Konsistenz der Konfiguration ueber Paketgrenzen (ohne ROS): Parameterdateien passen zu den Knoten,
Policy passt zum Robotermodell, Physiktakt passt zum Regeltakt."""
import ast
import os
import re

import mujoco
import yaml

from sim_helpers import BESCHREIBUNG, SIMULATION_DT, STEUER, WS, load_config


def _deklariert(quelldatei):
    """Parameternamen aus declare_parameter("...") und aus einer Tabelle PARAMETER = {...} (mujoco_node)."""
    with open(quelldatei) as f:
        text = f.read()
    namen = set(re.findall(r'declare_parameter\(\s*"([a-z_0-9]+)"', text))
    for knoten in ast.walk(ast.parse(text)):
        if isinstance(knoten, ast.Assign) and any(getattr(z, "id", None) == "PARAMETER" for z in knoten.targets):
            namen |= {k.value for k in knoten.value.keys}
    return namen


def _params(yaml_datei, knoten):
    with open(yaml_datei) as f:
        return set(yaml.safe_load(f)[knoten]["ros__parameters"])


def test_parameterdateien_nur_deklarierte_parameter():
    """Ein Tippfehler in einer Parameterdatei wuerde sonst still ignoriert (ROS setzt nur deklarierte Werte)."""
    sim = WS + "roboter_simulation/"
    faelle = [
        (sim + "config/simulation.yaml", "mujoco_bridge_node", sim + "roboter_simulation/mujoco_node.py"),
        (STEUER + "config/steuerung.yaml", "steuereinheit_node", STEUER + "roboter_steuerung/steuereinheit_node.py"),
        (STEUER + "config/steuerung.yaml", "balance_dienst_node", STEUER + "roboter_steuerung/balance_dienst_node.py"),
        (STEUER + "config/steuerung.yaml", "beinodometrie_node", STEUER + "roboter_steuerung/beinodometrie_node.py"),
    ]
    for datei, knoten, quelle in faelle:
        fremd = _params(datei, knoten) - _deklariert(quelle)
        assert not fremd, f"{os.path.basename(datei)}/{knoten}: nicht deklariert: {sorted(fremd)}"


def test_simulation_dt_gleich_im_test_und_im_system():
    with open(WS + "roboter_simulation/config/simulation.yaml") as f:
        dt = yaml.safe_load(f)["mujoco_bridge_node"]["ros__parameters"]["simulation_dt"]
    assert dt == SIMULATION_DT


def test_policy_passt_zum_robotermodell():
    cfg = load_config()
    m = mujoco.MjModel.from_xml_path(cfg["xml_path"])
    n = cfg["num_actions"]
    assert m.nu == n == len(cfg["kps"]) == len(cfg["kds"]) == len(cfg["default_angles"])
    assert os.path.exists(cfg["policy_path"])
    # Regeltakt ist ein ganzzahliges Vielfaches des Physiktakts (50 Hz Policy bei 500 Hz Physik)
    assert abs(cfg["control_decimation"] * SIMULATION_DT - cfg["regeltakt_s"]) < 1e-9


def test_geometrie_einmal_definiert():
    with open(BESCHREIBUNG + "config/roboter.yaml") as f:
        r = yaml.safe_load(f)
    assert 0.6 < r["becken_hoehe_stand"] < 1.0
    assert set(r["lidar"]) == {"parent", "xyz", "rpy"} and len(r["lidar"]["xyz"]) == len(r["lidar"]["rpy"]) == 3
    m = mujoco.MjModel.from_xml_path(BESCHREIBUNG + r["modell_xml"])
    assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, r["lidar"]["parent"]) >= 0     # Glied existiert im Modell
    urdf = open(BESCHREIBUNG + "modelle/g1_description/" + r["urdf"]).read()
    assert f'<link name="{r["lidar"]["parent"]}"' in urdf                                # ... und in der URDF
    with open(STEUER + "config/policy_29dof.yaml") as f:
        p = yaml.safe_load(f)
    assert "becken_hoehe_stand" not in p and "xml_path" not in p     # nur noch in roboter.yaml
