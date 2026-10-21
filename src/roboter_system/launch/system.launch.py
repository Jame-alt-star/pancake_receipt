"""Gesamtsystem: setzt die Schichten zusammen. Jede Schicht startet nur sich selbst (eigene Launch-Datei im
eigenen Paket); hier wird nur entschieden, WELCHE Schichten laufen und mit welchen Schaltern.

    ros2 launch roboter_system system.launch.py                                   # Roboter allein (Layer 3+4)
    ros2 launch roboter_system system.launch.py mission:=testwelt navigation:=nav2 headless:=true
    ros2 launch roboter_system system.launch.py mission:=beispiel navigation:=testdouble auto_start:=true
    ros2 launch roboter_system system.launch.py mission:=testwelt navigation:=extern     # Nav2 laeuft separat

Schichten:
  Layer 4  Simulation     roboter_simulation/launch/simulation.launch.py   (immer)
  Layer 3  Steuerung      roboter_steuerung/launch/steuerung.launch.py     (immer)
  Sensorik                roboter_sensorik/launch/sensorik.launch.py       (bei lidar_modus 3d)
  Layer 2  Navigation     roboter_navigation/launch/navigation.launch.py   (navigation:=nav2)
                          extern gestartetes Nav2, z. B. Variante des Kollegen (navigation:=extern)
                          Testdouble aus roboter_task_planning             (navigation:=testdouble, nur Tests)
  Layer 1  Mission        roboter_task_planning/launch/mission.launch.py   (mission:=<name oder Pfad>)
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTE = [
    ("mission", "", "Mission: Name aus roboter_task_planning/missions (z. B. testwelt) oder Pfad; leer = keine"),
    ("welt", "", "Umgebung aus roboter_welten; leer = Welt der Mission (ohne Mission: nur Ebene)"),
    ("navigation", "aus", "aus | nav2 (roboter_navigation) | extern (Nav2 laeuft separat, z. B. Variante des Kollegen; "
                          "Lidar, Sensorik und Watchdog wie bei nav2) | testdouble (nur Tests: geradeaus, ohne Karte)"),
    ("nav2_params", "", "eigene Nav2-Parameterdatei (leer = Standard aus roboter_navigation passend zum Lidar)"),
    ("slam", "false", "slam_toolbox liefert /map und map->odom (nur mit navigation:=nav2)"),
    ("lidar_modus", "", "aus | 3d (ein 3D-Lidar, /scan abgeleitet) | 2d (eine Scanebene, Vergleich); "
                        "leer = 3d, sobald eine Navigation laeuft (nav2, extern, testdouble), sonst aus"),
    ("odom_quelle", "ground_truth", "ground_truth (Simulation) | beinodometrie (Schaetzer aus Gelenken + IMU)"),
    ("realistic_sensors", "false", "Sensorrauschen, Bias, Drift, Latenz, Lidar-Ausfaelle"),
    ("headless", "false", "Simulation ohne Viewer-Fenster (weniger CPU/Speicher)"),
    ("record_path", "", ".npz-Datei fuer die Posenaufzeichnung (Demo-Video)"),
    ("cmd_vel_timeout_s", "", "cmd_vel-Watchdog [s]; leer = 1.0, sobald eine Navigation laeuft (auch extern; "
                              "dauernder Befehlsstrom), sonst 0 (aus, Handbetrieb mit einzelnen Befehlen)"),
    ("auto_start", "false", "Mission sofort starten (sonst Dienst /mission/start)"),
    ("rviz", "false", "RViz mit der Ansicht rviz/ansicht_3d.rviz"),
]


def _include(paket, datei, argumente):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(paket), "launch", datei)),
        launch_arguments=list(argumente.items()))


def _mission_pfad(mission):
    if not mission or os.path.sep in mission or mission.endswith(".yaml"):
        return mission
    return os.path.join(get_package_share_directory("roboter_task_planning"), "missions", mission + ".yaml")


def _aufbauen(context):
    a = {name: LaunchConfiguration(name).perform(context) for name, _, _ in ARGUMENTE}
    navigation = a["navigation"].lower()
    if navigation not in ("aus", "nav2", "extern", "testdouble"):
        raise ValueError(f"navigation:={navigation!r} - erlaubt: aus, nav2, extern, testdouble")
    mission = _mission_pfad(a["mission"])
    if mission and navigation == "aus":
        # Eine Mission faehrt Orte an und braucht deshalb eine Navigation; frueher startete sie trotzdem und
        # scheiterte erst beim ersten Fahrschritt.
        raise ValueError("mission:= braucht navigation:=nav2, extern oder testdouble")
    # Eine Quelle fuer die Umgebung: ohne Argument welt laeuft die Simulation in der Welt der Mission, damit
    # Simulation und Orte nicht auseinanderlaufen koennen.
    welt = a["welt"]
    if not welt and mission:
        with open(mission) as f:
            welt = (yaml.safe_load(f) or {}).get("welt", "")
    lidar = a["lidar_modus"] or ("3d" if navigation != "aus" else "aus")
    timeout = a["cmd_vel_timeout_s"] or ("1.0" if navigation != "aus" else "0.0")

    aktionen = [
        _include("roboter_simulation", "simulation.launch.py", {
            "welt": welt, "headless": a["headless"], "lidar_modus": lidar,
            "realistic_sensors": a["realistic_sensors"], "record_path": a["record_path"],
            "odom_ground_truth": str(a["odom_quelle"] == "ground_truth").lower()}),
        _include("roboter_steuerung", "steuerung.launch.py", {
            "odom_quelle": a["odom_quelle"], "cmd_vel_timeout_s": timeout}),
    ]
    if lidar == "3d":
        aktionen.append(_include("roboter_sensorik", "sensorik.launch.py", {}))
    if navigation == "nav2":
        cfg = os.path.join(get_package_share_directory("roboter_navigation"), "config")
        params = a["nav2_params"] or os.path.join(cfg, "nav2_params_2d.yaml" if lidar == "2d" else "nav2_params.yaml")
        aktionen.append(_include("roboter_navigation", "navigation.launch.py",
                                 {"slam": a["slam"], "params_file": params}))
    elif navigation == "testdouble":
        aktionen.append(Node(package="roboter_task_planning", executable="nav_testdouble_node", output="screen"))
    if mission:
        aktionen.append(_include("roboter_task_planning", "mission.launch.py", {
            "mission_file": mission, "auto_start": a["auto_start"],
            "nav_backend": "gehaktion" if navigation == "testdouble" else "nav2"}))
    if a["rviz"].lower() == "true":
        rviz = os.path.join(get_package_share_directory("roboter_system"), "rviz", "ansicht_3d.rviz")
        aktionen.append(Node(package="rviz2", executable="rviz2", arguments=["-d", rviz], output="log"))
    return aktionen


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in ARGUMENTE]
                             + [OpaqueFunction(function=_aufbauen)])
