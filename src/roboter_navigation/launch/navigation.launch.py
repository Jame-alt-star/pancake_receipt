import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTE = [
    ("params_file", "", "Nav2-Parameter (leer = config/nav2_params.yaml dieses Pakets)"),
    ("overlays", "", "Overlay-Dateien, durch Komma getrennt; jede ueberschreibt nur ihre Eintraege (z. B. "
                     "config/nav2_params_2d.yaml fuer lidar_modus:=2d)"),
    ("karte", "", "Kartendatei (Nav2-YAML) -> map_server + AMCL + statische Kartenschicht; leer = ohne Karte"),
    ("slam", "false", "true: slam_toolbox liefert /map und map->odom (nicht zusammen mit karte)"),
    ("slam_params_file", "", "slam_toolbox-Parameter (leer = config/slam_params.yaml)"),
    ("startpose", "0,0,0", "Startpose x,y,gier [m, m, rad] im map-Frame fuer AMCL (nur mit karte); Simulation: Ursprung"),
    ("use_composition", "True", "Nav2-Knoten in einem gemeinsamen Container"),
]


def mischen(basis: dict, overlay: dict) -> dict:
    """Overlay rekursiv ueber die Basis legen (Dictionaries werden zusammengefuehrt, alles andere ersetzt)."""
    erg = dict(basis)
    for k, v in overlay.items():
        erg[k] = mischen(erg[k], v) if isinstance(v, dict) and isinstance(erg.get(k), dict) else v
    return erg


def parameter_datei(basis_pfad: str, overlays, karte: str, slam: bool = False, startpose=(0.0, 0.0, 0.0)) -> str:
    """Basis + Overlays (+ Karten- bzw. SLAM-Modus) zu einer Datei zusammenfuehren; Pfad einer temporaeren Datei."""
    with open(basis_pfad) as f:
        p = yaml.safe_load(f)
    for o in overlays:
        with open(o) as f:
            p = mischen(p, yaml.safe_load(f))
    gc = p["global_costmap"]["global_costmap"]["ros__parameters"]
    if karte or slam:
        # Planung auf der Karte: feste Karte (map_server) oder die waehrend der Fahrt entstehende SLAM-Karte. Frueher
        # nutzte der SLAM-Modus seine Karte nur zur Lokalisierung, geplant wurde im Fenster der aktuellen Lidardaten.
        if "static_layer" not in gc["plugins"]:
            gc["plugins"] = ["static_layer"] + list(gc["plugins"])
        gc.setdefault("static_layer", {}).update({"plugin": "nav2_costmap_2d::StaticLayer",
                                                  "map_subscribe_transient_local": True,
                                                  "subscribe_to_updates": bool(slam)})
    if karte:
        p.setdefault("map_server", {}).setdefault("ros__parameters", {})["yaml_filename"] = karte
        x, y, gier = startpose
        p.setdefault("amcl", {}).setdefault("ros__parameters", {})["initial_pose"] = \
            {"x": float(x), "y": float(y), "z": 0.0, "yaw": float(gier)}
    f = tempfile.NamedTemporaryFile("w", prefix="nav2_", suffix=".yaml", delete=False)
    yaml.safe_dump(p, f, sort_keys=False)
    f.close()
    return f.name


def _aufbauen(context):
    share = get_package_share_directory("roboter_navigation")
    a = {n: LaunchConfiguration(n).perform(context) for n, _, _ in ARGUMENTE}
    slam, karte = a["slam"].lower() in ("true", "1"), a["karte"]
    if slam and karte:
        raise ValueError("karte:= und slam:=true schliessen sich aus (Lokalisierung auf fester Karte ODER Kartierung)")
    overlays = [o for o in a["overlays"].split(",") if o]
    if karte:
        overlays.append(os.path.join(share, "config", "nav2_karte.yaml"))
    startpose = [float(v) for v in a["startpose"].split(",")]
    if len(startpose) != 3:
        raise ValueError(f"startpose:={a['startpose']!r} - erwartet x,y,gier")
    params = parameter_datei(a["params_file"] or os.path.join(share, "config", "nav2_params.yaml"), overlays, karte,
                             slam, startpose)
    aktionen = []
    if karte:
        # Lokalisierung auf der festen Karte: map_server + AMCL (AMCL liefert map->odom)
        aktionen += [
            Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
                 parameters=[params]),
            Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen", parameters=[params]),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
                 output="screen", parameters=[{"autostart": True, "node_names": ["map_server", "amcl"]}]),
        ]
    elif slam:
        # slam_toolbox nur aufloesen, wenn gebraucht (Review B7: sonst brach nav2:=true ohne slam_toolbox ab)
        aktionen.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")),
            launch_arguments={"slam_params_file": a["slam_params_file"] or os.path.join(share, "config",
                                                                                        "slam_params.yaml"),
                              "use_sim_time": "false", "autostart": "true"}.items()))
    else:
        # ohne Karte: map = odom (Startpose = Kartenursprung)
        aktionen.append(Node(package="tf2_ros", executable="static_transform_publisher", name="map_to_odom",
                             arguments=["--frame-id", "map", "--child-frame-id", "odom"]))
    # navigation_launch.py laedt bei use_composition=True die Knoten nur in einen BEREITS laufenden Container
    # (den startet sonst bringup_launch.py) - also hier selbst starten.
    if a["use_composition"].lower() in ("true", "1"):
        aktionen.append(Node(package="rclcpp_components", executable="component_container_isolated",
                             name="nav2_container", output="screen", parameters=[params, {"autostart": True}],
                             arguments=["--ros-args", "--log-level", "info"]))
    aktionen.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("nav2_bringup"), "launch",
                                                   "navigation_launch.py")),
        launch_arguments={"params_file": params, "use_sim_time": "false", "autostart": "true",
                          "use_composition": a["use_composition"]}.items()))
    return aktionen


def generate_launch_description():
    """Layer 2: Nav2 fuer die Simulation (Platzhalter; die Variante des Kollegen ersetzt ihn).

        ros2 launch roboter_navigation navigation.launch.py [params_file:=<datei>] [karte:=<karte.yaml> | slam:=true]

    - map->odom: statisch (Standard), slam:=true -> slam_toolbox, karte:=... -> AMCL auf der festen Karte
    - Ausgang: /cmd_vel (velocity_smoother -> collision_monitor) -> Layer 3
    """
    return LaunchDescription([DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in ARGUMENTE]
                             + [OpaqueFunction(function=_aufbauen)])
