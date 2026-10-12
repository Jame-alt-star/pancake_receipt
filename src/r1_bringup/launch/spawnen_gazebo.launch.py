#!/usr/bin/env python3
"""
spawnen_gazebo.launch.py

Zweiter Schritt der Simulation: dasselbe (mesh-pfad-korrigierte) R1-URDF,
das schon in RViz sichtbar ist, wird jetzt zusaetzlich in eine echte
Gazebo-Physiksimulation gespawnt (statt nur angezeigt).

Startet:
  1. Eine leere Gazebo-Welt (ueber ros_gz_sim)
  2. robot_state_publisher (gleiche Mesh-Pfad-Fix-Logik wie im
     RViz-Launch anzeigen_rviz.launch.py)
  3. Den "create"-Spawner von ros_gz_sim, der den Roboter anhand des
     robot_description-Topics in die laufende Gazebo-Welt einfuegt
  4. Eine ros_gz_bridge fuer /clock, damit ROS2-Nodes (spaeter z.B.
     ros2_control) sich an der Simulationszeit statt der Wanduhr
     orientieren koennen

Voraussetzung: dieselbe wie bei anzeigen_rviz.launch.py -- das geklonte
r1_description von Unitree muss unter
~/Ros2/unitree_ros/robots/r1_description/R1.urdf liegen, und
ros-jazzy-ros-gz muss installiert sein (siehe README, Schritt 1).

Aufruf (nach colcon build + source install/setup.bash):

    ros2 launch r1_bringup spawnen_gazebo.launch.py

Es oeffnet sich ein Gazebo-Fenster mit einer leeren, flachen Welt. Der
Roboter faellt aus 1m Hoehe auf den Boden -- das ist Absicht (kleiner
Puffer, damit er nicht direkt im Boden "steckt"). Da es fuer die
Gelenke noch keine aktive Steuerung/Balance-Regelung gibt (der
Mock-Node steuereinheit_node kennt noch keine echten Gelenkbefehle),
wird der Roboter je nach Ausgangspose vermutlich einfach umfallen oder
in sich zusammensacken -- das ist der erwartete Zustand bis zur
ros2_control-Anbindung (siehe README "Naechster Schritt"), nicht ein
Fehler dieses Launch-Files.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Pfad zum geklonten r1_description-Ordner von Unitree -- siehe README.
R1_URDF_PFAD = os.path.expanduser(
    "~/Ros2/unitree_ros/robots/r1_description/R1.urdf"
)


def _urdf_lesen_und_meshpfade_fixen(urdf_pfad: str) -> str:
    """Gleiche Logik wie in anzeigen_rviz.launch.py: relative Mesh-Pfade
    ("meshes/xxx.STL") durch absolute file://-URIs ersetzen, sonst
    findet Gazebo die Meshes nicht."""
    with open(urdf_pfad, "r") as f:
        text = f.read()
    mesh_ordner = os.path.join(os.path.dirname(urdf_pfad), "meshes")
    return text.replace('filename="meshes/', f'filename="file://{mesh_ordner}/')


def generate_launch_description():
    if not os.path.exists(R1_URDF_PFAD):
        raise FileNotFoundError(
            f"R1.urdf nicht gefunden unter {R1_URDF_PFAD}. Siehe README: "
            "unitree_ros muss vorher geklont werden (git clone, "
            "sparse-checkout robots/r1_description)."
        )

    robot_description = _urdf_lesen_und_meshpfade_fixen(R1_URDF_PFAD)

    # Eigene Welt statt "empty.sdf": inhaltlich identisch, aber mit
    # Kamera-Startposition nah am Roboter-Spawnpunkt (siehe
    # worlds/r1_welt.sdf). "-r" laesst die Simulation sofort laufen
    # (nicht pausiert starten).
    welt_pfad = os.path.join(
        get_package_share_directory("r1_bringup"), "worlds", "r1_welt.sdf"
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": f"-r {welt_pfad}"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Roboter anhand des robot_description-Topics in die laufende
    # Gazebo-Welt spawnen, 1m ueber dem Boden.
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="r1_spawner",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "unitree_r1", "-z", "1.0"],
    )

    # Bruecke fuer die Simulationszeit (/clock aus Gazebo nach ROS2) --
    # wichtig fuer spaetere ros2_control-Anbindung.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
    )

    # Kleine Verzoegerung, damit Gazebo (Welt + Spawn-Service) sicher
    # bereit ist, bevor gespawnt wird -- besonders beim allerersten Start
    # (Ressourcen laden) kann das ein paar Sekunden dauern.
    spawn_verzoegert = TimerAction(period=5.0, actions=[spawn])
    bridge_verzoegert = TimerAction(period=5.0, actions=[clock_bridge])

    return LaunchDescription(
        [gazebo, robot_state_publisher, spawn_verzoegert, bridge_verzoegert]
    )
