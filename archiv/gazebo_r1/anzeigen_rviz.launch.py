#!/usr/bin/env python3
"""
anzeigen_rviz.launch.py

Erster Test der Unitree-R1-Spezialisierung: laedt das offizielle
Unitree-R1-URDF (Massen, Traegheiten, echte Geometrie -- Dinge, die das
generische SysML-Modell bewusst NICHT enthaelt, siehe Projektstand) und
zeigt es in RViz an. Bewusst VOR Gazebo, weil es weniger bewegliche Teile
gibt, die schiefgehen koennen -- wenn das URDF/die Meshes hier nicht
sauber laden, wuerde es in Gazebo genauso scheitern, nur schwerer zu
debuggen.

Voraussetzung: der Ordner r1_description aus
https://github.com/unitreerobotics/unitree_ros (Unterordner
robots/r1_description) muss lokal vorhanden sein -- NICHT Teil dieses
Pakets (Lizenz-/Groessengruende, ~22MB Meshes). Siehe README fuer den
git-clone-Befehl. Erwarteter Pfad standardmaessig:

    ~/Ros2/unitree_ros/robots/r1_description/R1.urdf

Falls du es woanders abgelegt hast, R1_URDF_PFAD unten anpassen.

Aufruf (nach colcon build + source install/setup.bash):

    ros2 launch r1_bringup anzeigen_rviz.launch.py

In RViz danach von Hand (kein mitgeliefertes .rviz-Profil, um
Versions-/Formatprobleme zu vermeiden):
  1. "Add" -> "By display type" -> "RobotModel" -> OK
  2. Links oben unter "Global Options" -> "Fixed Frame" auf
     "pelvis_link" setzen
  3. WICHTIG, sonst bleibt der Roboter unsichtbar: bei "RobotModel" ->
     "Description Topic" die "Durability Policy" von "Volatile"
     (RViz2-Standard) auf "Transient Local" umstellen, dann den Haken
     bei "RobotModel" einmal aus- und wieder anklicken. Grund:
     robot_state_publisher veroeffentlicht robot_description nur EINMAL
     beim Start (QoS "Transient Local"/gelatcht); ein spaeter
     verbindender "Volatile"-Abonnent verpasst diese eine Nachricht fuer
     immer, obwohl RViz weiterhin "Status: Ok" anzeigt statt eines
     Fehlers. Diagnose: `ros2 topic info /robot_description -v` zeigt,
     ob rviz2 ueberhaupt als Subscription auftaucht.
Mit dem sich oeffnenden "Joint State Publisher"-Fenster (Schieberegler)
lassen sich einzelne Gelenke bewegen, um die Kinematik zu pruefen.
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node

# Pfad zum geklonten r1_description-Ordner von Unitree -- siehe README.
R1_URDF_PFAD = os.path.expanduser(
    "~/Ros2/unitree_ros/robots/r1_description/R1.urdf"
)


def _urdf_lesen_und_meshpfade_fixen(urdf_pfad: str) -> str:
    """Die Roh-URDF von Unitree referenziert Meshes mit relativem Pfad
    ("meshes/xxx.STL"); ohne package://-URI findet RViz/Gazebo das nicht
    zuverlaessig. Hier durch einen absoluten file://-Pfad ersetzen."""
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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    return LaunchDescription([robot_state_publisher, joint_state_publisher_gui, rviz])
