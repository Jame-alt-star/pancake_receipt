"""Layer 3: Steuereinheit (Zustandsautomat + Policy), Balance-Dienste und optional die Beinodometrie.
Startet nur die Steuerung; die Simulation (oder spaeter die Hardware-Anbindung) kommt aus Layer 4.

    ros2 launch roboter_steuerung steuerung.launch.py [odom_quelle:=beinodometrie] [cmd_vel_timeout_s:=1.0]

Parameter aller Knoten: config/steuerung.yaml (wird immer geladen); die Policy: config/policy_29dof.yaml.
Die Knoten warten selbst auf die ersten Sensordaten (Start-Gate in steuereinheit_node), eine
Startreihenfolge zu Layer 4 ist nicht noetig.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTE = [
    ("odom_quelle", "ground_truth", "ground_truth (Layer 4 liefert /odom) | beinodometrie (Schaetzer hier)"),
    ("cmd_vel_timeout_s", "", "cmd_vel-Watchdog [s] (0 = aus); leer = Wert aus config/steuerung.yaml"),
]


def _knoten(context):
    params = os.path.join(get_package_share_directory("roboter_steuerung"), "config", "steuerung.yaml")
    timeout = LaunchConfiguration("cmd_vel_timeout_s").perform(context)
    ueberschrieben = [{"cmd_vel_timeout_s": float(timeout)}] if timeout else []
    knoten = [
        Node(package="roboter_steuerung", executable="steuereinheit_node", name="steuereinheit_node",
             output="screen", parameters=[params] + ueberschrieben),
        Node(package="roboter_steuerung", executable="balance_dienst_node", name="balance_dienst_node",
             output="screen", parameters=[params]),
    ]
    if LaunchConfiguration("odom_quelle").perform(context) == "beinodometrie":
        knoten.append(Node(package="roboter_steuerung", executable="beinodometrie_node", name="beinodometrie_node",
                           output="screen", parameters=[params]))
    return knoten


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in ARGUMENTE]
                             + [OpaqueFunction(function=_knoten)])
