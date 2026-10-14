"""Layer 3: Steuereinheit (Zustandsautomat + Policy), Balance-Dienste und optional die Beinodometrie.
Startet nur die Steuerung; die Simulation (oder spaeter die Hardware-Anbindung) kommt aus Layer 4.

    ros2 launch roboter_steuerung steuerung.launch.py [odom_quelle:=beinodometrie] [cmd_vel_timeout_s:=1.0]

Die Knoten warten selbst auf die ersten Sensordaten (Start-Gate in steuereinheit_node), eine
Startreihenfolge zu Layer 4 ist nicht noetig.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

ARGUMENTE = [
    ("odom_quelle", "ground_truth", "ground_truth (Layer 4 liefert /odom) | beinodometrie (Schaetzer hier)"),
    ("cmd_vel_timeout_s", "0.0", "cmd_vel-Watchdog [s]: ohne neue Nachricht wird das Kommando 0 (0 = aus)"),
]


def generate_launch_description():
    steuereinheit = Node(
        package="roboter_steuerung", executable="steuereinheit_node", name="steuereinheit_node", output="screen",
        parameters=[{"cmd_vel_timeout_s": LaunchConfiguration("cmd_vel_timeout_s")}])
    balance = Node(package="roboter_steuerung", executable="balance_dienst_node", output="screen")
    beinodometrie = Node(
        package="roboter_steuerung", executable="beinodometrie_node", name="beinodometrie_node", output="screen",
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("odom_quelle"), "' == 'beinodometrie'"])))
    return LaunchDescription([DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in ARGUMENTE]
                             + [steuereinheit, balance, beinodometrie])
