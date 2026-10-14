"""Sensorvorverarbeitung: leitet aus der 3D-Punktwolke (/points) den 2D-Scan (/scan) und die
Hinderniswolke (/punkte_hindernis) ab. Unabhaengig von der Navigation, damit eine andere Nav2-Variante
dieselben Eingaenge bekommt.

    ros2 launch roboter_sensorik sensorik.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="roboter_sensorik", executable="punktwolke_zu_scan", output="screen"),
    ])
