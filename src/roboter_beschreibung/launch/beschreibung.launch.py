"""Roboterbeschreibung im TF-Baum: robot_state_publisher mit der Unitree-URDF (alle Glieder aus /joint_states),
base_link -> pelvis und die Sensor-Montage (Lidar) aus config/roboter.yaml. Gilt fuer Simulation und Hardware.

    ros2 launch roboter_beschreibung beschreibung.launch.py

TF-Baum: odom -> base_link (Layer 4 oder Beinodometrie) -> pelvis -> ... -> torso_link -> ...; lidar_link haengt am
in roboter.yaml genannten Glied (Standard pelvis; am echten G1 sitzt der MID-360 im Kopf, also am torso_link).
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("roboter_beschreibung")
    with open(os.path.join(share, "config", "roboter.yaml")) as f:
        roboter = yaml.safe_load(f)
    g1 = os.path.join(share, "modelle", "g1_description")
    with open(os.path.join(g1, roboter["urdf"])) as f:
        # Unitree referenziert Meshes relativ ("meshes/x.STL"); RViz findet sie nur ueber package://
        urdf = f.read().replace('filename="meshes/', 'filename="package://roboter_beschreibung/modelle/g1_description/meshes/')
    lidar = roboter["lidar"]
    x, y, z = (str(v) for v in lidar["xyz"])
    r, p, yw = (str(v) for v in lidar["rpy"])
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="log",
             parameters=[{"robot_description": urdf}]),
        # base_link (Nav2-Konvention) = pelvis (Wurzel der Unitree-URDF)
        Node(package="tf2_ros", executable="static_transform_publisher", name="base_link_zu_pelvis", output="log",
             arguments=["--frame-id", "base_link", "--child-frame-id", "pelvis"]),
        Node(package="tf2_ros", executable="static_transform_publisher", name="lidar_montage", output="log",
             arguments=["--x", x, "--y", y, "--z", z, "--roll", r, "--pitch", p, "--yaw", yw,
                        "--frame-id", lidar["parent"], "--child-frame-id", "lidar_link"]),
    ])
