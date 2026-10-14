"""Layer 1: Missionsknoten (Planung mit Fast Downward, Ausfuehrung ueber ROS-2-Actions).
Startet nur Layer 1; Navigation, Steuerung und Simulation setzt roboter_system/launch/system.launch.py dazu.

    ros2 launch roboter_task_planning mission.launch.py mission_file:=<pfad> [nav_backend:=gehaktion]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("roboter_task_planning")
    args = [
        DeclareLaunchArgument("mission_file", default_value=os.path.join(share, "missions", "beispiel.yaml")),
        DeclareLaunchArgument("nav_backend", default_value="nav2",
                              description="nav2 (NavigateToPose) | gehaktion (GehAktion, z. B. Testdouble)"),
        DeclareLaunchArgument("auto_start", default_value="false"),
    ]
    mission = Node(package="roboter_task_planning", executable="mission_node", output="screen",
                   parameters=[{"mission_file": LaunchConfiguration("mission_file"),
                                "nav_backend": LaunchConfiguration("nav_backend"),
                                "auto_start": LaunchConfiguration("auto_start")}])
    return LaunchDescription(args + [mission])
