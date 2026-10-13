import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Layer 3/4 (Simulation + Steuereinheit) + Balance-Dienste + Missions-Knoten.

        ros2 launch roboter_task_planning mission.launch.py \
            config_file:=g1_policy_29dof.yaml nav_testdouble:=true auto_start:=true
    nav2:=true            -> Nav2 (roboter_navigation) wird mitgestartet, nav_backend=nav2
    nav_testdouble:=false -> Navigation kommt von aussen (Nav2 des Kollegen), nav_backend=nav2.
    """
    share = get_package_share_directory("roboter_task_planning")
    steuer = get_package_share_directory("roboter_steuerung")
    args = [
        DeclareLaunchArgument("config_file", default_value="g1_policy_29dof.yaml"),
        DeclareLaunchArgument("mission_file", default_value=os.path.join(share, "missions", "beispiel.yaml")),
        DeclareLaunchArgument("nav_testdouble", default_value="false"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="MuJoCo ohne Viewer-Fenster (weniger CPU/Speicher)"),
        DeclareLaunchArgument("scene_file", default_value="",
                              description="MuJoCo-Szene (relativ zu r1_bringup), z.B. Testwelt"),
        DeclareLaunchArgument("nav2", default_value="false",
                              description="Nav2 (roboter_navigation, Layer 2) mitstarten"),
        DeclareLaunchArgument("auto_start", default_value="false"),
    ]
    layer3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(steuer, "launch", "steuereinheit.launch.py")),
        launch_arguments={"config_file": LaunchConfiguration("config_file"),
                          "headless": LaunchConfiguration("headless"),
                          "scene_file": LaunchConfiguration("scene_file")}.items())
    balance = Node(package="roboter_steuerung", executable="balance_dienst_node", output="screen")
    double = Node(package="roboter_task_planning", executable="nav_testdouble_node", output="screen",
                  condition=IfCondition(LaunchConfiguration("nav_testdouble")))
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("roboter_navigation"), "launch", "navigation.launch.py")),
        condition=IfCondition(LaunchConfiguration("nav2")))
    mission = Node(package="roboter_task_planning", executable="mission_node", output="screen",
                   parameters=[{
                       "mission_file": LaunchConfiguration("mission_file"),
                       "nav_backend": PythonExpression(
                           ["'gehaktion' if '", LaunchConfiguration("nav_testdouble"), "' == 'true' else 'nav2'"]),
                       "auto_start": LaunchConfiguration("auto_start"),
                   }])
    return LaunchDescription(args + [layer3, balance, double, navigation, mission])
