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
    nav_testdouble:=false -> Navigation kommt von Nav2 (Layer 2, Kollege), nav_backend=nav2.
    """
    share = get_package_share_directory("roboter_task_planning")
    steuer = get_package_share_directory("roboter_steuerung")
    args = [
        DeclareLaunchArgument("config_file", default_value="g1_policy_29dof.yaml"),
        DeclareLaunchArgument("mission_file", default_value=os.path.join(share, "missions", "beispiel.yaml")),
        DeclareLaunchArgument("nav_testdouble", default_value="false"),
        DeclareLaunchArgument("auto_start", default_value="false"),
    ]
    layer3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(steuer, "launch", "steuereinheit.launch.py")),
        launch_arguments={"config_file": LaunchConfiguration("config_file")}.items())
    balance = Node(package="roboter_steuerung", executable="balance_dienst_node", output="screen")
    double = Node(package="roboter_task_planning", executable="nav_testdouble_node", output="screen",
                  condition=IfCondition(LaunchConfiguration("nav_testdouble")))
    mission = Node(package="roboter_task_planning", executable="mission_node", output="screen",
                   parameters=[{
                       "mission_file": LaunchConfiguration("mission_file"),
                       "nav_backend": PythonExpression(
                           ["'gehaktion' if '", LaunchConfiguration("nav_testdouble"), "' == 'true' else 'nav2'"]),
                       "auto_start": LaunchConfiguration("auto_start"),
                   }])
    return LaunchDescription(args + [layer3, balance, double, mission])
