import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Layer 2: Nav2 (nur Navigation, ohne Lokalisierung/SLAM) fuer die Simulation.

        ros2 launch roboter_navigation navigation.launch.py [params_file:=<datei>]

    - map->odom: statische Identitaet (Standard) oder slam:=true -> slam_toolbox (/map + map->odom)
    - Ausgang: /cmd_vel (velocity_smoother) -> Layer 3
    - Die Variante des Kollegen: params_file:=<seine.yaml> (oder dieses Paket ersetzen)
    """
    share = get_package_share_directory("roboter_navigation")
    nav2 = get_package_share_directory("nav2_bringup")
    params = DeclareLaunchArgument(
        "params_file", default_value=os.path.join(share, "config", "nav2_params.yaml"))
    composition = DeclareLaunchArgument("use_composition", default_value="True")
    slam = DeclareLaunchArgument(
        "slam", default_value="false",
        description="true: slam_toolbox liefert /map und map->odom (statt statischer Identitaet)")
    slam_params = DeclareLaunchArgument(
        "slam_params_file", default_value=os.path.join(share, "config", "slam_params.yaml"))

    static_tf = Node(package="tf2_ros", executable="static_transform_publisher", name="map_to_odom",
                     arguments=["--frame-id", "map", "--child-frame-id", "odom"],
                     condition=UnlessCondition(LaunchConfiguration("slam")))
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")),
        launch_arguments={"slam_params_file": LaunchConfiguration("slam_params_file"),
                          "use_sim_time": "false", "autostart": "true"}.items(),
        condition=IfCondition(LaunchConfiguration("slam")))
    # navigation_launch.py laedt bei use_composition=True die Knoten nur in einen BEREITS
    # laufenden Container (den startet sonst bringup_launch.py) - also hier selbst starten.
    container = Node(
        condition=IfCondition(LaunchConfiguration("use_composition")),
        package="rclcpp_components", executable="component_container_isolated",
        name="nav2_container", output="screen",
        parameters=[LaunchConfiguration("params_file"), {"autostart": True}],
        arguments=["--ros-args", "--log-level", "info"])
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2, "launch", "navigation_launch.py")),
        launch_arguments={
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": "false",
            "autostart": "true",
            "use_composition": LaunchConfiguration("use_composition"),
        }.items())
    return LaunchDescription([params, composition, slam, slam_params, static_tf, slam_launch, container, navigation])
