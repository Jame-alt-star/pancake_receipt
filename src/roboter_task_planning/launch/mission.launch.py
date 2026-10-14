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
        DeclareLaunchArgument("record_path", default_value="",
                              description=".npz-Datei fuer die Posenaufzeichnung (Demo-Video)"),
        DeclareLaunchArgument("odom_quelle", default_value="ground_truth",
                              description="ground_truth oder beinodometrie"),
        DeclareLaunchArgument("realistic_sensors", default_value="false",
                              description="Sensorrauschen, Latenz, Lidar-Ausfaelle in der Simulation"),
        DeclareLaunchArgument("lidar_modus", default_value="3d",
                              description="3d: ein 3D-Lidar (/points), /scan daraus abgeleitet (Standard); "
                                          "2d: nur eine Scanebene (/scan, alter Stand, zum Vergleich)"),
        DeclareLaunchArgument("nav2_params", default_value="",
                              description="eigene Nav2-Parameterdatei (leer = Standard aus roboter_navigation)"),
        DeclareLaunchArgument("lidar", default_value="true",
                              description="simuliertes Lidar (/scan) in der Simulation"),
        DeclareLaunchArgument("slam", default_value="false",
                              description="slam_toolbox statt statischem map->odom (mit nav2:=true)"),
        DeclareLaunchArgument("cmd_vel_timeout_s", default_value="1.0",
                              description="cmd_vel-Watchdog in Layer 3 [s] (0 = aus)"),
        DeclareLaunchArgument("welt", default_value="",
                              description="Umgebung der Simulation; leer = Welt aus der Missionsdatei (Schluessel welt)"),
        DeclareLaunchArgument("nav2", default_value="false",
                              description="Nav2 (roboter_navigation, Layer 2) mitstarten"),
        DeclareLaunchArgument("auto_start", default_value="false"),
    ]
    # Eine Quelle fuer die Umgebung: ohne Argument welt startet die Simulation in der Welt, auf die die
    # Missionsdatei verweist (sonst koennten Simulation und Orte auseinanderlaufen).
    welt_aus_mission = PythonExpression([
        "(__import__('yaml').safe_load(open('", LaunchConfiguration("mission_file"), "')) or {}).get('welt') or ''"])
    ist3d = PythonExpression(["'", LaunchConfiguration("lidar_modus"), "' == '3d'"])
    nav_cfg = os.path.join(get_package_share_directory("roboter_navigation"), "config")
    layer3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(steuer, "launch", "steuereinheit.launch.py")),
        launch_arguments={"config_file": LaunchConfiguration("config_file"),
                          "headless": LaunchConfiguration("headless"),
                          "welt": PythonExpression(["'", LaunchConfiguration("welt"), "' or '",
                                                    welt_aus_mission, "'"]),
                          "lidar": LaunchConfiguration("lidar"),
                          "lidar_3d": ist3d,
                          "lidar_2d": PythonExpression(["'", LaunchConfiguration("lidar_modus"), "' == '2d'"]),
                          "realistic_sensors": LaunchConfiguration("realistic_sensors"),
                          "odom_quelle": LaunchConfiguration("odom_quelle"),
                          "record_path": LaunchConfiguration("record_path"),
                          "cmd_vel_timeout_s": LaunchConfiguration("cmd_vel_timeout_s")}.items())
    balance = Node(package="roboter_steuerung", executable="balance_dienst_node", output="screen")
    double = Node(package="roboter_task_planning", executable="nav_testdouble_node", output="screen",
                  condition=IfCondition(LaunchConfiguration("nav_testdouble")))
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("roboter_navigation"), "launch", "navigation.launch.py")),
        launch_arguments={"slam": LaunchConfiguration("slam"),
                          # Standard je nach Lidar-Modus: nav2_params.yaml (3D) bzw. nav2_params_2d.yaml
                          "params_file": PythonExpression([
                              "'", LaunchConfiguration("nav2_params"), "' if '", LaunchConfiguration("nav2_params"),
                              "' else ('", os.path.join(nav_cfg, "nav2_params.yaml"), "' if '",
                              LaunchConfiguration("lidar_modus"), "' == '3d' else '",
                              os.path.join(nav_cfg, "nav2_params_2d.yaml"), "')"])}.items(),
        condition=IfCondition(LaunchConfiguration("nav2")))
    scan_ableitung = Node(package="roboter_navigation", executable="punktwolke_zu_scan", output="screen",
                          condition=IfCondition(PythonExpression(["'", LaunchConfiguration("lidar_modus"),
                                                                  "' == '3d' and '", LaunchConfiguration("lidar"),
                                                                  "' == 'true'"])))
    mission = Node(package="roboter_task_planning", executable="mission_node", output="screen",
                   parameters=[{
                       "mission_file": LaunchConfiguration("mission_file"),
                       "nav_backend": PythonExpression(
                           ["'gehaktion' if '", LaunchConfiguration("nav_testdouble"), "' == 'true' else 'nav2'"]),
                       "auto_start": LaunchConfiguration("auto_start"),
                   }])
    return LaunchDescription(args + [layer3, balance, double, scan_ableitung, navigation, mission])
