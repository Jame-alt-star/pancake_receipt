from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """
    Startet die komplette G1/MuJoCo-Pipeline (Layer 3 + Layer 4) mit einem
    einzigen Befehl:

        ros2 launch roboter_steuerung steuereinheit.launch.py

    statt bisher zwei separate Terminals mit je einem "ros2 run" zu
    brauchen (einmal fuer mujoco_node, einmal fuer steuereinheit_node).

    Beide Knoten starten hier praktisch gleichzeitig - das ist unproblematisch,
    weil steuereinheit_node.py in _control_loop() selbst erkennt, wenn noch
    keine echten Sensordaten von mujoco_node.py angekommen sind (das
    Start-Gate, siehe Kommentar dort), und in der Zwischenzeit einfach
    nichts tut statt mit bedeutungslosen Nullwerten zu rechnen. Eine
    explizite Start-Reihenfolge oder Verzoegerung zwischen den beiden
    Knoten ist deshalb nicht noetig.

    NEU (DOF-Migration): "config_file" Launch-Argument - waehlt, welche
    g1_policy*.yaml beide Knoten verwenden (siehe steuereinheit_node.py/
    mujoco_node.py, config_file-ROS2-Parameter). Default bleibt
    "g1_policy_29dof.yaml" (seit 14.10.2026; 12-DOF: config_file:=g1_policy.yaml, nicht mehr getestet,
    wenn das Argument nicht angegeben wird). Fuer die neue 29-DOF-
    Ganzkoerper-Policy:

        ros2 launch roboter_steuerung steuereinheit.launch.py config_file:=g1_policy_29dof.yaml

    WICHTIG: beide Knoten muessen DIESELBE Config verwenden (gleiche
    kps/kds/simulation_dt/control_decimation) - deshalb hier als EIN
    gemeinsames Launch-Argument statt zwei getrennter, die versehentlich
    auseinanderlaufen koennten.
    """
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="g1_policy_29dof.yaml",
        description="Policy-Config (im config/-Ordner von r1_bringup) - g1_policy.yaml (12-DOF, Default) oder g1_policy_29dof.yaml (29-DOF-Ganzkoerper).",
    )
    config_file = LaunchConfiguration("config_file")
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false",
        description="true = MuJoCo ohne Viewer-Fenster (weniger CPU/Speicher, fuer Tests/Nav2).")
    headless = LaunchConfiguration("headless")
    timeout_arg = DeclareLaunchArgument(
        "cmd_vel_timeout_s", default_value="0.0",
        description="cmd_vel-Watchdog [s]: ohne neue Nachricht wird das Kommando auf 0 gesetzt (0 = aus).")
    lidar_arg = DeclareLaunchArgument(
        "lidar", default_value="false",
        description="true = simuliertes Lidar in mujoco_node (Modus ueber lidar_2d/lidar_3d; fuer Nav2/SLAM).")
    realistic_arg = DeclareLaunchArgument(
        "realistic_sensors", default_value="false",
        description="true = Sensorrauschen, Gyro-Bias, Latenz und Lidar-Ausfaelle statt perfekter Simulationswerte.")
    odom_arg = DeclareLaunchArgument(
        "odom_quelle", default_value="ground_truth",
        description="ground_truth (perfekte Simulationswerte) oder beinodometrie (Schaetzer aus Gelenken + IMU)")
    odom_gt = PythonExpression(["'", LaunchConfiguration("odom_quelle"), "' == 'ground_truth'"])
    record_arg = DeclareLaunchArgument(
        "record_path", default_value="",
        description="Pfad einer .npz-Datei: Roboterpose (25 Hz) fuer Demo-Videos aufzeichnen.")
    lidar2d_arg = DeclareLaunchArgument(
        "lidar_2d", default_value="true",
        description="false = kein eigener 2D-Scan (nur 3D-Punktwolke, /scan wird daraus abgeleitet).")
    lidar3d_arg = DeclareLaunchArgument(
        "lidar_3d", default_value="false",
        description="true = 3D-Lidar als PointCloud2 (/points, 24 Ringe, 5 Hz).")
    welt_arg = DeclareLaunchArgument(
        "welt", default_value="",
        description="Umgebung aus roboter_welten (leer, testwelt, testwelt3d, demo_lager); leer = nur Ebene.")

    mujoco_node = Node(
        package="r1_bringup",
        executable="mujoco_node",
        name="mujoco_bridge_node",
        output="screen",
        parameters=[{"config_file": config_file, "headless": headless, "welt": LaunchConfiguration("welt"),
                     "lidar": LaunchConfiguration("lidar"),
                     "realistic_sensors": LaunchConfiguration("realistic_sensors"),
                     "record_path": LaunchConfiguration("record_path"),
                     "lidar_3d": LaunchConfiguration("lidar_3d"),
                     "lidar_2d": LaunchConfiguration("lidar_2d"),
                     "publish_odom": odom_gt, "publish_tf": odom_gt}],
    )

    steuereinheit_node = Node(
        package="roboter_steuerung",
        executable="steuereinheit_node",
        name="steuereinheit_node",
        output="screen",
        parameters=[{"config_file": config_file,
                     "cmd_vel_timeout_s": LaunchConfiguration("cmd_vel_timeout_s")}],
    )

    beinodometrie = Node(
        package="roboter_steuerung",
        executable="beinodometrie_node",
        name="beinodometrie_node",
        output="screen",
        parameters=[{"config_file": config_file}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration("odom_quelle"), "' == 'beinodometrie'"])),
    )

    return LaunchDescription([
        config_file_arg,
        headless_arg,
        lidar_arg,
        lidar2d_arg,
        lidar3d_arg,
        realistic_arg,
        record_arg,
        odom_arg,
        welt_arg,
        timeout_arg,
        mujoco_node,
        steuereinheit_node,
        beinodometrie,
    ])
