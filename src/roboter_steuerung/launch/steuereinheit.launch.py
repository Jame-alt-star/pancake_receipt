from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    "g1_policy.yaml" (bisheriger 12-DOF-Pfad, unveraendertes Verhalten,
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
        default_value="g1_policy.yaml",
        description="Policy-Config (im config/-Ordner von r1_bringup) - g1_policy.yaml (12-DOF, Default) oder g1_policy_29dof.yaml (29-DOF-Ganzkoerper).",
    )
    config_file = LaunchConfiguration("config_file")

    mujoco_node = Node(
        package="r1_bringup",
        executable="mujoco_node",
        name="mujoco_bridge_node",
        output="screen",
        parameters=[{"config_file": config_file}],
    )

    steuereinheit_node = Node(
        package="roboter_steuerung",
        executable="steuereinheit_node",
        name="steuereinheit_node",
        output="screen",
        parameters=[{"config_file": config_file}],
    )

    return LaunchDescription([
        config_file_arg,
        mujoco_node,
        steuereinheit_node,
    ])
