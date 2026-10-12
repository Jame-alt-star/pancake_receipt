from launch import LaunchDescription
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
    """
    mujoco_node = Node(
        package="r1_bringup",
        executable="mujoco_node",
        name="mujoco_bridge_node",
        output="screen",
    )

    steuereinheit_node = Node(
        package="roboter_steuerung",
        executable="steuereinheit_node",
        name="steuereinheit_node",
        output="screen",
    )

    return LaunchDescription([
        mujoco_node,
        steuereinheit_node,
    ])
