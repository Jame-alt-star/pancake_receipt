#!/usr/bin/env python3
"""
steuerung_gazebo.launch.py

Dritter Schritt der Simulation: wie spawnen_gazebo.launch.py, aber
zusaetzlich mit echter Gelenksteuerung ueber ros2_control +
gz_ros2_control -- statt nur passiv in der Physik zu haengen/umzufallen.

Neu gegenueber spawnen_gazebo.launch.py:
  1. Der URDF-Text bekommt programmatisch einen <ros2_control>-Block
     hinzugefuegt (genau wie beim Mesh-Pfad-Fix -- Unitrees Original
     R1.urdf bleibt dabei unveraendert), mit allen 26 beweglichen
     Gelenken als Position-Command- / Position+Velocity-State-
     Interfaces. Die Gelenknamen entsprechen 1:1 den <joint>-Elementen
     in R1.urdf und den in R1Spezifikation.sysml (SysML-Modell)
     definierten Aktuatoren (z.B. left_hip_pitch_joint <->
     HueftPitchAktuator im linken Bein).
  2. Ein <gazebo><plugin>-Block registriert das gz_ros2_control-Plugin
     mit Verweis auf die Controller-Konfiguration
     config/r1_controllers.yaml (in diesem Paket).
  3. Zwei Controller werden per Spawner geladen, zeitlich nach dem
     Roboter-Spawn (damit gz_ros2_control das Modell schon kennt):
     - joint_state_broadcaster: liest Ist-Gelenkwinkel/-geschwindigkeit
       zurueck (Voraussetzung fuer jede Regelung/Rueckmeldung).
     - joint_trajectory_controller: nimmt Trajectory-Ziele entgegen --
       genau das Interface, an das steuereinheit_node's "gehen"/
       "balancieren"-Actions in einem spaeteren Schritt echte Ziele
       schicken sollen (siehe Projektstand, Abschnitt "Naechster
       Schritt", Phase B). Fuer diesen Schritt (Phase A) wird nur
       geprueft, ob sich ein einzelnes Gelenk per CLI ansteuern laesst.
  4. NEU: ein simulierter IMU-Sensor wird an "imu_in_pelvis_link"
     angehaengt (die physische Halterung, die in Unitrees Original-URDF
     bereits als leeres Bauteil existiert, aber ohne jede Sensorlogik).
     Voraussetzung dafuer ist das Welt-Plugin "gz-sim-imu-system" in
     worlds/r1_welt.sdf -- ohne das liefert der Sensor keine Werte.
     Per Test bestaetigt (09.09.2026): Gazebo-Topic heisst "/imu",
     liefert plausible Werte (linear_acceleration.z ~= 9.8 m/s^2 im
     Stillstand -- das ist die Erdbeschleunigung, wie erwartet). Eine
     zusaetzliche ros_gz_bridge macht "/imu" als ROS2-Topic
     (sensor_msgs/msg/Imu) verfuegbar, analog zur bestehenden
     /clock-Bruecke.

Voraussetzung zusaetzlich zu spawnen_gazebo.launch.py: die apt-Pakete
fuer ros2_control + gz_ros2_control muessen installiert sein (siehe
README, neuer Abschnitt "Schritt 3").

Aufruf (nach colcon build + source install/setup.bash):

    ros2 launch r1_bringup steuerung_gazebo.launch.py

Test in einem zweiten Terminal, sobald Gazebo laeuft (ein paar Sekunden
warten, bis beide Controller-Spawner durchgelaufen sind -- Log-Ausgabe
beobachten):

    ros2 control list_controllers

    ros2 topic pub /joint_trajectory_controller/joint_trajectory \\
      trajectory_msgs/msg/JointTrajectory \\
      "{joint_names: ['left_elbow_joint'], points: [{positions: [1.0], time_from_start: {sec: 2}}]}" \\
      --once

Erwartung: der linke Ellbogen bewegt sich in Gazebo sichtbar in die
Zielstellung (1.0 rad), waehrend der Rest des Koerpers weiterhin ohne
aktive Regelung ist (faellt ggf. um -- das ist fuer diesen Schritt kein
Fehler, siehe spawnen_gazebo.launch.py).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Pfad zum geklonten r1_description-Ordner von Unitree -- siehe README.
R1_URDF_PFAD = os.path.expanduser(
    "~/Ros2/unitree_ros/robots/r1_description/R1.urdf"
)

# Alle 26 beweglichen Gelenke aus R1.urdf, in der Reihenfolge des
# offiziellen URDF. Muss 1:1 mit den "joints" in config/r1_controllers.yaml
# uebereinstimmen.
R1_GELENKE = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_roll_joint", "waist_yaw_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "head_pitch_joint", "head_yaw_joint",
]


def _urdf_lesen_und_meshpfade_fixen(urdf_pfad: str) -> str:
    """Gleiche Logik wie in anzeigen_rviz.launch.py/spawnen_gazebo.launch.py."""
    with open(urdf_pfad, "r") as f:
        text = f.read()
    mesh_ordner = os.path.join(os.path.dirname(urdf_pfad), "meshes")
    return text.replace('filename="meshes/', f'filename="file://{mesh_ordner}/')


def _ros2_control_block_einfuegen(urdf_text: str, controller_yaml_pfad: str) -> str:
    """Fuegt einen <ros2_control>- und einen <gazebo><plugin>-Block direkt
    vor dem schliessenden </robot> ein -- programmatisch wie der
    Mesh-Pfad-Fix, damit R1.urdf selbst (Unitrees Original) unveraendert
    bleibt und bei einem erneuten git clone/pull nichts verloren geht."""
    gelenke_xml = "\n".join(
        f'    <joint name="{name}">\n'
        f'      <command_interface name="position"/>\n'
        f'      <state_interface name="position"/>\n'
        f'      <state_interface name="velocity"/>\n'
        f'    </joint>'
        for name in R1_GELENKE
    )
    ros2_control_block = (
        '  <ros2_control name="R1GazeboSystem" type="system">\n'
        '    <hardware>\n'
        '      <plugin>gz_ros2_control/GazeboSimSystem</plugin>\n'
        '    </hardware>\n'
        f'{gelenke_xml}\n'
        '  </ros2_control>\n'
        '  <gazebo>\n'
        '    <plugin filename="gz_ros2_control-system" '
        'name="gz_ros2_control::GazeboSimROS2ControlPlugin">\n'
        f'      <parameters>{controller_yaml_pfad}</parameters>\n'
        '    </plugin>\n'
        '  </gazebo>\n'
    )
    return urdf_text.replace("</robot>", ros2_control_block + "</robot>")


def _imu_sensor_einfuegen(urdf_text: str) -> str:
    """Fuegt einen <gazebo>-Sensorblock fuer "imu_in_pelvis_link" ein --
    programmatisch, gleiches Vorgehen wie beim ros2_control-Block, damit
    R1.urdf selbst unangetastet bleibt. "imu_in_pelvis_link" existiert
    in Unitrees Original-URDF bereits als reines Halterungs-Bauteil
    (Mesh, keine Sensorlogik) -- dieser Block macht daraus einen echten
    simulierten Sensor. Braucht zusaetzlich das Welt-Plugin
    "gz-sim-imu-system" (siehe worlds/r1_welt.sdf), sonst bleiben die
    Werte leer."""
    imu_block = (
        '  <gazebo reference="imu_in_pelvis_link">\n'
        '    <sensor name="imu_sensor" type="imu">\n'
        '      <always_on>true</always_on>\n'
        '      <update_rate>100</update_rate>\n'
        '      <visualize>false</visualize>\n'
        '      <topic>imu</topic>\n'
        '    </sensor>\n'
        '  </gazebo>\n'
    )
    return urdf_text.replace("</robot>", imu_block + "</robot>")


def generate_launch_description():
    if not os.path.exists(R1_URDF_PFAD):
        raise FileNotFoundError(
            f"R1.urdf nicht gefunden unter {R1_URDF_PFAD}. Siehe README: "
            "unitree_ros muss vorher geklont werden (git clone, "
            "sparse-checkout robots/r1_description)."
        )

    controller_yaml_pfad = os.path.join(
        get_package_share_directory("r1_bringup"), "config", "r1_controllers.yaml"
    )
    welt_pfad = os.path.join(
        get_package_share_directory("r1_bringup"), "worlds", "r1_welt.sdf"
    )

    robot_description = _urdf_lesen_und_meshpfade_fixen(R1_URDF_PFAD)
    robot_description = _ros2_control_block_einfuegen(robot_description, controller_yaml_pfad)
    robot_description = _imu_sensor_einfuegen(robot_description)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": f"-r {welt_pfad}"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="r1_spawner",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "unitree_r1", "-z", "1.0"],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
    )

    # Macht das per Test bestaetigte Gazebo-Topic "/imu" als ROS2-Topic
    # verfuegbar (gleiche Richtung wie clock_bridge: Gazebo -> ROS2,
    # daher das "[" statt "]" oder "@" in der Argument-Syntax).
    imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="imu_bridge",
        output="screen",
        arguments=["/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"],
        parameters=[{"use_sim_time": True}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    # Zeitliche Staffelung: Welt+Spawn brauchen 5s (siehe
    # spawnen_gazebo.launch.py). Die Controller-Spawner muessen NACH dem
    # Roboter-Spawn UND nachdem gz_ros2_control sich beim
    # controller_manager registriert hat kommen -- daher grosszuegiger
    # Puffer (8s/9s). Falls "joint_state_broadcaster" beim ersten Start
    # mit "controller_manager not available" fehlschlaegt, Zeiten weiter
    # erhoehen statt an der Logik zu zweifeln -- typischer Cold-Start-Effekt.
    spawn_verzoegert = TimerAction(period=5.0, actions=[spawn])
    bridge_verzoegert = TimerAction(period=5.0, actions=[clock_bridge])
    # 6s statt 5s: das /imu-Topic existiert erst, nachdem der Roboter
    # (bei 5s) tatsaechlich gespawnt wurde -- kleiner Puffer dahinter.
    imu_bridge_verzoegert = TimerAction(period=6.0, actions=[imu_bridge])
    jsb_verzoegert = TimerAction(period=8.0, actions=[joint_state_broadcaster_spawner])
    jtc_verzoegert = TimerAction(period=9.0, actions=[joint_trajectory_controller_spawner])

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_verzoegert,
        bridge_verzoegert,
        imu_bridge_verzoegert,
        jsb_verzoegert,
        jtc_verzoegert,
    ])
