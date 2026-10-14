"""Layer 4: MuJoCo-Simulation des G1 (mujoco_node). Startet nur die Simulation; das Gesamtsystem setzt
roboter_system/launch/system.launch.py zusammen.

    ros2 launch roboter_simulation simulation.launch.py welt:=testwelt lidar_modus:=3d
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

ARGUMENTE = [
    ("welt", "", "Umgebung aus roboter_welten (leer, testwelt, testwelt3d, demo_lager); leer = nur Ebene"),
    ("headless", "false", "ohne Viewer-Fenster"),
    ("lidar_modus", "aus", "aus | 3d (/points, 24 Ringe, 5 Hz) | 2d (/scan, eine Ebene)"),
    ("realistic_sensors", "false", "Sensorrauschen, Bias, Drift, Latenz, Lidar-Ausfaelle"),
    ("record_path", "", ".npz-Datei fuer die Posenaufzeichnung"),
    ("odom_ground_truth", "true", "/odom und TF odom->base_link aus der Simulation (false: Schaetzer liefert sie)"),
]


def generate_launch_description():
    def modus(m):
        return PythonExpression(["'", LaunchConfiguration("lidar_modus"), f"' == '{m}'"])
    sim = Node(
        package="roboter_simulation", executable="mujoco_node", name="mujoco_bridge_node", output="screen",
        parameters=[{
            "welt": LaunchConfiguration("welt"),
            "headless": LaunchConfiguration("headless"),
            "lidar": PythonExpression(["'", LaunchConfiguration("lidar_modus"), "' != 'aus'"]),
            "lidar_3d": modus("3d"),
            "lidar_2d": modus("2d"),
            "realistic_sensors": LaunchConfiguration("realistic_sensors"),
            "record_path": LaunchConfiguration("record_path"),
            "publish_odom": LaunchConfiguration("odom_ground_truth"),
            "publish_tf": LaunchConfiguration("odom_ground_truth"),
        }])
    return LaunchDescription([DeclareLaunchArgument(n, default_value=d, description=t) for n, d, t in ARGUMENTE]
                             + [sim])
