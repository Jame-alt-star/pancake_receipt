"""Zustandsschaetzer (Beinodometrie) als ROS-2-Knoten.

Ersetzt die Ground-Truth-Odometrie der Simulation: liest /joint_states und /imu (wie auf echter
Hardware) und publiziert /odom (nav_msgs/Odometry, odom->base_link) sowie die TF odom->base_link.
Start ueber steuerung.launch.py bzw. system.launch.py odom_quelle:=beinodometrie (mujoco_node publiziert dann kein /odom).
"""
import math
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

from roboter_steuerung.zustandsschaetzung.beinodometrie import Beinodometrie


class BeinodometrieNode(Node):
    def __init__(self):
        super().__init__("beinodometrie_node")
        self.declare_parameter("config_file", "g1_policy_29dof.yaml")
        self.declare_parameter("start_position", [0.0, 0.0, -1.0])   # z < 0: aus Config (becken_hoehe_stand)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        share = get_package_share_directory("roboter_simulation")
        with open(os.path.join(share, "config", self.get_parameter("config_file").value)) as f:
            cfg = yaml.safe_load(f)
        xml = os.path.join(share, cfg["xml_path"])
        self.est = Beinodometrie(xml)
        start = list(self.get_parameter("start_position").value)
        if start[2] < 0:
            start[2] = float(cfg.get("becken_hoehe_stand", 0.793))
        self.est.reset(start)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.quat = None
        self.omega = np.zeros(3)
        self.pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Imu, "/imu", self._imu, 10)
        self.create_subscription(JointState, "/joint_states", self._joints, 10)
        self.get_logger().info(f"Beinodometrie aktiv (Modell {cfg['xml_path']}).")

    def _imu(self, m: Imu):
        o = m.orientation
        self.quat = np.array([o.w, o.x, o.y, o.z])
        self.omega = np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])

    def _joints(self, m: JointState):
        # Pro /joint_states-Nachricht (100 Hz) ein Schaetzschritt mit der zuletzt empfangenen IMU-
        # Orientierung; Zeit aus dem Nachrichtenstempel (nicht Empfangszeit), damit Latenz und
        # Jitter des Transports die Geschwindigkeitsschaetzung nicht verfaelschen.
        if self.quat is None or not m.position:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        pos, vel, quat = self.est.update(t, np.array(m.position), self.quat)
        w, x, y, z = quat
        # Weltgeschwindigkeit -> Koerperframe (ROS-Konvention fuer Odometry.twist)
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        vb = r.T @ vel
        od = Odometry()
        od.header.stamp = m.header.stamp
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = map(float, pos)
        od.pose.pose.orientation.w, od.pose.pose.orientation.x = float(w), float(x)
        od.pose.pose.orientation.y, od.pose.pose.orientation.z = float(y), float(z)
        od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = map(float, vb)
        od.twist.twist.angular.x, od.twist.twist.angular.y, od.twist.twist.angular.z = map(float, self.omega)
        self.pub.publish(od)
        tr = TransformStamped()
        tr.header = od.header
        tr.child_frame_id = self.base_frame
        tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = map(float, pos)
        tr.transform.rotation = od.pose.pose.orientation
        self.tf.sendTransform(tr)


def main(args=None):
    rclpy.init(args=args)
    n = BeinodometrieNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
