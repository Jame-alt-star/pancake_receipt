"""Leitet aus der 3D-Punktwolke einen 2D-Scan ab (wie pointcloud_to_laserscan).

Eingang:  /points (sensor_msgs/PointCloud2, beliebiger Sensorframe, beliebige Felder), /imu (Orientierung der Basis),
          TF base_link <- Sensorframe (Lidar-Montage aus roboter_beschreibung, ggf. ueber die Taillengelenke)
Ausgang:  /scan   (sensor_msgs/LaserScan, Frame base_link)
          /punkte_hindernis (sensor_msgs/PointCloud2, Sensorframe): nur Punkte im Hoehenband,
          auf 5 cm ausgeduennt - fuer die Nav2-Costmaps. Die Rohwolke (~100 KB) ging im Best-Effort-
          Transport unter Last teilweise verloren; die gefilterte Wolke ist ~10x kleiner.

Die Punkte werden per TF in base_link uebertragen (frueher: feste Hoehe ueber dem Becken, Sensor achsparallel - am
echten G1 sitzt das Lidar geneigt im Kopf und bewegt sich mit der Taille), mit der IMU-Orientierung waagerecht
ausgerichtet (Roll/Nick entfernt, Gier bleibt) und je Winkelsegment der naechste Punkt innerhalb eines
Hoehenbandes ueber dem Boden genommen. Annahme: ebener Boden auf Hoehe 0, Becken in Standhoehe.
So enthaelt der Scan alle Hindernisse zwischen min_hoehe und max_hoehe - auch niedrige, die eine
einzelne Scanebene nicht sehen wuerde - aber nicht den Boden. Genutzt von slam_toolbox und dem
collision_monitor; Nav2-Costmaps lesen direkt die Punktwolke.
"""
import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from roboter_sensorik.wolke import ausduennen, eigenkoerper_maske, in_basis, maske_aus_level, scan_aus_level, waagerecht, xyz_aus_wolke


class PunktwolkeZuScan(Node):
    def __init__(self):
        super().__init__("punktwolke_zu_scan")
        # Hoehe des Basis-Ursprungs (Becken) ueber dem Boden im Stand [m]; < 0 = aus roboter_beschreibung
        self.declare_parameter("becken_hoehe", -1.0)
        self.declare_parameter("basis_frame", "base_link")
        self.declare_parameter("min_hoehe", 0.12)       # Boden ausblenden
        self.declare_parameter("max_hoehe", 1.9)
        self.declare_parameter("bins", 360)
        self.declare_parameter("range_min", 0.2)
        self.declare_parameter("range_max", 10.0)
        g = lambda n: self.get_parameter(n).value
        self.h0, self.hmin, self.hmax = float(g("becken_hoehe")), float(g("min_hoehe")), float(g("max_hoehe"))
        if self.h0 < 0:
            from roboter_beschreibung import lade as lade_roboter
            self.h0 = float(lade_roboter()["becken_hoehe_stand"])
        self.basis = str(g("basis_frame"))
        self.tf_puffer = tf2_ros.Buffer()
        self.tf_hoerer = tf2_ros.TransformListener(self.tf_puffer, self)
        self.n, self.rmin, self.rmax = int(g("bins")), float(g("range_min")), float(g("range_max"))
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.pub = self.create_publisher(LaserScan, "/scan", 10)
        self.pub_hind = self.create_publisher(PointCloud2, "/punkte_hindernis", 5)
        # Greifen: solange die Arme gefuehrt werden (/arme/zustand != aus), ist der Bereich vor dem Koerper tabu -
        # der getragene Gegenstand waere sonst ein Hindernis direkt vor dem Roboter (Collision-Monitor, Costmap).
        self.arme_gefuehrt = False
        zustand_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(String, "/arme/zustand", self._arme, zustand_qos)
        self.create_subscription(Imu, "/imu", self._imu, 10)
        self.create_subscription(PointCloud2, "/points", self._cloud, 5)
        self.get_logger().info("Punktwolke -> 2D-Scan aktiv (/points -> /scan).")

    def _arme(self, m: String):
        self.arme_gefuehrt = m.data != "aus"

    def _imu(self, m: Imu):
        o = m.orientation
        self.quat = np.array([o.w, o.x, o.y, o.z])

    def _cloud(self, m: PointCloud2):
        # Pro Punktwolke: (1) waagerechter Scan je Winkel (naechster Punkt im Hoehenband) fuer SLAM
        # und Collision-Monitor, (2) Hinderniswolke (Hoehenband, 5 cm ausgeduennt) fuer die Costmaps.
        # Ausrichtung ueber die zuletzt empfangene IMU-Orientierung.
        pts = xyz_aus_wolke(m.data, m.fields, m.point_step, m.width * m.height, m.is_bigendian)
        try:
            tf = self.tf_puffer.lookup_transform(self.basis, m.header.frame_id, Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Lidar-Montage {self.basis} <- {m.header.frame_id} fehlt: {e}",
                                   throttle_duration_sec=5.0)
            return
        level = waagerecht(in_basis(pts, tf.transform), self.quat)      # einmal je Wolke
        mask = maske_aus_level(level, self.h0, self.hmin, self.hmax, self.rmin, self.rmax)
        if self.arme_gefuehrt:
            mask &= ~eigenkoerper_maske(level)
        ranges = scan_aus_level(level, mask, self.n)
        hp = ausduennen(pts[mask]).astype(np.float32)
        hc = PointCloud2()
        hc.header = m.header
        hc.height, hc.width = 1, int(len(hp))
        # nur x, y, z weitergeben (die Eingangswolke kann weitere Felder haben)
        hc.fields = [PointField(name=k, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, k in enumerate("xyz")]
        hc.is_bigendian, hc.point_step, hc.row_step = False, 12, 12 * len(hp)
        hc.data = hp.tobytes()
        hc.is_dense = True
        self.pub_hind.publish(hc)
        sc = LaserScan()
        sc.header.stamp = m.header.stamp
        sc.header.frame_id = self.basis
        sc.angle_min = -math.pi
        sc.angle_increment = 2 * math.pi / self.n
        sc.angle_max = math.pi - sc.angle_increment
        sc.scan_time = 0.2
        sc.range_min, sc.range_max = self.rmin, self.rmax
        sc.ranges = ranges.astype(np.float32)
        self.pub.publish(sc)


def main(args=None):
    rclpy.init(args=args)
    n = PunktwolkeZuScan()
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
