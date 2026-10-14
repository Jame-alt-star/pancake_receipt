"""Leitet aus der 3D-Punktwolke einen 2D-Scan ab (wie pointcloud_to_laserscan).

Eingang:  /points (sensor_msgs/PointCloud2, Frame lidar_link) und /imu (Orientierung)
Ausgang:  /scan   (sensor_msgs/LaserScan, Frame lidar_link)
          /punkte_hindernis (sensor_msgs/PointCloud2, Frame lidar_link): nur Punkte im Hoehenband,
          auf 5 cm ausgeduennt - fuer die Nav2-Costmaps. Die Rohwolke (~100 KB) ging im Best-Effort-
          Transport unter Last teilweise verloren; die gefilterte Wolke ist ~10x kleiner.

Die Punkte werden mit der IMU-Orientierung waagerecht ausgerichtet (Roll/Nick entfernt, Gier bleibt),
dann wird je Winkelsegment der naechste Punkt innerhalb eines Hoehenbandes ueber dem Boden genommen.
So enthaelt der Scan alle Hindernisse zwischen min_hoehe und max_hoehe - auch niedrige, die eine
einzelne Scanebene nicht sehen wuerde - aber nicht den Boden. Genutzt von slam_toolbox und dem
collision_monitor; Nav2-Costmaps lesen direkt die Punktwolke.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan, PointCloud2


def hoehenband_maske(pts, quat, sensor_hoehe, min_hoehe, max_hoehe, r_min, r_max):
    """Maske der Punkte im Hoehenband (waagerecht ausgerichtet) + waagerechte Koordinaten."""
    w, x, y, z = quat
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    yaw = math.atan2(R[1, 0], R[0, 0])
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz_t = np.array([[cy, sy, 0.0], [-sy, cy, 0.0], [0.0, 0.0, 1.0]])
    level = pts @ (rz_t @ R).T
    h = level[:, 2] + sensor_hoehe
    r = np.hypot(level[:, 0], level[:, 1])
    return (h >= min_hoehe) & (h <= max_hoehe) & (r >= r_min) & (r <= r_max), level


def ausduennen(pts, voxel=0.05):
    """Ein Punkt je Wuerfel der Kantenlaenge voxel."""
    if len(pts) == 0:
        return pts
    _, idx = np.unique(np.floor(pts / voxel).astype(np.int64), axis=0, return_index=True)
    return pts[np.sort(idx)]


def scan_aus_punkten(pts, quat, sensor_hoehe, min_hoehe, max_hoehe, n_bins, r_min, r_max):
    """pts: (N,3) im Sensorframe; quat [w,x,y,z] der Basis (Sensor fest montiert, nicht gedreht).
    Gibt ranges (n_bins,) zurueck, inf = kein Treffer."""
    w, x, y, z = quat
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    yaw = math.atan2(R[1, 0], R[0, 0])
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz_t = np.array([[cy, sy, 0.0], [-sy, cy, 0.0], [0.0, 0.0, 1.0]])
    level = pts @ (rz_t @ R).T                       # waagerecht, Blickrichtung wie der Sensor
    h = level[:, 2] + sensor_hoehe                   # Hoehe ueber Boden
    r = np.hypot(level[:, 0], level[:, 1])
    ok = (h >= min_hoehe) & (h <= max_hoehe) & (r >= r_min) & (r <= r_max)
    ang = np.arctan2(level[ok, 1], level[ok, 0])
    idx = ((ang + math.pi) / (2 * math.pi) * n_bins).astype(int) % n_bins
    ranges = np.full(n_bins, np.inf)
    np.minimum.at(ranges, idx, r[ok])
    return ranges


class PunktwolkeZuScan(Node):
    def __init__(self):
        super().__init__("punktwolke_zu_scan")
        self.declare_parameter("sensor_hoehe", 1.24)    # Lidar ueber Boden im aufrechten Stand [m]
        self.declare_parameter("min_hoehe", 0.12)       # Boden ausblenden
        self.declare_parameter("max_hoehe", 1.9)
        self.declare_parameter("bins", 360)
        self.declare_parameter("range_min", 0.2)
        self.declare_parameter("range_max", 10.0)
        g = lambda n: self.get_parameter(n).value
        self.h0, self.hmin, self.hmax = float(g("sensor_hoehe")), float(g("min_hoehe")), float(g("max_hoehe"))
        self.n, self.rmin, self.rmax = int(g("bins")), float(g("range_min")), float(g("range_max"))
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.pub = self.create_publisher(LaserScan, "/scan", 10)
        self.pub_hind = self.create_publisher(PointCloud2, "/punkte_hindernis", 5)
        self.create_subscription(Imu, "/imu", self._imu, 10)
        self.create_subscription(PointCloud2, "/points", self._cloud, 5)
        self.get_logger().info("Punktwolke -> 2D-Scan aktiv (/points -> /scan).")

    def _imu(self, m: Imu):
        o = m.orientation
        self.quat = np.array([o.w, o.x, o.y, o.z])

    def _cloud(self, m: PointCloud2):
        # Pro Punktwolke: (1) waagerechter Scan je Winkel (naechster Punkt im Hoehenband) fuer SLAM
        # und Collision-Monitor, (2) Hinderniswolke (Hoehenband, 5 cm ausgeduennt) fuer die Costmaps.
        # Ausrichtung ueber die zuletzt empfangene IMU-Orientierung.
        if m.point_step != 12 or m.width == 0:
            pts = np.zeros((0, 3))
        else:
            pts = np.frombuffer(bytes(m.data), dtype=np.float32).reshape(-1, 3).astype(np.float64)
        ranges = scan_aus_punkten(pts, self.quat, self.h0, self.hmin, self.hmax, self.n, self.rmin, self.rmax)
        mask, _ = hoehenband_maske(pts, self.quat, self.h0, self.hmin, self.hmax, self.rmin, self.rmax)
        hp = ausduennen(pts[mask]).astype(np.float32)
        hc = PointCloud2()
        hc.header = m.header
        hc.height, hc.width = 1, int(len(hp))
        hc.fields = m.fields
        hc.is_bigendian, hc.point_step, hc.row_step = False, 12, 12 * len(hp)
        hc.data = hp.tobytes()
        hc.is_dense = True
        self.pub_hind.publish(hc)
        sc = LaserScan()
        sc.header = m.header
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
