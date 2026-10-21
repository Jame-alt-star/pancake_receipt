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

from roboter_beschreibung.geometrie import quat_zu_rotmat


def in_basis(pts, transform):
    """Punkte (N x 3, Sensorframe) mit einem TF-Transform (geometry_msgs/Transform: Basis <- Sensor) uebertragen."""
    q, t = transform.rotation, transform.translation
    return pts @ quat_zu_rotmat((q.w, q.x, q.y, q.z)).T + np.array([t.x, t.y, t.z])


def waagerecht(pts, quat):
    """Punkte (Sensorframe) mit der IMU-Orientierung waagerecht ausrichten: Roll/Nick entfernen,
    Gier bleibt (Blickrichtung wie der Sensor)."""
    R = quat_zu_rotmat(quat)
    yaw = math.atan2(R[1, 0], R[0, 0])
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz_t = np.array([[cy, sy, 0.0], [-sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return pts @ (rz_t @ R).T


def maske_aus_level(level, sensor_hoehe, min_hoehe, max_hoehe, r_min, r_max):
    h = level[:, 2] + sensor_hoehe
    r = np.hypot(level[:, 0], level[:, 1])
    return (h >= min_hoehe) & (h <= max_hoehe) & (r >= r_min) & (r <= r_max)


def scan_aus_level(level, mask, n_bins):
    ang = np.arctan2(level[mask, 1], level[mask, 0])
    idx = ((ang + math.pi) / (2 * math.pi) * n_bins).astype(int) % n_bins
    ranges = np.full(n_bins, np.inf)
    np.minimum.at(ranges, idx, np.hypot(level[mask, 0], level[mask, 1]))
    return ranges


def hoehenband_maske(pts, quat, sensor_hoehe, min_hoehe, max_hoehe, r_min, r_max):
    """Maske der Punkte im Hoehenband (waagerecht ausgerichtet) + waagerechte Koordinaten."""
    level = waagerecht(pts, quat)
    return maske_aus_level(level, sensor_hoehe, min_hoehe, max_hoehe, r_min, r_max), level


_NUMPY_TYP = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}   # PointField.datatype


def xyz_aus_wolke(data: bytes, fields, point_step: int, n: int, bigendian: bool = False) -> np.ndarray:
    """x, y, z aus einer beliebigen PointCloud2 (N x 3, float64), unabhaengig von weiteren Feldern.

    Echte Lidar-Treiber senden mehr als x/y/z (Livox MID-360 im G1: Intensitaet, Tag, Linie, Zeit; 18-32 Byte je
    Punkt). Frueher wurde nur genau 12 Byte (x, y, z als float32) akzeptiert und alles andere still als leere Wolke
    behandelt - auf echter Hardware haette es weder Scan noch Hindernisse gegeben. Punkte mit NaN/inf entfallen."""
    feld = {f.name: f for f in fields}
    if n == 0 or not all(k in feld for k in "xyz"):
        return np.zeros((0, 3))
    ordnung = ">" if bigendian else "<"
    dtype = np.dtype({"names": list("xyz"),
                      "formats": [ordnung + _NUMPY_TYP[feld[k].datatype] for k in "xyz"],
                      "offsets": [feld[k].offset for k in "xyz"], "itemsize": point_step})
    roh = np.frombuffer(bytes(data), dtype=dtype, count=n)
    pts = np.stack([roh["x"], roh["y"], roh["z"]], axis=1).astype(np.float64)
    return pts[np.isfinite(pts).all(axis=1)]


def ausduennen(pts, voxel=0.05):
    """Ein Punkt je Wuerfel der Kantenlaenge voxel."""
    if len(pts) == 0:
        return pts
    _, idx = np.unique(np.floor(pts / voxel).astype(np.int64), axis=0, return_index=True)
    return pts[np.sort(idx)]


def scan_aus_punkten(pts, quat, sensor_hoehe, min_hoehe, max_hoehe, n_bins, r_min, r_max):
    """pts: (N,3) im Sensorframe; quat [w,x,y,z] der Basis (Sensor fest montiert, nicht gedreht).
    Gibt ranges (n_bins,) zurueck, inf = kein Treffer."""
    level = waagerecht(pts, quat)
    return scan_aus_level(level, maske_aus_level(level, sensor_hoehe, min_hoehe, max_hoehe, r_min, r_max), n_bins)


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
        pts = xyz_aus_wolke(m.data, m.fields, m.point_step, m.width * m.height, m.is_bigendian)
        try:
            tf = self.tf_puffer.lookup_transform(self.basis, m.header.frame_id, Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Lidar-Montage {self.basis} <- {m.header.frame_id} fehlt: {e}",
                                   throttle_duration_sec=5.0)
            return
        level = waagerecht(in_basis(pts, tf.transform), self.quat)      # einmal je Wolke
        mask = maske_aus_level(level, self.h0, self.hmin, self.hmax, self.rmin, self.rmax)
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
