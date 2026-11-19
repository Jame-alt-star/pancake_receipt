# roboter_navigation (Layer 2, Platzhalter)

Generische Nav2-Konfiguration, damit Nav2 in der Pipeline ist. **Die Variante des Kollegen
ersetzt diese Datei** (`params_file:=...` oder dieses Paket austauschen); was sie dafür einhalten muss, steht unter
[Eigene Nav2-Variante einbinden](#eigene-nav2-variante-einbinden).

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
# Hinweis: bei ~2,8 GB RAM Simulation headless fahren (headless:=true)
ros2 launch roboter_navigation navigation.launch.py          # nur Layer 2
# komplett mit Layer 1/3/4 und Sensorik:
ros2 launch roboter_system system.launch.py mission:=testwelt navigation:=nav2 headless:=true
```

- Lokalisierung: `map→odom` statisch (Identität), mit `slam:=true` von `slam_toolbox` oder mit `karte:=<karte.yaml>`
  (System: `karte:=true`) von AMCL auf der festen Karte der Welt (dann plant die globale Costmap auf der ganzen Karte);
  `/odom` + TF `odom→base_link` aus `mujoco_node` (Ground-Truth) oder `beinodometrie_node`.
- Hindernisse: 3D-Lidar → `punktwolke_zu_scan` (Paket `roboter_sensorik`) → `VoxelLayer` (beide Costmaps) und `/scan` für
  SLAM und `collision_monitor` (siehe Abschnitt Lidar).
- Geschwindigkeitsgrenzen dieser Konfiguration: vx ≤ 0,6, wz ≤ 1,0, vx ≥ 0,3 beim Fahren (innerhalb des
  Kommandobereichs der Steuerung, siehe unten).
- Ausgang: `velocity_smoother` → `collision_monitor` → `/cmd_vel` → Layer 3.

## Eigene Nav2-Variante einbinden

Die Schicht ist abgegrenzt: außerhalb von `roboter_navigation` muss nichts geändert werden.

**Eingänge**

| Topic / Frame | Typ | Kommt von |
|---|---|---|
| `/odom`, TF `odom → base_link` | `nav_msgs/Odometry` | Simulation (Ground-Truth) oder Beinodometrie (`odom_quelle:=beinodometrie`) |
| `/scan` | `sensor_msgs/LaserScan`, Frame `base_link` | `roboter_sensorik` (aus dem 3D-Lidar abgeleitet) – für SLAM, AMCL und `collision_monitor` |
| `/punkte_hindernis` | `sensor_msgs/PointCloud2`, Frame `lidar_link` | `roboter_sensorik` – Hinderniswolke für die Costmaps |
| TF-Baum des Roboters | `robot_state_publisher` | `base_link → pelvis → …`, `lidar_link` am Glied aus `roboter_beschreibung/config/roboter.yaml` |
| Karte je Welt | `roboter_welten/welten/<welt>/karte.yaml` | aus der Szene gerechnet; mit `karte:=true` nutzt der Platzhalter sie (map_server + AMCL) |
| TF `map → odom` | | **diese Schicht**: statisch, SLAM oder AMCL auf fester Karte |

**Ausgang:** `/cmd_vel` (`geometry_msgs/Twist`) an die Steuerung (Layer 3).

**Aufgerufen** wird die Schicht von der Missionsplanung (Layer 1) über die Actions `navigate_to_pose` (fahren) und
`compute_path_to_pose` (Weglänge zwischen zwei Orten für die Planung; Nav2-Standard des `planner_server`, fällt sie aus,
nimmt Layer 1 die Luftlinie).

**Eigenheiten des Roboters**, die die Parameter berücksichtigen müssen:
- Kommandobereich der Steuerung (wird dort begrenzt): vx −0,5…1,0 m/s, vy ±0,3 m/s, wz ±1,0 rad/s. Oberhalb vx 0,7 sinkt
  die erlaubte Drehrate linear auf 0,5 bei vx 1,0.
- Unter ~0,3 m/s läuft der Roboter nicht los.
- Drehen auf der Stelle braucht |wz| ≳ 0,8; kleine Drehbefehle setzt er im Stand kaum um.
- `/cmd_vel` gibt die Bewegung nicht frei: vorher muss `/freigabe` gesendet sein (bei Missionen macht das der
  Missionsknoten).
- Watchdog: bleibt `/cmd_vel` länger als 1 s aus, setzt die Steuerung den Wunsch auf 0.
- Vorrang: `/cmd_vel/hand` (Bediener) vor `/cmd_vel/balance` (Balance-Dienst) vor `/cmd_vel`, jeweils solange sie
  senden. Das eigene `/cmd_vel` wird dabei kurz überstimmt, nicht gestört.

**Hinweise für die Nav2-Konfiguration** (Befunde mit dem Platzhalter):
- RPP `max_angular_accel` hoch setzen (bezieht sich auf die gemessene Drehrate, die die Policy träge umsetzt);
  `PoseProgressChecker` (zählt auch Drehen als Fortschritt).
- Hindernisse in 3D: `VoxelLayer` auf `/punkte_hindernis` – die 2D-`ObstacleLayer` löscht niedrige Hindernisse wieder.
- In Jazzy brauchen `collision_monitor` und `docking_server` eine Konfiguration, auch wenn man sie nicht nutzt.
  Einmal hing beim Hochfahren die Aktivierung des `docking_server` (Lifecycle-Manager wartete endlos): wer kein
  Andocken braucht, lässt ihn in der eigenen Konfiguration weg.
- `navigation_launch.py` mit `use_composition=True` startet den Container nicht selbst (macht `navigation.launch.py`).

**So wird die Variante eingebunden**
```bash
git fetch origin && git checkout -b nav2 origin/pascal/pipeline-v2.4
# eigene Parameter testen, ohne unseren Code zu ändern:
ros2 launch roboter_system system.launch.py mission:=testwelt navigation:=nav2 headless:=true nav2_params:=<deine.yaml>
# oder eigenes Nav2 selbst starten und unser System ohne eigenes Nav2 dazu
# (Lidar, Sensorik und Watchdog laufen wie mit unserem Nav2):
ros2 launch roboter_system system.launch.py mission:=testwelt navigation:=extern headless:=true
```
Referenz zum Vergleichen: `config/nav2_params.yaml` (3D-Lidar) und die Overlays `nav2_params_2d.yaml` (eine Scanebene)
und `nav2_karte.yaml` (feste Karte + AMCL). Wenn `bash tests/alle_tests.sh` mit der eigenen Konfiguration grün ist,
Pull Request auf `pascal/pipeline-v2.4`.

## Lidar (Stand v1.4)
- `nav2_params.yaml`: Standard mit einem 3D-Lidar. Costmaps = `VoxelLayer` auf `/punkte_hindernis`;
  `/scan` für SLAM und Collision-Monitor kommt aus `punktwolke_zu_scan` (`roboter_sensorik`, bleibt auch
  bei einer anderen Nav2-Variante erhalten).
- `nav2_params_2d.yaml`: **Overlay** mit nur den Unterschieden für eine Scanebene (`lidar_modus:=2d`);
  `nav2_karte.yaml`: Overlay für die feste Karte (static_layer, AMCL, map_server). `navigation.launch.py` legt
  Overlays über die Basisdatei (Argument `overlays`); eigene Parameter (`nav2_params:=`) bleiben unverändert.

## Visualisierung
`system.launch.py ... rviz:=true` (Ansicht `roboter_system/rviz/ansicht_3d.rviz`)
zeigt Karte, beide Costmaps, Punktwolke, Hinderniswolke, Voxel der lokalen Costmap, Plan und TF.
