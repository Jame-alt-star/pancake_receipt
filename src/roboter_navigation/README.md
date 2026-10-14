# roboter_navigation (Layer 2, Platzhalter)

Generische Nav2-Konfiguration, damit Nav2 in der Pipeline ist. **Die Variante des Kollegen
ersetzt diese Datei** (`params_file:=...` oder dieses Paket austauschen).

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
# Hinweis: bei ~2,8 GB RAM Simulation headless fahren (headless:=true)
ros2 launch roboter_navigation navigation.launch.py
# komplett mit Layer 1/3/4:
ros2 launch roboter_task_planning mission.launch.py nav2:=true headless:=true
```

- Lokalisierung: `map→odom` statisch (Identität) oder mit `slam:=true` von `slam_toolbox`;
  `/odom` + TF `odom→base_link` aus `mujoco_node` (Ground-Truth) oder `beinodometrie_node`.
- Hindernisse: 3D-Lidar → `punktwolke_zu_scan` → `VoxelLayer` (beide Costmaps) und `/scan` für
  SLAM und `collision_monitor` (siehe Abschnitt Lidar).
- Geschwindigkeitsgrenzen = Kommandobereich der Policy (vx ≤ 0,6, wz ≤ 1,0, vx ≥ 0,3 beim Fahren). Drehen auf der Stelle braucht |wz| ≳ 0,8 (kleine wz werden im Stand fast ignoriert).
- Ausgang: `velocity_smoother` → `collision_monitor` → `/cmd_vel` → Layer 3.
- Layer 3 hat einen `/cmd_vel`-Watchdog (in `mission.launch` 1 s); `/cmd_vel` gibt die Bewegung
  nicht frei (Freigabe über `/freigabe`, bei Missionen durch den Missionsknoten).

## Lidar (Stand v1.4)
- `nav2_params.yaml`: Standard mit einem 3D-Lidar. Costmaps = `VoxelLayer` auf `/punkte_hindernis`;
  `/scan` für SLAM und Collision-Monitor kommt aus `punktwolke_zu_scan` (dieses Paket).
- `nav2_params_2d.yaml`: alte Variante mit einer Scanebene (`lidar_modus:=2d`).

## Visualisierung
`rviz2 -d $(ros2 pkg prefix roboter_navigation)/share/roboter_navigation/rviz/ansicht_3d.rviz`
zeigt Karte, beide Costmaps, Punktwolke, Hinderniswolke, Voxel der lokalen Costmap, Plan und TF.
