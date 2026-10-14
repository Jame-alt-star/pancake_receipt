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

- Navigation ohne Lokalisierung: `map→odom` statisch (Identität), Odometrie = Ground-Truth aus
  `mujoco_node` (`/odom`, TF `odom→base_link`). Keine Sensorik, daher kein Hindernis-Layer.
- Geschwindigkeitsgrenzen = Kommandobereich der Policy (vx ≤ 0,6, wz ≤ 1,0, vx ≥ 0,3 beim Fahren). Drehen auf der Stelle braucht |wz| ≳ 0,8 (kleine wz werden im Stand fast ignoriert).
- Ausgang `/cmd_vel` kommt vom `velocity_smoother` direkt an Layer 3.
- Hinweis: Layer 3 hält das letzte `/cmd_vel` ohne Watchdog; Nav2 sendet zum Zielende Nullen.

## Lidar (Stand v1.4)
- `nav2_params.yaml`: Standard mit einem 3D-Lidar. Costmaps = `VoxelLayer` auf `/punkte_hindernis`;
  `/scan` für SLAM und Collision-Monitor kommt aus `punktwolke_zu_scan` (dieses Paket).
- `nav2_params_2d.yaml`: alte Variante mit einer Scanebene (`lidar_modus:=2d`).
