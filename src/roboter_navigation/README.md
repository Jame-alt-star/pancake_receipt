# roboter_navigation (Layer 2, Platzhalter)

Generische Nav2-Konfiguration, damit Nav2 in der Pipeline ist. **Die Variante des Kollegen
ersetzt diese Datei** (`params_file:=...` oder dieses Paket austauschen).

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
ros2 launch roboter_navigation navigation.launch.py
# komplett mit Layer 1/3/4:
ros2 launch roboter_task_planning mission.launch.py nav2:=true
```

- Navigation ohne Lokalisierung: `map→odom` statisch (Identität), Odometrie = Ground-Truth aus
  `mujoco_node` (`/odom`, TF `odom→base_link`). Keine Sensorik, daher kein Hindernis-Layer.
- Geschwindigkeitsgrenzen = Trainingsbereich der Policy (wz ≤ 0,2 rad/s, vx ≥ 0,3 beim Fahren).
- Ausgang `/cmd_vel` kommt vom `velocity_smoother` direkt an Layer 3.
- Hinweis: Layer 3 hält das letzte `/cmd_vel` ohne Watchdog; Nav2 sendet zum Zielende Nullen.
