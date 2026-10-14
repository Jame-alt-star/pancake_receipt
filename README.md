# G1-Humanoid: autonome Pipeline (Simulation)

Unitree G1 (29 DOF) in MuJoCo mit RL-Lauf-Policy (ONNX), Navigation (Nav2/SLAM) und symbolischer
Missionsplanung (PDDL/Fast Downward). Aufbau nach dem 4-Schichtenmodell der Software-Architektur.

```
Layer 1  Deliberative   roboter_task_planning   Mission (YAML) -> PDDL -> Fast Downward -> Dispatcher (ROS-2-Actions)
Layer 2  Executive/Nav  roboter_navigation      Nav2 (+ optional slam_toolbox), Lidar-Costmaps       [Variante des Kollegen ersetzbar]
Layer 3  Reactive       roboter_steuerung       Zustandsautomat (State Pattern), RL-Controller, Kurs-/Positionshaltung, NotAus, Watchdog
Layer 4  HAL            r1_bringup              IRobotHardwareBridge, mujoco_node (Physik, Lidar, Odometrie, TF)
```

Schnittstellen: Freigabe per `/freigabe` (Bool, eigener Kanal; `/cmd_vel` gibt nicht frei, während NotAus
wird jede Freigabe abgelehnt; eine Mission gibt beim Start selbst frei); Layer 2 -> 3 per `/cmd_vel`; Layer 1 -> 2 per `NavigateToPose` (Nav2) bzw. `GehAktion`;
Layer 1 -> 3 per `BalancierAktion`; Sicherheit per `/not_aus` (latched).

## Schnellstart
```bash
source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash

# Roboter allein (Viewer-Fenster), Steuerung per /cmd_vel
ros2 launch roboter_steuerung steuereinheit.launch.py config_file:=g1_policy_29dof.yaml
ros2 topic pub --once /freigabe std_msgs/msg/Bool "{data: true}"      # Bewegung freigeben (eigener Kanal)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}"

# Komplettes System in der Testwelt: Nav2 + SLAM + Mission (headless, wenig RAM)
ros2 launch roboter_task_planning mission.launch.py nav2:=true slam:=true headless:=true \
     config_file:=g1_policy_29dof.yaml \
     mission_file:=$(ros2 pkg prefix roboter_task_planning)/share/roboter_task_planning/missions/patrouille.yaml
ros2 service call /mission/start std_srvs/srv/Trigger     # Mission starten
ros2 topic echo /mission/status                           # Fortschritt
ros2 topic pub --once /not_aus std_msgs/msg/Bool "{data: true}"   # NotAus
```
Missionsdateien: `missions/beispiel.yaml`, `testwelt.yaml`, `patrouille.yaml`, `testwelt3d.yaml`, `demo_lager.yaml`
(Ziele `besucht`, `balanciert`, `gewartet`, `am_ort`). Eine Mission verweist mit `welt:` auf eine Welt aus
`roboter_welten` (Szene für die Simulation + Orte); die Simulation startet automatisch in dieser Welt.

## Realismus-Schalter
- `realistic_sensors:=true`: Sensorrauschen, Gyro-Bias und -Drift, Latenz, Lidar-Ausfälle statt idealer Werte.
- `odom_quelle:=beinodometrie`: Position aus Beinodometrie (Gelenke + IMU) statt Ground-Truth; mit
  `slam:=true` korrigiert SLAM die Drift über `map→odom`. Das ist die Konfiguration, die der echten
  Hardware am nächsten kommt.

- **Ein 3D-Lidar** (Standard, `lidar_modus:=3d`): Simulation publiziert nur `/points`; `punktwolke_zu_scan` leitet
  `/scan` (SLAM, Collision-Monitor) und `/punkte_hindernis` (Nav2-VoxelLayer) ab. Sieht auch niedrige Hindernisse
  (Test: `integration_mission.py --drei-d`). `lidar_modus:=2d` = alte Einzel-Scanebene zum Vergleich (`--nur-2d`).
- Demo-Video "Kontrollrundgang im Lager": `demo/README.md`.

## Voraussetzungen
ROS 2 Jazzy, `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`, `ros-jazzy-slam-toolbox`;
Python-Pakete `mujoco`, `onnxruntime`; **Fast Downward** (Release 24.06.1) unter `~/tools/downward`
(Anleitung: `src/roboter_task_planning/README.md`). Auf WSL2 mit ~2,8 GB RAM Simulation mit
`headless:=true` betreiben.

## Tests (alle bestanden, Stand siehe `docs/statusupdate_g1_projekt.md`)
| Aufruf | Umfang |
|---|---|
| `bash tests/alle_tests.sh` | **komplette Serie** in fester Reihenfolge (Build, pytest, alle Systemtests, ~25 min; `--schnell`, `--mit-slam`) |
| `python3 -m pytest -q` | alle schnellen Tests: Unit-Tests der Pakete, Regelkreis ohne ROS, Greif-Regression (~1 min) |
| `python3 tests/system/integration_ros.py` | Pipeline: Start, Freigabe, Gehen, Begrenzung, NotAus, Odometrie (~3 min) |
| `python3 tests/system/integration_watchdog.py` | `cmd_vel`-Watchdog |
| `... --realistisch --beinodometrie` | dieselben Integrationstests mit realistischer Sensorik und geschätzter Position |
| `python3 tests/system/integration_mission.py [--nav2 \| --welt \| --patrouille \| --slam]` | Mission mit Testdouble, Nav2, Testwelt (Hindernis, Tür), Patrouille, SLAM-Karte |

Details: `tests/README.md`. Bei Codeänderungen erst `colcon build` (Install ist eine Kopie).

## Wichtige Eigenschaften und Grenzen
- **Kommandobereich** (Layer 3 clippt): vx −0,5…1,0, vy ±0,3, wz ±1,0 m/s bzw. rad/s; oberhalb vx 0,7
  sinkt die erlaubte Drehrate linear auf 0,5. Unter ~0,3 m/s läuft der Roboter nicht an; kleine `wz`
  werden im Stand fast ignoriert (Drehen auf der Stelle braucht |wz| ≳ 0,8).
- **Stehen:** Drift < 1 cm in 120 s (Kurshaltung auch in Bereitschaft), Positionshaltung gegen Störungen.
- **Greifen:** gemessener Betriebspunkt (Arme per IK, Policy „blind“ für Arme, Taille −0,15 rad,
  0,30 m Reichweite, ≤ 1 kg je Hand); `Manipulieren`-Zustand und Hand-Modell sind **nicht** umgesetzt.
- **Position:** standardmäßig Ground-Truth aus der Simulation; mit `odom_quelle:=beinodometrie` geschätzt
  (< 1 % der Strecke). Lidar ist ein Raycast-Modell (3D, 24 Ringe, 5 Hz), optional mit Rauschen/Ausfällen.
- **Nicht umgesetzt:** Zustände AkkuSchwach/Laden (keine echten Batteriedaten), gelernte Greif-Policy,
  Wahrnehmung per Kamera, echte Hardware.

## Versionen (Git-Tags)
`git tag` listet die Rücksprungpunkte, z. B. `stand-2026-10-13-nav2-grundlage`, `v0.8-testwelt-nav2-lidar`,
`v0.9-patrouille`, `v0.10-slam`, `v1.0`. Zurück: `git checkout <tag>`.

## Weitere Dokumente
`docs/statusupdate_g1_projekt.md` (Verlauf mit allen Messwerten), `STATUSUPDATE_fuer_cowork.md`,
`src/roboter_task_planning/README.md`, `src/roboter_navigation/README.md`, `tests/README.md`.
