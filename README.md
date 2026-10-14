# G1-Humanoid: autonome Pipeline (Simulation)

Unitree G1 (29 DOF) in MuJoCo mit RL-Lauf-Policy (ONNX), Navigation (Nav2/SLAM) und symbolischer
Missionsplanung (PDDL/Fast Downward). Aufbau nach dem 4-Schichtenmodell der Software-Architektur;
Paketübersicht, Schnittstellen und Abhängigkeitsregeln: `docs/architektur.md`.

```
Layer 1  Deliberative   roboter_task_planning   Mission (Auftrag) -> PDDL -> Fast Downward -> Dispatcher (ROS-2-Actions)
Layer 2  Navigation     roboter_navigation      Nav2 (+ optional slam_toolbox)                       [Variante des Kollegen ersetzbar]
         Sensorik       roboter_sensorik        3D-Punktwolke -> /scan + Hinderniswolke
Layer 3  Reactive       roboter_steuerung       Zustandsautomat, RL-Policy (ONNX + policy_29dof.yaml), Kurs-/Positionshaltung,
                                                NotAus, Watchdog, Hardware-Bridge (Schnittstelle zu Layer 4)
Layer 4  HAL            roboter_simulation      mujoco_node (Physik, Sensoren, Lidar, Odometrie, TF)
Roboter                 roboter_beschreibung    Unitree-G1-Modell + Geometrie (Becken-/Lidar-Hoehe)
Umgebung                roboter_welten          je Welt: Szene (Simulation) + Orte (Missionen)
Gesamt                  roboter_system          system.launch.py setzt die Schichten zusammen, RViz-Ansicht
```

Schnittstellen: Freigabe per `/freigabe` (Bool, eigener Kanal; `/cmd_vel` gibt nicht frei, während NotAus
wird jede Freigabe abgelehnt; eine Mission gibt beim Start selbst frei); Layer 2 -> 3 per `/cmd_vel`; Layer 1 -> 2 per `NavigateToPose` (Nav2) bzw. `GehAktion`;
Layer 1 -> 3 per `BalancierAktion`; Sicherheit per `/not_aus` (latched).

## Schnellstart
```bash
source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash

# Roboter allein (Layer 3+4, Viewer-Fenster), Steuerung per /cmd_vel
ros2 launch roboter_system system.launch.py
ros2 topic pub --once /freigabe std_msgs/msg/Bool "{data: true}"      # Bewegung freigeben (eigener Kanal)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}"

# Komplettes System: Mission + Nav2 + SLAM (headless, wenig RAM); die Welt kommt aus der Mission
ros2 launch roboter_system system.launch.py mission:=patrouille navigation:=nav2 slam:=true headless:=true
ros2 service call /mission/start std_srvs/srv/Trigger     # Mission starten (oder auto_start:=true)
ros2 topic echo /mission/status                           # Fortschritt
# NotAus: /not_aus ist latched (TRANSIENT_LOCAL), der Sender muss dieselbe QoS haben, sonst wartet er ewig
ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable /not_aus std_msgs/msg/Bool "{data: true}"
```
`system.launch.py`-Schalter: `mission` (Name oder Pfad), `welt`, `navigation` (aus/nav2/testdouble), `nav2_params`,
`slam`, `lidar_modus` (aus/3d/2d), `odom_quelle`, `realistic_sensors`, `headless`, `record_path`, `cmd_vel_timeout_s`,
`auto_start`, `rviz`. Jede Schicht lässt sich auch einzeln starten (Launch-Datei im jeweiligen Paket).

Missionen (`roboter_task_planning/missions/`): `beispiel`, `testwelt`, `patrouille`, `testwelt3d`, `demo_lager`
(Ziele `besucht`, `balanciert`, `gewartet`, `am_ort`). Eine Mission verweist mit `welt:` auf eine Welt aus
`roboter_welten` (Szene für die Simulation + Orte); die Simulation startet automatisch in dieser Welt.

## Realismus-Schalter
- `realistic_sensors:=true`: Sensorrauschen, Gyro-Bias und -Drift, Latenz, Lidar-Ausfälle statt idealer Werte.
- `odom_quelle:=beinodometrie`: Position aus Beinodometrie (Gelenke + IMU) statt Ground-Truth; mit
  `slam:=true` korrigiert SLAM die Drift über `map→odom`. Das ist die Konfiguration, die der echten
  Hardware am nächsten kommt.

- **Ein 3D-Lidar** (Standard mit Navigation, `lidar_modus:=3d`): Simulation publiziert nur `/points`; `punktwolke_zu_scan` leitet
  `/scan` (SLAM, Collision-Monitor) und `/punkte_hindernis` (Nav2-VoxelLayer) ab. Sieht auch niedrige Hindernisse
  (Test: `tests/system/integration_mission.py --drei-d`). `lidar_modus:=2d` = alte Einzel-Scanebene zum Vergleich (`--nur-2d`).
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
`git tag` listet die Rücksprungpunkte, z. B. `v1.0` (Pipeline), `v1.9-review-abgeschlossen` (letzter Stand vor dem
Strukturumbau), `v2.0`–`v2.4` (Strukturumbau je Schritt). Zurück: `git checkout <tag>` (danach `colcon build`;
beim Sprung über v2.2 die Ordner `build/` und `install/` vorher leeren, weil Pakete umbenannt wurden).

## Weitere Dokumente
`docs/stand_fuer_kollegen.md` (was seit dem 10.10. neu ist, für den Nav2-Kollegen), `docs/architektur.md` (Pakete, Abhängigkeitsregeln, Schnittstellen), `docs/statusupdate_g1_projekt.md` (Verlauf mit allen
Messwerten), `STATUSUPDATE_fuer_cowork.md`, `src/*/README.md` (Layer 1, Layer 2, Welten), `tests/README.md`, `archiv/README.md`.
