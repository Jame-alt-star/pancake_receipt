# Was ist neu seit dem Stand vom 10.10.2026?

Übersicht für den Nav2-Kollegen · Stand 15.10.2026 (`v2.5`) · Branch `pascal/pipeline-v2.4`

**Wozu diese Datei:** einmaliger Vergleich „dein Stand vom 10.10. → jetzt“, damit du nicht die ganze Historie lesen musst.
Den laufend aktualisierten Stand findest du in `PROJEKTSTAND_AKTUELL.md`.

Dein Repo `pancake_receipt` (`main`) zeigt das Projekt vom 10.10.: Der G1 lief mit **12 Gelenken**, startete mit einem
festen Gehbefehl, und vieles war erst als leere Datei angelegt (Zustände, PDDL, Planer, Dispatcher). Seitdem ist
daraus eine **komplette Pipeline über alle vier Schichten** geworden, die in der Simulation selbstständig Missionen
abfährt, Hindernissen ausweicht und mit einer realistischen Sensorik zurechtkommt.

Diese Datei erklärt, was fertig ist, was neu ist, was sich umbenannt hat und was das für deine Nav2-Arbeit bedeutet.
Details stehen in den verlinkten Dokumenten, du musst nichts davon vorher lesen.

---

## Auf einen Blick

| | Dein Stand (10.10.) | Jetzt (15.10.) |
|---|---|---|
| Roboter | G1 mit 12 Gelenken (nur Beine), TorchScript-Policy | G1 mit **29 Gelenken** (Beine, Taille, Arme), ONNX-Policy von Unitree |
| Steuerung | fester Startbefehl aus der YAML, kein `/cmd_vel` | `/cmd_vel` von beliebiger Quelle, Zustandsautomat, NotAus, Freigabe, Watchdog |
| Navigation | – | **Nav2** als Platzhalter (für deine Variante austauschbar), optional SLAM |
| Missionsplanung | leere Dateien | PDDL + Fast Downward, Neuplanung bei Fehlern, 5 Beispielmissionen |
| Sensorik | IMU, Gelenke | zusätzlich **3D-Lidar**, Odometrie, optional realistisches Rauschen und Latenz |
| Umgebungen | leere Ebene | 4 Welten (u. a. Testwelt mit Tür, Lagerhalle) |
| Starten | mehrere Terminals | **ein Befehl**: `ros2 launch roboter_system system.launch.py ...` |
| Tests | ament-Vorlagen | 64 schnelle Tests + 9 Systemtests, **alle grün** |

---

## Was fertig ist und funktioniert

Alles hier ist in der Simulation getestet, die Zahlen sind gemessen.

**Layer 4 – Simulation** (`roboter_simulation`)
- MuJoCo in Echtzeit, 500 Hz Physik, Sensordaten mit 100 Hz.
- Wartet beim Start, bis die Steuerung bereit ist. Vorher fiel der Roboter um, bevor die Policy das erste Kommando senden konnte.
- 3D-Lidar (24 Ringe, 5 Hz) als Raycast auf die Welt, alternativ eine 2D-Scanebene.
- Odometrie und TF `odom → base_link` aus der Simulation.
- Schalter `realistic_sensors`: Rauschen, Gyro-Bias, Gier-Drift, 10 ms Latenz, Lidar-Ausfälle.

**Layer 3 – Steuerung** (`roboter_steuerung`)
- Zustandsautomat: Start → Bereitschaft → Gehen / Balancieren, dazu NotAus aus jedem Zustand.
- Bewegung nur nach **bewusster Freigabe** über `/freigabe`. `/cmd_vel` allein bewegt den Roboter nicht.
- Fahrbefehle werden weich gerampt und auf den Bereich begrenzt, in dem die Policy trainiert wurde.
- Kurshaltung (Drift < 1 cm in 120 s im Stand) und Positionshaltung gegen Stöße.
- Watchdog: Bleibt `/cmd_vel` länger als 1 s aus, bleibt der Roboter stehen.
- **Beinodometrie**: Position aus Gelenken und IMU statt aus der Simulation, Fehler < 1 % der Strecke.

**Layer 2 – Navigation** (`roboter_navigation`, Platzhalter für dich)
- Nav2 mit Regulated Pure Pursuit und NavFn, Hinderniserkennung in 3D (VoxelLayer).
- Optional `slam_toolbox`, der die Karte während der Fahrt baut.
- Umfährt in der Testwelt Hindernis und Tür. Eine nur 0,45 m hohe Palette wird mit dem 3D-Lidar erkannt
  (Abstand 0,26–0,45 m); mit der alten 2D-Scanebene gab es Kontakt.

**Layer 1 – Missionsplanung** (`roboter_task_planning`)
- Mission als YAML → PDDL-Problem → Fast Downward → Plan → Ausführung über ROS-2-Actions.
- Ziele: Ort besuchen, dort balancieren, warten, am Ende an einem Ort stehen.
- Scheitert eine Etappe, plant er von der aktuellen Position neu. Bei NotAus bricht er ab.

**Gesamtsystem**
- Demo-Video „Kontrollrundgang im Lager“ (`demo/`).
- RViz-Ansicht mit Karte, Costmaps, Punktwolke und Pfad.

---

## Was du für Nav2 wissen musst

Das ist der wichtigste Teil für dich. Deine Schicht ist sauber abgegrenzt, du musst außerhalb von
`roboter_navigation` nichts anfassen.

**Deine Eingänge**

| Topic / Frame | Typ | Kommt von |
|---|---|---|
| `/odom`, TF `odom → base_link` | `nav_msgs/Odometry` | Simulation (Ground-Truth) oder Beinodometrie |
| `/scan` | `sensor_msgs/LaserScan` | `roboter_sensorik` (aus dem 3D-Lidar abgeleitet) – für SLAM und Collision-Monitor |
| `/punkte_hindernis` | `sensor_msgs/PointCloud2` | `roboter_sensorik` – Hinderniswolke für die Costmaps |
| TF `base_link → lidar_link` | statisch | Simulation |
| TF `map → odom` | | **deine Schicht**: statisch oder SLAM |

**Dein Ausgang:** `/cmd_vel` (`geometry_msgs/Twist`) an die Steuerung.

**Aufgerufen wirst du** von der Missionsplanung über die Action `navigate_to_pose` (Nav2-Standard).

**Besonderheiten des Roboters**, die deine Parameter berücksichtigen müssen:
- Kommandobereich: vx −0,5…1,0 m/s, vy ±0,3, wz ±1,0 rad/s. Oberhalb vx 0,7 sinkt die erlaubte Drehrate auf 0,5.
- Unter ~0,3 m/s läuft der Roboter nicht los.
- Drehen auf der Stelle braucht |wz| ≳ 0,8. Kleine Drehbefehle setzt er im Stand kaum um.
- Vor dem ersten `/cmd_vel` muss `/freigabe` gesendet sein. Die Missionsplanung macht das selbst.
- In ROS 2 Jazzy brauchen `collision_monitor` und `docking_server` eine Konfiguration, auch wenn man sie nicht nutzt.

**So bindest du deine Variante ein**
```bash
git fetch origin && git checkout -b nav2 origin/pascal/pipeline-v2.4
# deine Parameter testen, ohne unseren Code zu ändern:
ros2 launch roboter_system system.launch.py mission:=testwelt navigation:=nav2 headless:=true nav2_params:=<deine.yaml>
```
Referenz zum Vergleichen: `src/roboter_navigation/config/nav2_params.yaml` (3D-Lidar) und `nav2_params_2d.yaml`.
Wenn `bash tests/alle_tests.sh` mit deiner Konfiguration grün ist, Pull Request auf `pascal/pipeline-v2.4`.

---

## Was sich umbenannt oder verschoben hat

Wenn du in deinem Stand nach einer Datei suchst, steht hier, wo sie jetzt liegt.

| Früher (`pancake_receipt`) | Jetzt | Warum |
|---|---|---|
| Paket `r1_bringup` | `roboter_simulation` | Name passte nicht mehr (R1/Gazebo → G1/MuJoCo) |
| `r1_bringup/models/robots/g1_description/` | `roboter_beschreibung/modelle/g1_description/` | Das Robotermodell braucht nicht nur die Simulation |
| `r1_bringup/config/g1_policy.yaml` (12 DOF) | `roboter_steuerung/config/policy_29dof.yaml` | Die Policy gehört zur Steuerung; 12 DOF ist entfernt (Tag `v1.9-review-abgeschlossen`) |
| `r1_bringup/models/policies/motion.pt` | `roboter_steuerung/modelle/policy_29dof.onnx` | neue Policy (29 DOF, ONNX) |
| `r1_bringup/bridges/` | `roboter_steuerung/hardware/` | Die Steuerung legt fest, was sie von der Hardware braucht |
| `roboter_steuerung/launch/steuereinheit.launch.py` | `roboter_steuerung/launch/steuerung.launch.py` + `roboter_system/launch/system.launch.py` | Jede Schicht startet nur sich selbst |
| Gazebo-Launches, `r1_welt.sdf`, 12-DOF-Teile | entfernt | nicht mehr gebaut/getestet; vollständig unter Tag `v1.9-review-abgeschlossen` |

**Ganz neue Pakete**

| Paket | Inhalt |
|---|---|
| `roboter_navigation` | Nav2-Konfiguration – **dein Bereich** |
| `roboter_sensorik` | macht aus der 3D-Punktwolke `/scan` und `/punkte_hindernis` |
| `roboter_welten` | Umgebungen: je Welt eine Szene (Simulation) und eine Ortsliste (Missionen) |
| `roboter_beschreibung` | G1-Modell und Geometrie (Becken- und Lidar-Höhe) |
| `roboter_system` | Gesamtstart und RViz-Ansicht |

---

## Was sich am Verhalten geändert hat

- **12 → 29 Gelenke.** Die alte Policy kippte; die Ganzkörper-Policy von Unitree steht und geht stabil.
- **Kein fester Startbefehl mehr.** Der Roboter steht nach dem Start still, bis eine Freigabe kommt.
- **NotAus** setzt sofort alles auf Stillstand. Danach ist eine neue Freigabe nötig.
- **Die Simulation startet erst**, wenn die Steuerung ihr Verstärkungen und Startpose geschickt hat (`/antrieb/parameter`).
  Vorher las die Simulation dafür eine Datei der Steuerung.
- **Missionen enthalten nur noch den Auftrag.** Die Orte stehen bei der jeweiligen Welt, damit mehrere Missionen in derselben Halle dieselben Koordinaten nutzen.

---

## Starten und Ausprobieren

```bash
cd ~/humanoid_robot && source /opt/ros/jazzy/setup.bash
colcon build && source install/setup.bash

# Roboter allein, mit Fenster, Steuerung von Hand
ros2 launch roboter_system system.launch.py welt:=testwelt
ros2 topic pub --once /freigabe std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}"

# Mission mit Nav2 (wegen RAM ohne Simulationsfenster), mit RViz
ros2 launch roboter_system system.launch.py mission:=testwelt navigation:=nav2 headless:=true rviz:=true auto_start:=true

# alle Tests
bash tests/alle_tests.sh
```
Voraussetzungen: ROS 2 Jazzy mit `navigation2`, `nav2-bringup`, `slam-toolbox`; Python-Pakete aus
`requirements.txt` (`mujoco`, `onnxruntime`); Fast Downward unter `~/tools/downward`
(Anleitung in `src/roboter_task_planning/README.md`).

---

## Was noch nicht geht

- **Echte Hardware:** Es fehlt die Anbindung an das Unitree-SDK. Die Schnittstelle dafür ist vorbereitet.
- **Reale Orte:** Orte sind fest als Koordinaten hinterlegt. Gespeicherte Karte, Lokalisierung und Teach-in fehlen.
- **Greifen:** gemessen, dass es im Stehen grundsätzlich geht (0,30 m Reichweite, ≤ 1 kg je Hand), aber nicht umgesetzt.
- **Akku/Laden:** bewusst weggelassen, solange es keine echten Batteriedaten gibt.
- **Kamera / Objekterkennung:** keine.
- **SysML-Modell:** liegt noch nicht im Repo, daher keine Zuordnung Anforderung → Test.

---

## Wo steht was

| Dokument | Wofür |
|---|---|
| `README.md` | Start, alle Schalter, Tests |
| `docs/architektur.md` | Pakete, Schnittstellen, wer worauf zugreifen darf |
| `src/roboter_navigation/README.md` | Nav2-Platzhalter, Lidar-Varianten |
| `tests/README.md` | welche Tests es gibt und wie man sie startet |
| `docs/statusupdate_g1_projekt.md` | ausführlicher Verlauf mit allen Messwerten |

Versionen: `git tag` zeigt alle Zwischenstände (`v1.0` erste Pipeline … `v2.4` aktueller Stand).
