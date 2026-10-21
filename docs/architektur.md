# Architektur G1-Projekt

Stand 14.10.2026 (Strukturumbau v2.x). Einstieg für Übergaben; Bedienung: `README.md`, Tests: `tests/README.md`.

## Schichten und Pakete

| Schicht | Paket | Inhalt | Verantwortlich |
|---|---|---|---|
| Layer 1 Deliberative | `roboter_task_planning` | Mission (Auftrag) → PDDL → Fast Downward → Dispatcher; `mission_node`; Testdouble für Layer 2 (`testing/`) | wir |
| Layer 2 Navigation | `roboter_navigation` | Nav2-Parameter (3D/2D), SLAM-Parameter, `navigation.launch.py` – **Platzhalter** | Kollege (ersetzbar) |
| Sensorik | `roboter_sensorik` | `punktwolke_zu_scan`: `/points` → `/scan` + `/punkte_hindernis` | wir |
| Layer 3 Reactive | `roboter_steuerung` | `steuereinheit_node` (Zustandsautomat `states/`, Policy `control/`, Befehlsaufbereitung `kommando.py`, Haltung `haltung.py`), Hardware-Schnittstelle `hardware/`, `balance_dienst_node`, `beinodometrie_node` (`zustandsschaetzung/`), Policy `config/policy_29dof.yaml` + `modelle/policy_29dof.onnx` | wir |
| Layer 4 HAL | `roboter_simulation` | `mujoco_node` mit `szene.py`, `sensormodell.py`, `lidar.py`, `aufnahme.py`, `geometrie.py` | wir |
| Roboter | `roboter_beschreibung` | Unitree-G1-Modelle (unverändert) + `config/roboter.yaml` (Modell, Becken- und Lidar-Höhe) | – |
| Umgebung | `roboter_welten` | je Welt `szene.xml` (Simulation) + `orte.yaml` (Missionen) | wir |
| Schnittstellen | `roboter_interfaces` | `GehAktion`, `BalancierAktion`, `GleichgewichtService`, `Antriebsparameter` | – |
| Gesamt | `roboter_system` | `system.launch.py` (setzt die Schichten zusammen), RViz-Ansicht | wir |

## Abhängigkeitsregeln

- Höhere Schichten kennen nur die **Schnittstellen** der tieferen (Topics, Actions, Services), keinen Code.
- Layer 3 hängt von **keiner** Simulation ab: die Schnittstelle `IRobotHardwareBridge` liegt in Layer 3,
  `TopicBridge` spricht Topics. Für echte Hardware ersetzt eine Unitree-SDK-Bridge (oder ein Knoten mit denselben
  Topics) die Simulation, ohne Layer 3 anzufassen.
- Was zur trainierten Policy gehört (kp, kd, Ruhepose, Takt, Kommandobereich), steht **nur** in Layer 3. Die Simulation
  bekommt kp/kd/Ruhepose zur Laufzeit (`/antrieb/parameter`), wie ein Motortreiber.
- Robotergeometrie steht **nur** in `roboter_beschreibung`, Orte **nur** in `roboter_welten`.
- Jede Schicht startet sich selbst (Launch-Datei im eigenen Paket); nur `roboter_system` kennt alle.
- Laufzeitparameter je Schicht in **einer** YAML-Datei, die die Launch-Datei immer lädt
  (`roboter_simulation/config/simulation.yaml`, `roboter_steuerung/config/steuerung.yaml`, Nav2: `roboter_navigation/config/`).
  `tests/simulation/test_konfiguration.py` prüft, dass dort nur Parameter stehen, die der Knoten kennt.

## Schnittstellen

| Name | Typ | Von → Zu | Bemerkung |
|---|---|---|---|
| `/joint_states`, `/imu` | Topic | L4 → L3, Beinodometrie, Sensorik | 100 Hz |
| `/joint_commands` | Topic | L3 → L4 | Zielwinkel, 50 Hz |
| `/antrieb/parameter` | Topic (latched) | L3 → L4 | kp, kd, Ruhepose; L4 startet erst danach |
| `/odom` + TF `odom→base_link` | Topic/TF | L4 (Ground-Truth) **oder** `beinodometrie_node` | Schalter `odom_quelle` |
| `/points` | Topic | L4 → Sensorik | 3D-Lidar, 5 Hz |
| `/scan`, `/punkte_hindernis` | Topic | Sensorik → L2 | SLAM/Collision-Monitor bzw. Costmaps |
| `/cmd_vel` | Topic | L2 (oder Hand/Testdouble) → L3 | Wunsch [vx, vy, wz], L3 begrenzt |
| `/freigabe` | Topic | L1 oder Bediener → L3 | eigener Kanal, während NotAus abgelehnt |
| `/not_aus` | Topic (latched) | Bediener → L3, L1 | unterbricht jeden Zustand |
| `navigate_to_pose` | Action (Nav2) | L1 → L2 | Standard-Backend |
| `/geh_aktion` | Action `GehAktion` | L1 → Testdouble | nur Tests |
| `/balanciere` | Action `BalancierAktion` | L1 → L3 (`balance_dienst_node`) | |
| `/gleichgewicht` | Service | → L3 | Neigung und Stabilität |
| `/mission/start`, `/mission/abort`, `/mission/status` | Service/Topic | Bediener ↔ L1 | Status als JSON |
| `/base_position` | Topic | L4 → Tests | nur Simulation (Ground-Truth) |

## Ablauf beim Start

1. `steuereinheit_node` lädt die Policy und sendet `/antrieb/parameter` (latched).
2. `mujoco_node` übernimmt kp/kd/Ruhepose, stellt den Roboter hin und sendet Sensordaten; die Physik pausiert.
3. Mit dem ersten `/joint_commands` läuft die Physik in Echtzeit; der Roboter steht in **Bereitschaft**.
4. Erst `/freigabe` (Bediener oder Missionsstart) erlaubt Bewegung.

## Modell und Rückverfolgbarkeit

Die SysML-Modelle (`RoboterZustaende.sysml`, `RoboterVerhalten.sysml`, `RoboterKnoten.sysml`) liegen noch nicht im
Repo; vorgesehener Ort ist `modell/` mit einer Tabelle Anforderung → Umsetzung → Test (offen, Review Kategorie 5).
