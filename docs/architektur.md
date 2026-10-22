# Architektur G1-Projekt

Stand 21.10.2026 (v2.7, nach Review 2). Einstieg für Übergaben; Bedienung: `README.md`, Tests: `tests/README.md`.

## Schichten und Pakete

| Schicht | Paket | Inhalt | Verantwortlich |
|---|---|---|---|
| Layer 1 Deliberative | `roboter_task_planning` | Mission (Auftrag) → PDDL → Fast Downward → Dispatcher; `mission_node`; Testdouble für Layer 2 (`testing/`) | wir |
| Layer 2 Navigation | `roboter_navigation` | Nav2-Parameter (3D/2D), SLAM-Parameter, `navigation.launch.py` – **Platzhalter** | Kollege (ersetzbar) |
| Sensorik | `roboter_sensorik` | `punktwolke_zu_scan`: `/points` → `/scan` + `/punkte_hindernis` | wir |
| Layer 3 Reactive | `roboter_steuerung` | `steuereinheit_node` (Zustandsautomat `states/`, Policy `control/`, Befehlsaufbereitung `kommando.py`, Haltung `haltung.py`), Hardware-Schnittstelle `hardware/`, `balance_dienst_node`, `beinodometrie_node` (`zustandsschaetzung/`), Policy `config/policy_29dof.yaml` + `modelle/policy_29dof.onnx` | wir |
| Layer 4 HAL | `roboter_simulation` | `mujoco_node` mit `szene.py`, `sensormodell.py`, `lidar.py`, `aufnahme.py`, `geometrie.py` | wir |
| Roboter | `roboter_beschreibung` | Unitree-G1-Modelle (bis auf `armature` an den Gelenken unverändert), `config/roboter.yaml` (Modell, URDF, Beckenhöhe, Lidar-Montage), Quaternion-Hilfen, TF-Baum (`launch/beschreibung.launch.py`) | – |
| Umgebung | `roboter_welten` | je Welt `szene.xml` (Simulation) + `orte.yaml` (Missionen) + `karte.yaml/.pgm` (Nav2-Karte, aus der Szene gerechnet) | wir |
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
| `/joint_states` (mit Namen), `/imu` | Topic | L4 → L3, Beinodometrie, Sensorik, `robot_state_publisher` | 100 Hz; L3 rechnet bei jeder 2. Gelenknachricht (Takt aus den Sensordaten) |
| `/clock` | Topic | L4 → Tests | Simulationszeit (die Knoten nutzen die Wanduhr) |
| `/joint_commands` | Topic | L3 → L4 | Zielwinkel, 50 Hz |
| `/antrieb/parameter` | Topic (latched) | L3 → L4 | kp, kd, Ruhepose; L4 startet erst danach |
| `/odom` + TF `odom→base_link` | Topic/TF | L4 (Ground-Truth) **oder** `beinodometrie_node` | Schalter `odom_quelle` |
| `/points` | Topic | L4 → Sensorik | 3D-Lidar, 5 Hz |
| `/scan`, `/punkte_hindernis` | Topic | Sensorik → L2 | SLAM/Collision-Monitor bzw. Costmaps |
| `/cmd_vel` | Topic | L2 (oder Testdouble, Hand ohne Navigation) → L3 | Wunsch [vx, vy, wz], L3 begrenzt |
| `/cmd_vel/balance` | Topic | Balance-Dienst → L3 | Vorrang vor `/cmd_vel`, solange er sendet (0,5 s) |
| `/cmd_vel/hand` | Topic | Bediener → L3 | höchster Vorrang (vor Balance und Nav2), solange er sendet (0,5 s) |
| `/steuerung/zustand` | Topic (latched) | L3 → L1, Bediener | Start/Bereitschaft/Gehen/Balancieren/NotAus/Stoerung |
| `/steuerung/quittieren` | Service (Trigger) | Bediener → L3 | Stoerung bewusst verlassen |
| `/freigabe` | Topic | L1 oder Bediener → L3 | eigener Kanal, während NotAus abgelehnt |
| `/not_aus` | Topic (latched) | Bediener → L3, L1 | unterbricht jeden Zustand |
| `navigate_to_pose` | Action (Nav2) | L1 → L2 | Standard-Backend |
| `/geh_aktion` | Action `GehAktion` | L1 → Testdouble | nur Tests |
| `/balanciere` | Action `BalancierAktion` | L1 → L3 (`balance_dienst_node`) | |
| `/gleichgewicht` | Service | → L3 | Neigung und Stabilität |
| `/mission/start`, `/mission/abort`, `/mission/status` | Service/Topic | Bediener ↔ L1 | Status als JSON |
| `/mission/starte` | Service `MissionStarten` | Bediener → L1 | Mission nach Name oder Pfad starten |
| `compute_path_to_pose` | Action (Nav2) | L1 → L2 | Wegkosten für die Planung |
| `/base_position` | Topic | L4 → Tests | nur Simulation (Ground-Truth) |

## Sicherheit

| Fall | Wer erkennt es | Reaktion |
|---|---|---|
| NotAus (`/not_aus`) | Layer 3 | Zustand NotAus: Policy aktiv mit Befehl 0 (aktiv geregelter Stillstand), Freigabe weg, Mission bricht ab |
| Sensordaten bleiben aus (> `sensor_timeout_s`, 1 s) | Layer 3 | Zustand **Stoerung**: keine Zielwinkel mehr |
| Policy liefert NaN/inf | Layer 3 | Stoerung (der Wert wird nicht gesendet) |
| Sturz (Neigung > `sturz_neigung_grad`, 45°) | Layer 3 | Stoerung |
| Zielwinkel ausserhalb der Gelenkgrenzen | Layer 3 | auf die Grenzen des Robotermodells begrenzt |
| Zielwinkel springt > `max_zielsprung_rad` (0,6 rad) in einem Takt | Layer 3 | Sprung begrenzt, Warnung (normal: ≤ 0,32 rad) |
| Kommandos bleiben aus (> `kommando_timeout_s`, 1 s) | Layer 4 (Simulation; auf Hardware die Unitree-Firmware) | **Daempfungsmodus**: Steifigkeit 0, nur Daempfung |
| Stoerung | Layer 1 | Mission bricht ab, Neustart erst nach `/steuerung/quittieren` |
| `/cmd_vel` bleibt aus (> 1 s mit Navigation) | Layer 3 | Watchdog setzt den Wunsch auf 0 |
| Missionsende | Layer 1 | Freigabe wird entzogen |

Zustand der Steuerung: `/steuerung/zustand` (latched). Stoerung verlassen nur bewusst: `ros2 service call
/steuerung/quittieren std_srvs/srv/Trigger` (klappt nur mit Sensordaten, aufrecht, ohne NotAus).

**Anforderungen an die Hardware-Anbindung (noch nicht umgesetzt; Bestellung und zu ermittelnde Werte:
`docs/g1_hardware_checkliste.md`):**
- Der Software-NotAus ersetzt **keinen** physischen Not-Halt. Auf dem echten G1 braucht es den Not-Halt der Unitree-
  Fernbedienung bzw. einen verdrahteten Not-Halt, der unabhaengig von dieser Software wirkt.
- Die Unitree-SDK-Bridge muss bei ausbleibenden Kommandos selbst daempfen (Firmware-Daempfungsmodus) und kp/kd aus
  `/antrieb/parameter` mit jedem Gelenkkommando senden.
- `auto_start` ist nur fuer Simulation und Tests gedacht; ohne Simulation (kein `/clock`) wird er abgelehnt.

## TF-Baum

```
map ──(statisch | SLAM | AMCL auf fester Karte)── odom ──(Simulation oder Beinodometrie)── base_link
base_link ── pelvis ──(robot_state_publisher aus /joint_states + URDF)── alle Glieder (torso_link, head_link, ...)
<Elternglied aus roboter.yaml, Standard pelvis> ── lidar_link
```
Die Lidar-Montage steht nur in `roboter.yaml` (Elternglied + Pose); Simulation, TF-Baum und Sensorik lesen sie von dort
bzw. aus dem TF-Baum. Am echten G1 sitzt der MID-360 im Kopf: Elternglied `torso_link`, Pose aus dem Unitree-Datenblatt.

## Karte und Lokalisierung (Layer 2)

| Modus | `map → odom` | globale Planung | wann |
|---|---|---|---|
| ohne Karte (Standard) | statisch (Startpose = Kartenursprung) | nur im 24-m-Fenster der aktuellen Lidardaten | Simulation, schnelle Tests |
| `slam:=true` | slam_toolbox (Kartierung) | auf der entstehenden SLAM-Karte (static_layer mit Aktualisierungen) | Karte aufnehmen |
| `karte:=true` | AMCL auf `welten/<welt>/karte.yaml`, Startpose über `startpose:=x,y,gier` | auf der ganzen Karte (static_layer) | feste Halle: Orte bleiben gültig |
| `karte:=<pfad>.yaml` | AMCL auf einer selbst aufgenommenen Karte | wie `karte:=true` | wie in echt: vorher mit SLAM abgefahren |

In allen Modi bleibt das Lidar nötig: AMCL lokalisiert über den Abgleich der Scans mit der Karte, und die Hinderniskarten
von Nav2 tragen laufend ein, was nicht auf der Karte steht (Paletten, Menschen, Türen).
Die Karten der Simulationswelten sind aus der Szene gerechnet (perfekt, ohne Messfehler). Den echten Ablauf prüft
`integration_mission.py --aufnehmen`: Welt mit SLAM abfahren, mit `map_saver_cli` speichern, neu starten und die Mission
nur mit dieser Karte fahren (21.10.2026: 34/34). Für die echte Halle genauso, die Karte dann als `karte.yaml/.pgm` der Welt.

Layer 1 fragt die Wegkosten zwischen den Orten beim Nav2-Planer ab (`compute_path_to_pose`); ohne Planer gilt die Luftlinie.

## Ablauf beim Start

1. `steuereinheit_node` lädt die Policy und sendet `/antrieb/parameter` (latched).
2. `mujoco_node` übernimmt kp/kd/Ruhepose, stellt den Roboter hin und sendet Sensordaten; die Physik pausiert.
3. Mit dem ersten `/joint_commands` läuft die Physik in Echtzeit; der Roboter steht in **Bereitschaft**.
4. Erst `/freigabe` (Bediener oder Missionsstart) erlaubt Bewegung.

## Bewusste Entscheidungen (Review 21.10.2026)

| Frage | Entscheidung | Begründung |
|---|---|---|
| PDDL/Fast Downward für „Orte besuchen, stehen, warten“ zu schwer? | **bleibt** | Planung bleibt erweiterbar (Manipulation, Ressourcen, Zeitfenster) ohne neuen Planer; Plan ist lesbar und optimal. Aufwand der externen Installation: `scripts/einrichten.sh`; CI überspringt die Planer-Tests ohne Fast Downward |
| Ortsgraph (`verbindungen`) doppelt zu Nav2? | **bleibt, mit klarer Rolle** | Der Graph legt erlaubte Wege und Pflicht-Durchgänge fest (z. B. „nur durch die Tür“); wie gefahren wird, plant Nav2. Kosten kommen vom Nav2-Planer, nicht mehr aus der Luftlinie. Ohne `verbindungen` ist jeder Ort mit jedem verbunden |
| Zustand/Aktion „Balancieren“ balanciert nicht selbst | **Name bleibt** (SysML) | Bedeutung: anhalten und sicheres Stehen bestätigen; das Gleichgewicht hält immer die Policy. So steht es in den Docstrings (`balancieren_state.py`, `balance_dienst_node.py`) |
| `GehAktion` ohne produktiven Nutzer | **bleibt als Test-Schnittstelle** | SysML-Port; nur das Testdouble (`navigation:=testdouble`) bedient sie. Produktiv navigiert Layer 1 über Nav2 (`navigate_to_pose`) |
| Boden/Licht im Robotermodell statt in der Welt | **bleibt** | Stehen so in Unitrees Original (`unitree_rl_gym`); die Datei soll nah am Original bleiben (einzige Änderung `armature`). Welten ergänzen nur Hindernisse |
| `auto_start` + Freigabe durch die Mission | **nur Simulation/Tests** | ohne Simulation (kein `/clock`-Sender) lehnt der Missionsknoten `auto_start` ab; auf Hardware startet ein Bediener die Mission (Dienst), am Missionsende wird die Freigabe entzogen |
| NotAus in Software | **ersetzt keinen physischen Not-Halt** | Anforderung an die Hardware-Anbindung (Abschnitt Sicherheit) |
| Steuerung (Layer 3) ohne MuJoCo | **Gelenkgrenzen aus der Modelldatei gelesen**; Beinodometrie lädt das Modell ohne Netze | Das volle Modell mit allen Netzen belegt ~410 MB je Prozess; unter WSL2 (2,8 GB) lagerte das System aus und der Roboter stürzte beim Anhalten (Verlauf, 21.10.2026). Auf Hardware braucht die Steuerung keine Physik-Engine |

## Modell und Rückverfolgbarkeit

Die SysML-Modelle (`RoboterZustaende.sysml`, `RoboterVerhalten.sysml`, `RoboterKnoten.sysml`) liegen noch nicht im
Repo; vorgesehener Ort ist `modell/` mit einer Tabelle Anforderung → Umsetzung → Test (offen, Review Kategorie 5).
