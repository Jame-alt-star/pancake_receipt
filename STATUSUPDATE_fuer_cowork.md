# Statusupdate G1-Humanoid-Projekt für Cowork — Stand 13.10.2026 (abends, nach Nav2-Anbindung)

Dieses Dokument ersetzt den Kontext aus `UEBERGABE_fuer_claude_code_wsl2.md` (damals offene Aufgabe:
29-DOF-Policy fällt im echten ROS2-Lauf um). Diese Aufgabe ist **gelöst**. Detailverlauf mit allen
Messwerten: `statusupdate_g1_projekt.md` (im Projekt-Root, laufend aktualisiert). Git: Branch `main`,
letzter Commit `470615e`.

## Kurzfassung
- Das Projekt ist ein **Arbeitsprojekt, keine Thesis**.
- Es wird **komplett auf 29 DOF** umgestellt; der 12-DOF-Pfad wird nicht mehr getestet.
- Layer 3 (Bewegung) und Layer 4 (Simulation) sind stabil und durch Tests abgesichert.
- Layer 1 (Missionsplanung) ist gebaut und in der Simulation getestet.
- Layer 2 (Navigation, Nav2) macht ein **Kollege**; Schnittstellen sind vorbereitet, aber nicht gemeinsam getestet.
- Greifen/Hände ist **bewusst zurückgestellt**; Machbarkeit im Stehen ist gemessen.

## Status je Schicht
| Schicht | Stand |
|---|---|
| Layer 4 Simulation/Bridge (`r1_bringup/mujoco_node.py`) | stabil, Echtzeit (~490 Hz); `/odom` + TF `odom→base_link` (Ground-Truth) neu; Physik pausiert bis der Regler verbunden ist |
| Layer 3 Bewegung (`roboter_steuerung`) | 29-DOF-Policy (ONNX) stabil: Stehen, Gehen, NotAus, Kurs- und Positionshaltung, cmd-Begrenzung; `balance_dienst_node` (Service `/gleichgewicht`, Action `/balanciere`) |
| Layer 2 Navigation | Kollege, Nav2. Hier ein **Platzhalter-Paket `roboter_navigation`** (generische Nav2-Konfiguration) läuft mit echtem Nav2 (installiert); Variante des Kollegen später per `params_file`/Paketaustausch. Daneben bleibt `nav_testdouble_node` (KEINE Navigation) für Tests |
| Layer 1 Mission (`roboter_task_planning`) | PDDL + Fast Downward + Dispatcher mit Neuplanung; Mission als YAML; Services `/mission/start`, `/mission/abort`, Topic `/mission/status` |
| Zustandsautomat | Start, Bereitschaft, Balancieren, Gehen, NotAus umgesetzt+getestet. **AkkuSchwach/Laden fehlen** (keine echten Batteriedaten; Platzhalter-Akku wurde abgelehnt) |

## Was in dieser Sitzung gefunden und behoben wurde (Ursachen)
Die Latenz-Hypothese der Übergabe war **falsch**. Tatsächliche Ursachen:
1. **Startrennen:** `steuereinheit_node` braucht ~7 s zum Hochfahren, `mujoco_node` simulierte schon mit reiner PD-Haltung (kippt nachweislich). Fix: Physik pausiert bis zum ersten `/joint_commands` (`wait_for_controller`).
2. **Kurshaltung war ein ungebremster Integrator:** `cmd[2]` wuchs im Stand auf −572 rad/s, Sturz direkt nach der Freigabe. Fix: eigener Rampen-Zustand `cmd_ramped`, Korrekturen akkumulieren nicht.
3. **Performance:** onnxruntime mit Defaults belegte alle Kerne (270 % CPU) → 1 Thread, kein Spin-Waiting; Publish nur alle 5 Schritte (100 Hz); absolute Zeitplanung statt Sleep pro Schritt. Sim lief vorher mit 0,35–0,55× Echtzeit.
4. **Stand-Drift:** entstand in Bereitschaft, weil dort keine Kurshaltung aktiv war. Fix: Kurshaltung auch in Bereitschaft. Ergebnis: 0,9 cm max. Abweichung über 120 s, Beine praktisch still (0,0004 rad/s).
5. Positionshaltung (nur Zustand Balancieren) bringt gegen Störungen etwas (Stoß 0,6 m/s: 6 cm statt 24 cm Endabweichung), ist für reines Stehen nicht nötig. Frühere Aussage „Positionshaltung behebt den Drift“ war irreführend und ist im Statusupdate korrigiert.

## Greifen im Stehen (zurückgestellt, Ergebnisse bleiben gültig)
Werkzeug `tests/arm_feasibility.py`. Kernergebnisse (synchroner Kreis, Last = Zusatzmasse je Hand):
- Unveränderte Policy trägt Arme nur bis ~0,20 m vor dem Becken. Ursache des Sturzes bei mehr: **Arm-Beobachtungen außerhalb des Trainingsbereichs**, nicht der Schwerpunkt.
- Policy „blind für die Arme“ (Ruhepose als Beobachtung) + Taille 0,15 rad nach hinten: stabil bei 0,30 m Reichweite bis 1 kg je Hand (5,6 cm Drift, 4° Neigung, 60 s Halten ohne Anwachsen). Mischung der Beobachtung (25–75 %) ist nicht robust.
- Offen: keine Hand im Modell (`g1_29dof_with_hand.xml` existiert), Einarm-Aufgaben, echte Objektkontakte, Wahrnehmung, IK nur als eigene MuJoCo-Least-Squares-Lösung. Geplanter Zustand `Manipulieren` ist **nicht gebaut**.
- Gelernte Policies: keine fertige Greif-Policy für G1 gefunden; Training/Feintuning bräuchte GPU und Daten.

## Tests (alle bestanden, Stand 13.10.2026)
| Test | Aufruf | Umfang |
|---|---|---|
| Synchroner Regelkreis | `cd tests && python3 -m pytest -q` | 11 + 7 + 4 Tests (Regelkreis, Knoten-Logik, Greifen), ~40 s, ohne ROS2 (Teile brauchen gesourcten Workspace) |
| Pipeline-Integration | `python3 tests/integration_ros.py` | 28 Prüfungen, ~3 min, 3 Läufe in Folge grün (Start, Freigabe, Stehen 30 s ohne Drift, Gehen, zu großes cmd, NotAus, Odometrie) |
| Missions-Integration | `python3 tests/integration_mission.py [--nav2]` | 14 Prüfungen, ~2–5 min (Mission start→a→b, NotAus bricht ab); `--nav2` mit echtem Nav2 (headless) |
| Layer-1-Kern | `python3 -m pytest src/roboter_task_planning/test/test_layer1.py` | 10 Tests, braucht Fast Downward |

Vorher `source /opt/ros/jazzy/setup.bash && source install/setup.bash`; nach Codeänderungen `colcon build` (Install ist eine Kopie, kein Symlink). Details: `tests/README.md`.

## Wichtige Dateien
- `src/r1_bringup/r1_bringup/mujoco_node.py` — Simulation, Publish, Odometrie, Zeitplanung, Physik-Pause
- `src/r1_bringup/config/g1_policy_29dof.yaml` — Gains, Skalierungen, `cmd_training_range`, `position_hold`
- `src/roboter_steuerung/roboter_steuerung/steuereinheit_node.py` — Regelschleife, Rampe, Kurs-/Positionshaltung, `_clip_cmd`
- `src/roboter_steuerung/roboter_steuerung/states/*.py` — Zustandsautomat
- `src/roboter_steuerung/roboter_steuerung/control/rl_locomotion_controller_29dof.py` — ONNX-Controller
- `src/roboter_steuerung/roboter_steuerung/balance_dienst_node.py` — Layer-3-Dienste für Layer 1
- `src/roboter_navigation/` — Layer 2 Platzhalter (Nav2-Parameter + Launch)
- `src/roboter_task_planning/` — Layer 1 (`README.md` dort beschreibt Ablauf, Fehlerverhalten, Start)
- `tests/` — Testsuite, `arm_feasibility.py`

## Für den Kollegen (Nav2 / Layer 2) — Schnittstelle
- Nav2 liefert `/cmd_vel` (`geometry_msgs/Twist`); Layer 3 begrenzt: **vx −0,5…1,0, vy ±0,3, wz ±1,0 rad/s**, und oberhalb vx 0,7 wird |wz| linear bis 0,5 bei vx 1,0 abgesenkt. Unter ~0,3 m/s läuft der Roboter nicht an. **Drehen auf der Stelle** funktioniert nur mit |wz| ≳ 0,8 (≈0,5 rad/s real); kleine wz werden im Stand fast ignoriert (0,2 → 0,025 rad/s), beim Gehen folgt die Policy gut (~75 %). Nav2 sollte daher `rotate_to_heading_angular_vel ≈ 1,0`, `min_vel_x ≥ 0,3`, `max_vel_theta ≤ 1,0` setzen und einen Progress-Checker nehmen, der Drehung zählt (`PoseProgressChecker`). Beschleunigung ist auf 1,0 m/s² gerampt. Referenz: `src/roboter_navigation/config/nav2_params.yaml`.
- `/odom` und TF `odom→base_link` kommen aus `mujoco_node` (perfekte Odometrie, Ground-Truth, Parameter zum Abschalten). `map→odom` liefert Layer 2.
- Layer 1 ruft Nav2 über `nav2_msgs/NavigateToPose` (`navigate_to_pose`, Frame `map`) auf (`nav_backend=nav2`) und wartet vorher auf `/lifecycle_manager_navigation/is_active`. **Mit der Platzhalter-Konfiguration getestet** (`tests/integration_mission.py --nav2`, 14/14); mit der Konfiguration des Kollegen noch nicht.
- In Jazzy gehören `collision_monitor` und `docking_server` zur festen Lifecycle-Kette von `navigation_launch.py` und brauchen Konfiguration; ohne Sensorik ist die Quelle deaktiviert. `navigation_launch.py` mit `use_composition=True` startet den Container nicht selbst.
- Ressourcen: WSL2 hat nur ~2,8 GB RAM. Nav2 + Simulation + Viewer reicht nicht; `headless:=true` (MuJoCo ohne Fenster) verwenden.
- `balanciere` publiziert `/cmd_vel = 0`; bei gleichzeitigem Nav2 ist ggf. ein `cmd_vel`-Multiplexer nötig.
- Auf echter Hardware gibt es keine Ground-Truth-Position (`/base_position`, `/odom`): dort braucht es Odometrie/Lokalisierung.

## Offene Punkte
1. Nav2-Variante des Kollegen einbinden (`params_file:=...`) und mit Grenzen oben prüfen; `cmd_vel`-Konflikt (Balancieren publiziert `/cmd_vel = 0`) und fehlender `cmd_vel`-Watchdog in Layer 3 klären.
2. Neuplanung nach Fehlschlag ist in der Simulation nicht abgedeckt (nur Unit-Tests; der Testdouble scheitert nie).
3. AkkuSchwach/Laden: Schnittstelle (`/battery_state`) erst mit echten Daten; Designentscheidungen sind im Statusupdate dokumentiert (Laden wird extern ausgelöst, AkkuSchwach = sicher stehen).
4. NotAus direkt in Start/Bereitschaft ist nicht gesondert getestet.
5. Greifen: Hand-Modell, `Manipulieren`-Zustand, Einarm, Objektkontakt, Wahrnehmung; nur nach Bedarf.
6. Aufräumen: `g1_29dof.xml.bak_vor_armature_fix` und `src/_backup_vor_lockstep_*` (nach Absprache), 12-DOF-Pfad bei der Umstellung entfernen; `onnxruntime`/`mujoco`/Fast Downward sind nicht über `package.xml` installierbar (README-Hinweise).
7. Echte Hardware: Sicherheitsrelais für NotAus, echte Batteriedaten, Lokalisierung statt Ground-Truth.

## Umgebung / Hinweise zur Arbeitsweise
- Fast Downward liegt **außerhalb des Repos** unter `~/tools/downward` (Release 24.06.1, Bauanleitung in `src/roboter_task_planning/README.md`).
- Beim Integrationstest keine parallelen `ros2 topic echo/hz`-Aufrufe (stören die Simulation). Persistente Skripte statt vieler CLI-Starts.
- Messbare Behauptungen belegen (Zahlen), „getestet“ von „angenommen“ trennen. Commits nur auf Wunsch; `MUJOCO_LOG.TXT` ist in `.gitignore`.
- Sprache der Kommentare/Doku: Deutsch.
