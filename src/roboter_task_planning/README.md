# roboter_task_planning (Layer 1: Missionsplanung)

Mission (YAML) → PDDL-Problem → **Fast Downward** → Plan → Dispatcher → Bridges
(Layer 2 Navigation / Layer 3 Balancieren). Neuplanung bei Fehlschlag, Abbruch bei NotAus.

## Aktionen (PDDL-Domäne `pddl/domain.pddl`)
| Aktion | Wirkung | Ausführung |
|---|---|---|
| `gehe_zu(von, nach)` | Ort wechseln, Kosten = Strecke in cm | `NavigationBridge`: `nav2` (`nav2_msgs/NavigateToPose`, Layer 2 des Kollegen) oder `gehaktion` (`roboter_interfaces/GehAktion`) |
| `balanciere(ort)` | sicher stehen | `BalanceBridge` → `BalancierAktion` (`balance_dienst_node`, Layer 3) |
| `warte(ort)` | `warte_s` Sekunden warten | `WarteBridge` (intern) |

Ziele in der Mission: `besucht`, `balanciert`, `gewartet` je Ort. Planung kostenoptimal
(`astar(lmcut())`).

## Mission (`missions/beispiel.yaml`)
`orte` (x/y in m, Frame `map`), `verbindungen` (ungerichtet; fehlt = vollvernetzt), `start_ort`
(optional, sonst nächster Ort zur Position), `ziele`, `parameter` (`warte_s`,
`balancier_toleranz_grad`, `max_replans`; `nav_geschwindigkeit` und `nav_toleranz_m` wirken nur im
GehAktion-Backend/Testdouble – mit Nav2 gelten dessen Parameter).
Ortsnamen: ASCII, Buchstaben/Ziffern/`_`; werden beim Laden kleingeschrieben, `pos_aktuell` ist reserviert.

## Ablauf bei Fehlern
- Aktion scheitert → Roboterposition per TF `map→base_link` (Rückfall `/odom`), dynamischen Ort
  `pos_aktuell` einfügen – nur mit **ausgehenden** Kanten zu den 2 nächsten Orten und zum Ziel des
  gescheiterten Schritts (keine Abkürzung durch ihn) –, neu planen; höchstens `max_replans`-mal.
- Unerwartete Fehler (Ausnahmen) beenden die Mission mit `fehlgeschlagen` und Endstatus.
- Ohne `start_ort` und ohne Position wird die Mission abgelehnt.
- `/not_aus` = true → laufende Aktion abbrechen, Mission `abgebrochen`, **keine** Neuplanung;
  Start bei aktivem NotAus wird abgelehnt.

## Starten
```bash
ros2 launch roboter_task_planning mission.launch.py config_file:=g1_policy_29dof.yaml \
     nav_testdouble:=true            # Nav2 durch Testdouble ersetzen (nur Test!)
ros2 service call /mission/start std_srvs/srv/Trigger     # Mission starten
ros2 topic echo /mission/status                           # JSON-Status
ros2 service call /mission/abort std_srvs/srv/Trigger
```
Produktiv: `nav2:=true` → `nav_backend=nav2`, Navigation liefert Nav2 (Layer 2); getestet mit der
Platzhalter-Konfiguration aus `roboter_navigation` (`tests/integration_mission.py --welt/--slam`).
Beim Missionsstart gibt der Knoten die Bewegung über `/freigabe` frei.

## Voraussetzungen
- **Fast Downward** (nicht über rosdep verfügbar), gebaut außerhalb des Repos:
  ```bash
  git clone --depth 1 --branch release-24.06.1 https://github.com/aibasel/downward.git ~/tools/downward
  cd ~/tools/downward && python3 build.py release
  ```
  Gefunden über `$FAST_DOWNWARD`, `~/tools/downward/fast-downward.py` oder `PATH`.
- Layer 3/4 (`steuereinheit_node`, `mujoco_node`), `balance_dienst_node`.

## Nav2 (Layer 2): Schnittstelle zu uns
Nav2 liefert `/cmd_vel` (`geometry_msgs/Twist`) an Layer 3. Layer 3 begrenzt: vx −0,5…1,0, vy ±0,3,
**wz ±1,0 rad/s** (oberhalb vx 0,7 linear bis 0,5 abgesenkt); unter ~0,3 m/s läuft der Roboter nicht
an, kleine wz setzt er im Stand kaum um (Drehen auf der Stelle braucht |wz| ≳ 0,8). `mujoco_node` publiziert `/odom` (`nav_msgs/Odometry`, Frame `odom`, Child
`base_link`, Twist im Körper-Frame) und TF `odom→base_link` als **Ground-Truth** (perfekte
Odometrie, kein Drift). Parameter `publish_odom`, `publish_tf`, `odom_frame`, `base_frame`
(abschaltbar; mit `odom_quelle:=beinodometrie` kommt `/odom` vom Schätzer). `map→odom` liefert
Layer 2 (statisch oder SLAM).

## Tests
```bash
python3 -m pytest src/roboter_task_planning/test/test_layer1.py       # Kern, braucht Fast Downward
python3 tests/integration_mission.py                                  # Simulation, Nav2-Testdouble
```
`nav_testdouble_node` ist ein TESTDOUBLE (ausrichten + geradeaus), keine Navigation.
