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

## Mission (`missions/*.yaml`)
Eine Mission ist nur der **Auftrag**: `welt` (Name einer Welt aus `roboter_welten`), `start_ort`
(optional, sonst nächster Ort zur Position), `ziele`, `parameter` (`warte_s`,
`balancier_toleranz_grad`, `max_replans`; `nav_geschwindigkeit` wirkt nur im GehAktion-Backend/Testdouble –
mit Nav2 gelten dessen Parameter; unbekannte Parameter werden abgelehnt).
Die **Orte** (x/y in m, Frame `map`) und **Verbindungen** (ungerichtet; fehlt = vollvernetzt) stehen in
`roboter_welten/welten/<welt>/orte.yaml`, weil sie zur Umgebung gehören: mehrere Missionen in derselben Halle
nutzen dieselben Orte, und `system.launch.py` (Paket `roboter_system`) startet die Simulation mit derselben Welt.
Ortsnamen: ASCII, Buchstaben/Ziffern/`_`; werden beim Laden kleingeschrieben, `pos_aktuell` ist reserviert.

```yaml
welt: testwelt
start_ort: start
ziele:
  - {art: besucht,    ort: a}
  - {art: balanciert, ort: b}
parameter: {warte_s: 2.0, max_replans: 2}
```

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
# Gesamtsystem mit Mission (Layer 1-4), Testdouble statt Nav2:
ros2 launch roboter_system system.launch.py mission:=beispiel navigation:=testdouble
# mit Nav2 in der Testwelt (Welt kommt aus der Mission):
ros2 launch roboter_system system.launch.py mission:=testwelt navigation:=nav2 headless:=true
ros2 service call /mission/start std_srvs/srv/Trigger     # Mission starten (oder auto_start:=true)
ros2 topic echo /mission/status                           # JSON-Status
ros2 service call /mission/abort std_srvs/srv/Trigger
```
`roboter_task_planning/launch/mission.launch.py` startet nur den Missionsknoten (Layer 1).
`navigation:=testdouble` → `nav_backend=gehaktion` (nur Test!), `navigation:=nav2` → `nav_backend=nav2`;
Navigation des Kollegen: `navigation:=nav2 nav2_params:=<datei>`, oder sein Nav2 selbst starten und unser System
mit `navigation:=extern` (Lidar, Sensorik und Watchdog laufen dann wie mit eigenem Nav2). Beim Missionsstart gibt der Knoten die Bewegung über
`/freigabe` frei.

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
python3 tests/system/integration_mission.py                                  # Simulation, Nav2-Testdouble
```
`nav_testdouble_node` ist ein TESTDOUBLE (ausrichten + geradeaus), keine Navigation.
