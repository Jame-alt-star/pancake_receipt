# Tests

Alle Aufrufe aus dem Projektordner (dort, wo `src/` und `tests/` liegen), Workspace gesourct
(`source /opt/ros/jazzy/setup.bash && source install/setup.bash`). Nach Codeänderungen erst `colcon build`
(Install-Ordner ist eine Kopie).

**Komplette Serie:** `bash tests/alle_tests.sh` (Build, pytest, alle Systemtests nacheinander, Ergebnistabelle;
`--schnell` nur Build + pytest, `--mit-slam` zusätzlich die SLAM-Mission und „Karte aufnehmen + damit arbeiten“,
`--video DIR` zeichnet jeden Systemtest auf; daraus macht `demo/video_tests.py DIR video.mp4` ein Video mit einem
Kapitel je Test).

## Wo liegt was
| Ort | Art | Inhalt |
|---|---|---|
| `src/roboter_steuerung/test/` | Unit (Layer 3) | `test_kommando_haltung.py`: Rampe, Begrenzung, Kombigrenze, Watchdog, Kurshaltung akkumuliert nicht (Regression −572 rad/s), Positionshaltung; `test_steuereinheit_logik.py`: Knoten mit NotAus/Freigabe, Watchdog, Befehl ohne Freigabe, Stoerung bei Sensorausfall/Sturz/NaN, eigene Pause ist kein Ausfall, Quittieren, Gelenkgrenzen, Sprunglimit, Vorrang Hand > Balance > `/cmd_vel` |
| `src/roboter_sensorik/test/` | Unit (Sensorvorverarbeitung) | `test_wolke.py` (reine Rechenfunktionen aus `wolke.py`): Boden raus, niedrige Hindernisse drin, Neigungsausgleich, Ausdünnung |
| `src/roboter_task_planning/test/` | Unit (Layer 1) | `test_layer1.py`: Weltwissen, PDDL, Planer, Dispatcher, Neuplanung (braucht Fast Downward), `auto_start` nur mit Simulation |
| `tests/simulation/` | Simulation + Controller ohne ROS | `test_closed_loop.py` (Stehen, Gehen, Kurshaltung, Robustheit gegen Verzögerung/Jitter, Welten laden), `test_beinodometrie.py` (< 2 % der Strecke; schlankes Kinematikmodell ohne Netze = gleiche Posen), `test_konfiguration.py` (Parameterdateien nur mit deklarierten Parametern, Policy passt zum Modell, Takt, Gelenkgrenzen ohne MuJoCo = kompiliertes Modell), `test_sim_bausteine.py` (Lidar, Sensormodell, Posenaufnahme), Helfer `sim_helpers.py` |
| `tests/system/` | System (`ros2 launch`) | `integration_ros.py` (Pipeline: Startrennen, Freigabe, Gehen, Begrenzung, NotAus, Vorrang der Hand, Sprunglimit greift nie, Odometrie; zum Schluss Ausfall der Steuerung -> Daempfungsmodus), `integration_watchdog.py` (Watchdog; zum Schluss Sensorausfall -> Stoerung), `integration_mission.py` (Layer 1–4, Freigabe-Entzug am Ende; `--aufnehmen`: wie in echt erst mit SLAM abfahren und die Karte speichern, dann neu starten und nur damit arbeiten), `testaufnahme.py` (Aufzeichnung fürs Testvideo), `stop_all.sh` |

`python3 -m pytest -q` sammelt alle schnellen Tests (siehe `pytest.ini`).

## Systemtests einzeln
| Aufruf | Dauer |
|---|---|
| `python3 tests/system/integration_ros.py [--realistisch] [--beinodometrie] [--headless]` | ~2,5 min |
| `python3 tests/system/integration_watchdog.py` | ~1 min |
| `python3 tests/system/integration_mission.py [--nav2 \| --welt \| --patrouille \| --drei-d \| --nur-2d \| --slam \| --karte \| --aufnehmen \| --kueche \| --robocasa] [--realistisch] [--beinodometrie]` | 2–6 min (`--aufnehmen` ~10 min) |

Hinweise:
- Die Systemtests starten und beenden die Pipeline selbst; währenddessen keine eigenen `ros2 topic echo/hz`-Aufrufe.
- Strenger Stillstand bei NotAus (Beine < 0,05 rad/s, Drift < 5 cm).
- `--kueche`: Mission in der Welt `libero_kueche` (LIBERO-Fixtures und -Objekte mit Gelenken im Modell).
- `--robocasa`: Mission in der RoboCasa-Küche `robocasa_kueche7` (Ring um die Insel; 105 Weltgelenke, ~2600 Geoms).
- `--welt`, `--patrouille`, `--slam`, `--drei-d`, `--kueche`, `--robocasa` nutzen echtes Nav2 headless. WSL2 hat ~2,8 GB RAM: Serien nacheinander.
- `bash tests/system/stop_all.sh` beendet Reste nach abgebrochenen Tests. **Als eigenen Befehl ausführen**: die
  Suchmuster dürfen nicht in der Kommandozeile des Aufrufers stehen, sonst beendet `pgrep -f` den Aufrufer.
