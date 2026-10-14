# Tests

Alle Aufrufe aus dem Projekt-Root (`~/humanoid_robot`), Workspace gesourct
(`source /opt/ros/jazzy/setup.bash && source install/setup.bash`). Nach Codeänderungen erst `colcon build`
(Install-Ordner ist eine Kopie).

**Komplette Serie:** `bash tests/alle_tests.sh` (Build, pytest, alle Systemtests nacheinander, Ergebnistabelle;
`--schnell` nur Build + pytest, `--mit-slam` zusätzlich die SLAM-Mission).

## Wo liegt was
| Ort | Art | Inhalt |
|---|---|---|
| `src/roboter_steuerung/test/` | Unit (Layer 3) | `test_steuereinheit_logik.py`: Rampe, Kurshaltung akkumuliert nicht (Regression −572 rad/s), cmd-Begrenzung, Watchdog, Positionshaltung, NotAus/Freigabe |
| `src/roboter_navigation/test/` | Unit (Sensorvorverarbeitung) | `test_punktwolke_zu_scan.py`: Boden raus, niedrige Hindernisse drin, Neigungsausgleich, Ausdünnung |
| `src/roboter_task_planning/test/` | Unit (Layer 1) | `test_layer1.py`: Weltwissen, PDDL, Planer, Dispatcher, Neuplanung (braucht Fast Downward) |
| `tests/simulation/` | Simulation + Controller ohne ROS | `test_closed_loop.py` (Stehen, Gehen, Kurshaltung, Robustheit gegen Verzögerung/Jitter), `test_beinodometrie.py` (< 2 % der Strecke), Helfer `sim_helpers.py` |
| `tests/system/` | System (`ros2 launch`) | `integration_ros.py` (Pipeline: Startrennen, Freigabe, Gehen, Begrenzung, NotAus, Odometrie), `integration_watchdog.py`, `integration_mission.py` (Layer 1–4), `stop_all.sh` |
| `experimente/` | Experiment + Regression | `arm_feasibility.py` (Greifen im Stehen, Optionen `--obs-mask`, `--waist`, `--empfohlen`, `--phases`), `test_arm_feasibility.py` sichert den Betriebspunkt (0,30 m, ≤ 1 kg je Hand) |

`python3 -m pytest -q` sammelt alle schnellen Tests (siehe `pytest.ini`).

## Systemtests einzeln
| Aufruf | Dauer |
|---|---|
| `python3 tests/system/integration_ros.py [--realistisch] [--beinodometrie]` | ~2,5 min |
| `python3 tests/system/integration_watchdog.py` | ~1 min |
| `python3 tests/system/integration_mission.py [--nav2 \| --welt \| --patrouille \| --drei-d \| --nur-2d \| --slam] [--realistisch] [--beinodometrie]` | 2–6 min |

Hinweise:
- Die Systemtests starten und beenden die Pipeline selbst; währenddessen keine eigenen `ros2 topic echo/hz`-Aufrufe.
- Strenger Stillstand bei NotAus (Beine < 0,05 rad/s, Drift < 5 cm).
- `--welt`, `--patrouille`, `--slam`, `--drei-d` nutzen echtes Nav2 headless. WSL2 hat ~2,8 GB RAM: Serien nacheinander.
- `bash tests/system/stop_all.sh` beendet Reste nach abgebrochenen Tests. **Als eigenen Befehl ausführen**: die
  Suchmuster dürfen nicht in der Kommandozeile des Aufrufers stehen, sonst beendet `pgrep -f` den Aufrufer.
