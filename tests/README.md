# Tests

Alle Aufrufe aus dem Projekt-Root (`~/humanoid_robot`).

| Datei | Was | Voraussetzung | Dauer |
|---|---|---|---|
| `test_closed_loop.py` | Synchroner Regelkreis (echter Controller + echtes Modell, ohne ROS2/Viewer): Stehen, Gehen, Kurshaltung, Stopp, Robustheit gegen Verzögerung/Jitter/Takt-Abweichung, PD-Haltung ohne Policy fällt um (Begründung für `wait_for_controller`) | `pytest`, `mujoco`, `onnxruntime` | ~20 s |
| `test_node_logic.py` | Knoten-Logik ohne laufendes ROS2: Rampe, Kurshaltung akkumuliert nicht (Regression -572 rad/s), cmd-Begrenzung, Zielrichtung | gesourcter Workspace (`rclpy`, `roboter_steuerung`) | ~5 s |
| `arm_feasibility.py` / `test_arm_feasibility.py` | Machbarkeit Greifen im Stehen: Arme per IK bewegen, Last, Reichweite; Optionen `--obs-mask`, `--waist`, `--empfohlen`, `--phases`; pytest sichert den Betriebspunkt (0,30 m, ≤ 1 kg je Hand) | `mujoco`, `onnxruntime` | ~15 s |
| `integration_watchdog.py` | `cmd_vel`-Watchdog: einzelnes Kommando, Roboter stoppt nach Timeout | gesourcter Workspace | ~1 min |
| `integration_mission.py` | Layer 1 + 3 + 4 in der Simulation mit Nav2-Testdouble (`--nav2`: echtes Nav2 aus `roboter_navigation`, headless): Mission start→a→b läuft komplett, Plan, NotAus bricht ab, Start bei NotAus abgelehnt | gesourcter Workspace, Display, Fast Downward | ~2 min |
| `integration_ros.py` | Echte Pipeline (`ros2 launch`): Start ohne Kommando (Startrennen), Freigabe, Gehen, zu großes `cmd_vel`, NotAus, Entwarnung ohne Freigabe, Neustart der Bewegung, Log ohne Instabilität/Traceback | gesourcter Workspace, Display (WSLg) | ~2,5 min |

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
cd tests && python3 -m pytest -q                      # schnelle Tests
cd .. && python3 tests/integration_ros.py             # 29-DOF-Pipeline
python3 tests/integration_ros.py --config g1_policy.yaml   # 12-DOF-Regression
```

Hinweise:
- Nach Codeänderungen erst `colcon build` (Install-Ordner ist eine Kopie, kein Symlink).
- `integration_ros.py` startet und beendet die Pipeline selbst; währenddessen keine eigenen `ros2 topic echo/hz`-Aufrufe (stören die Simulation).
- Strenger Stillstand bei NotAus (Beine < 0,05 rad/s, Drift < 5 cm) gilt nur für 29-DOF; die 12-DOF-Policy trippelt bei cmd=0 (dokumentiert), dort wird nur „kein Sturz, Drift < 30 cm“ geprüft.

## Hilfsmittel
`bash tests/stop_all.sh` beendet Pipeline, Nav2/SLAM und Hilfsknoten nach abgebrochenen Tests (als eigener
Befehl ausfuehren). `integration_mission.py` kennt `--nav2`, `--welt`, `--patrouille`, `--slam`
(`--drei-d`: 3D-Lidar + niedrige Palette, `--nur-2d`: Gegenprobe, `--realistisch`, `--beinodometrie`; Nav2 laeuft headless; WSL2 hat nur ~2,8 GB RAM, Testserien daher einzeln und nacheinander starten).
