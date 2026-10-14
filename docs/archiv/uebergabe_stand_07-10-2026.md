# Übergabe – Stand 07.10.2026 (Humanoid Robot / Ros2.1-Workspace)

## Was heute gemacht wurde

### 1. Code-Dokumentation wiederhergestellt

Der vom Kollegen umstrukturierte Code enthielt keine erklärenden Kommentare mehr. Der gesamte Code wurde durchgegangen und Dokumentation ergänzt in:

- `g1_policy.yaml`
- `mujoco_bridge.py`
- `i_hardware_bridge.py`
- `mujoco_node.py`
- `steuereinheit_node.py`
- `rl_locomotion_controller.py`

### 2. Bugfix: Gelenkanzahl als Single Source of Truth

In `mujoco_bridge.py` war die Anzahl der Robotergelenke (12, da nur die Bein-Policy aktiv ist) an mehreren Stellen hart codiert (`np.zeros(12, ...)`). Jetzt wird sie als `num_joints`-Parameter übergeben, gespeist aus `g1_policy.yaml`'s `num_actions`. `steuereinheit_node.py` übergibt diesen Wert beim Instanziieren der Bridge (`MujocoBridge(self, num_joints=self.config["num_actions"])`).

### 3. Performance-Fix: Render-Decimation

In `mujoco_node.py` wurde `viewer.sync()` (Fenster-Rendering) vorher bei jedem einzelnen Physikschritt aufgerufen und hat die Simulationsschleife von den angenommenen 500 Hz auf gemessene ~90 Hz ausgebremst. Fix: Rendering nur noch alle `RENDER_DECIMATION = 10` Physikschritte, Physik-Berechnung und `/joint_states`/`/imu`-Publishing laufen weiterhin bei jedem Schritt.

### 4. Zwei echte, blockierende Bugs gefunden und gefixt

- `steuereinheit_node.py`: `policy_path` wurde nicht gegen das Package-Share-Verzeichnis aufgelöst → `ValueError: The provided filename models/policies/motion.pt does not exist`. Gefixt nach demselben Muster wie `xml_path` in `mujoco_node.py` (Share-Verzeichnis, mit Fallback auf den Source-Baum).
- `r1_bringup/setup.py`: fehlender `data_files`-Eintrag für den neuen `labels/`-Ordner → `ValueError: Error opening file 'labels/1.png'`. Ergänzt nach demselben Muster wie die bestehenden `meshes/`/`images/`-Einträge.

### 5. Test-Welten für den Roboter gebaut

Die bestehende modulare Scene-Struktur (`<include file="g1_12dof.xml"/>` trennt Robotermodell von Umgebung) wurde genutzt, um mehrere eigenständige MuJoCo-Testszenen anzulegen, ohne das Robotermodell zu duplizieren:

- **Welt 1** (`scene_lauf.xml`): einfache Laufbahn mit Meter-Markierungen 1–10 (farbige Bodenstreifen + Zahlen-Schilder als PNG-Texturen) und einer Ziel-Mauer bei x=11m, damit der Roboter nicht unendlich weiterläuft.
- **Welt 2** (`scene_hindernisse.xml`): 5-Stationen-Hindernisparkour, orientiert an realen Lauf-Benchmarks für Beinroboter (DARPA Robotics Challenge, ANYmal/ANYbotics Rough-Terrain-Tests, Boston Dynamics Atlas, Isaac Gym/Isaac Lab Terrain-Curricula):
  1. Bordstein (8 cm Stufe hoch/runter)
  2. Hürde (10 cm Balken quer zum Weg)
  3. Rampe (12° Steigung auf ein Podest)
  4. Gelände (7 unregelmäßige Bodenwellen, 2–4 cm)
  5. Schräge (8° Querneigung/seitliche Balance – bewusster Ersatz für eine ursprünglich angedachte "Lücke", die technisch nicht umsetzbar war, da der Boden eine unendliche MuJoCo-Plane ist und kein Loch haben kann)

  Dazu durchgehende Begrenzungsmauern links/rechts über die gesamte Parkourlänge und eine Ziel-Mauer am Ende, mit Beschriftung an jeder Station.
- Eine dritte Welt mit beweglichen Objekten (Kiste/Ball zum Wegschieben) wurde ebenfalls gebaut und kurz getestet. Dabei kam ein grundsätzlicher Bug in `mujoco_node.py` zum Vorschein (siehe Offene Punkte, Punkt 2) – diese Welt wurde danach wieder entfernt, um den Fokus auf Welt 1 und 2 zu halten.

Beide verbliebenen Welten wurden erfolgreich getestet. `g1_policy.yaml` → `xml_path` zeigt aktuell auf Welt 2 (`scene_hindernisse.xml`).

## Offene Probleme / Fragen

1. **Lessons Learned zu mehreren freien Körpern in einer Szene.** `mujoco_node.py` ging an mehreren Stellen (`qpos`/`qvel`-Indizierung) implizit davon aus, dass nach dem Roboter-Freejoint (7 `qpos`-Werte) nur die Roboter-Gelenke folgen. Sobald eine Szene weitere `<freejoint/>`-Körper enthält (z.B. bewegliche Objekte), muss das explizit berücksichtigt werden (`qpos[7:7+num_joints]` statt `qpos[7:]`, analog bei `qvel`), sonst gibt es einen Shape-Fehler (`could not broadcast ... (12,) (26,)`). Relevant, falls später wieder eine Welt mit beweglichen Objekten gebaut wird.
2. **Blockierender ImportError in `i_locomotion_controller.py`.** War zu Beginn der heutigen Arbeit als leere Datei (0 Bytes) identifiziert; `rl_locomotion_controller.py` importiert `ILocomotionController` daraus. Der Fix wurde bewusst zurückgestellt – Status unklar, sollte vor dem nächsten größeren Testlauf geprüft werden.
3. **Nav2 vs. perzeptive RL-Policy (Layer 2, Hindernisvermeidung).** Der Vorgehensplan erwähnt ggf. Nav2/SLAM für `/cmd_vel`, das unverändert in die bestehende RL-Policy gespeist würde – das steht im möglichen Widerspruch zu einem früher diskutierten Plan einer "perzeptiven" Policy mit Höhenkarten-/Kamera-/LiDAR-Input. Nicht aufgelöst, sollte mit dem Kollegen/Betreuer geklärt werden.
4. **Ursprung der Zahlenwerte in `g1_policy.yaml`** (kps/kds/scales): nicht zurückverfolgt, aus welchem Trainings-/Tuning-Skript die genauen Werte stammen. Für Nachvollziehbarkeit/Thesis-Dokumentation ggf. relevant.
5. **Kleinere Cleanup-Punkte** (niedrige Priorität): `<depend>r1_bringup</depend>` fehlt in `roboter_steuerung/package.xml`; alte R1/Gazebo-Reste und ungenutzte Dependencies in `r1_bringup/package.xml`/`setup.py`; aktuell nur eine synthetische IMU statt zwei physischer (Fidelity-Frage); der harte Fallback-Pfad in `mujoco_node.py`/`steuereinheit_node.py` geht davon aus, dass der Workspace exakt unter `~/humanoid_robot` liegt.
6. **PlanSys2 vs. eigener PDDL-Dispatcher** für Layer 1 (`roboter_task_planning`) – offene Architekturfrage, noch nicht mit dem Kollegen geklärt.

## Vorschläge für die Weiterarbeit
1. /cmd_vel-Subscriber in steuereinheit_node.py ergänzen. Aktuell wird self.cmd nur einmal beim Start aus der Config gesetzt (cmd_init: [0.5, 0, 0]) — es gibt keine Laufzeit-Steuerung. Ohne das ist eure eigene Pipeline in Sachen Bedienbarkeit noch nicht besser als die alte Testumgebung. Das ist der direkteste, klar umsetzbare nächste Programmierschritt.

2. State-Pattern-Dateien implementieren: i_robot_state.py, balancieren_state.py, gehen_state.py — aktuell leer/nicht umgesetzt. Das ist der Layer-3-Baustein, der im Vorgehensplan des Kollegen vorgesehen ist, aber noch fehlt.

3. Balance-Controller implementieren: i_balance_controller.py, p_balance_controller.py — ebenfalls noch leer. Der alte, 576-Zeilen-R1-Regler aus Ros2/roboter_ws ist dafür schon als mögliche Referenz vorgemerkt.
