# Übergabe an Claude Code (WSL2) — 29-DOF-Debugging, Stand 13.10.2026

Du läufst jetzt direkt in Pascals WSL2-Umgebung (`~/humanoid_robot`), mit echtem Zugriff auf ROS2, MuJoCo, colcon. Das hier ist die Übergabe aus einer vorherigen Claude-Sitzung (Cowork, Cloud-Sandbox ohne ROS2-Zugriff), die die Vorarbeit gemacht hat. Lies zuerst `statusupdate_g1_projekt.md` (liegt daneben, gehört ins Projekt-Root oder `docs/`) für die volle Historie. Diese Datei hier ist nur der direkte Einstiegspunkt für die aktuell offene Aufgabe.

## Kontext in Kürze

Migration von einer 12-DOF-Bein-only-Policy (TorchScript, `unitree_rl_gym`) auf eine 29-DOF-Ganzkörper-Policy (ONNX, `unitree_rl_lab`) — Hypothese: bewegliche Arme/Taille helfen beim Balance-Ausgleich. Beide Pfade existieren parallel (`config_file:=g1_policy.yaml` = alt/Standard, `config_file:=g1_policy_29dof.yaml` = neu), nichts am alten Pfad wurde verändert.

## Bereits gefunden und behoben

**Bug 1 (behoben, bestätigt):** `g1_29dof.xml` hatte für keines der 29 Gelenke einen `armature`-Wert gesetzt (Standard 0.0). Isaac Lab (wo die Policy trainiert wurde) nutzt für alle G1-29DOF-Aktuatoren `armature=0.01` (Quelle: `unitree_rl_lab/source/unitree_rl_lab/unitree_rl_lab/assets/robots/unitree.py`, `UNITREE_G1_29DOF_CFG`). Fehlende Rotor-Trägheit führte zu numerischer Instabilität nach ~5s (isoliert getestet, ohne ROS2). Fix: `armature="0.01"` auf allen 29 `<joint>`-Elementen ergänzt (Original liegt als `g1_29dof.xml.bak_vor_armature_fix` daneben). **Isoliert (reine Physik, kein ROS2) bestätigt: 20s stabil, Stehen UND Gehen, in meiner eigenen Headless-Sandbox.**

**Noch offenes Problem (NICHT behoben, das ist deine Aufgabe):** Trotz korrekt installiertem Fix (verifiziert: `grep -c 'armature="0.01"'` auf `src/.../g1_29dof.xml` UND `install/.../g1_29dof.xml` ergab beide Mal `29`) fällt der Roboter im echten ROS2-Lauf (`ros2 launch roboter_steuerung steuereinheit.launch.py config_file:=g1_policy_29dof.yaml`) weiterhin um — und zwar schon während `BereitschaftState` (noch bevor überhaupt ein `/cmd_vel`-Kommando kam), nach ca. 4,36 Sekunden, mit genau derselben Fehlermeldung wie vorher (`Nan, Inf or huge value in CTRL at ACTUATOR 1`).

## Diagnose-Hypothese (durch Simulation gestützt, NICHT im echten System verifiziert)

`mujoco_node.py` und `steuereinheit_node.py` laufen als zwei unabhängig getaktete ROS2-Prozesse, verbunden nur über Topics (`/joint_states`, `/imu`, `/joint_commands`) — keine echte Synchronisation. Ich habe in der isolierten Sandbox eine künstliche Verzögerung zwischen "Sensor lesen" und "Zielwinkel anwenden" simuliert (simuliert exakt das Publish/Subscribe-Timing):

| Verzögerung | Ergebnis |
|---|---|
| 0–100ms | stabil (20s getestet) |
| 160ms | fällt nach 2,05s um |

**Das deutet stark darauf hin, dass die reale Round-Trip-Latenz zwischen den beiden Prozessen bei dir über ~100-150ms liegt** — für die 29-DOF-Policy offenbar zu viel (bei der alten 12-DOF-Policy war das Problem ähnlicher Art, wurde aber durch `RENDER_DECIMATION = 10` in `mujoco_node.py` ausreichend entschärft; für 29 DOF reicht das scheinbar nicht mehr, vermutlich weil mehr Gelenke = mehr Rechenlast pro PD-Schritt + größere `/joint_states`/`/joint_commands`-Nachrichten = mehr Publish-Overhead).

## Konkrete nächste Schritte

1. **Tatsächliche Latenz/Rate messen, nicht nur vermuten:**
   ```
   ros2 topic hz /joint_states
   ros2 topic hz /joint_commands
   ```
   während die Pipeline mit `config_file:=g1_policy_29dof.yaml` läuft. Vergleiche mit dem erwarteten Ziel: 500 Hz Physik, 50 Hz Regeltakt. Beim 12-DOF-Pfad lag die reale Rate nach dem `RENDER_DECIMATION`-Fix bei ~320-355 Hz — miss, ob die 29-DOF-Variante deutlich darunter liegt.

2. **Overhead in `mujoco_node.py::_publish_states()` prüfen/reduzieren.** Aktuell werden `/joint_states`, `/imu`, `/base_position` bei JEDEM Physikschritt publiziert (bis zu 500 Hz) — für 29 Gelenke (statt 12) ist die Nachrichtengröße ~2,4x größer. Ein möglicher erster Versuch: Publish-Rate dieser Topics auf z.B. 100-125 Hz drosseln (eigener `PUBLISH_DECIMATION`-Zähler, analog zu `RENDER_DECIMATION`) und beobachten, ob sich die Stabilität verbessert oder verschlechtert (Vorsicht: zu seltenes Publizieren könnte die effektive Latenz für `steuereinheit_node.py` auch erhöhen statt senken — das ist ein Zielkonflikt, den du empirisch austesten musst).

3. **Alternativ/zusätzlich:** profilieren, was in `mujoco_node.py`'s Schleife pro Schritt tatsächlich Zeit kostet (reine Physik vs. Publish-Overhead vs. PD-Berechnung) — z.B. mit einfachen `time.perf_counter()`-Messungen um die einzelnen Teile der `_run_simulation()`-Schleife.

4. Erst wenn die reale Pipeline stabil läuft (Ziel: mindestens so lange wie mein isolierter Test, 20s+, für Stehen UND Gehen), zurück zur eigentlichen Forschungsfrage: hilft die 29-DOF-Ganzkörper-Policy tatsächlich gegen das Kippen (Pascals Ausgangshypothese), verglichen mit dem 12-DOF-Pfad?

## Wichtige Dateien

- `src/r1_bringup/models/robots/g1_description/g1_29dof.xml` — Modell, armature-Fix bereits drin
- `src/r1_bringup/config/g1_policy_29dof.yaml` — Config (kps/kds/default_angles/controller_type)
- `src/roboter_steuerung/roboter_steuerung/control/rl_locomotion_controller_29dof.py` — ONNX-Controller
- `src/r1_bringup/r1_bringup/mujoco_node.py` — hier vermutlich der nächste Eingriffspunkt (Publish-Overhead)
- `src/roboter_steuerung/roboter_steuerung/steuereinheit_node.py` — Regelschleife, `config_file`-Parameter

## Arbeitsweise, die sich bisher bewährt hat (bitte fortsetzen)

- Jede Behauptung mit echter Messung belegen (Zahlen, nicht "sollte jetzt gehen").
- Alternativen zurückstellen, nicht löschen (altes Verhalten bleibt per Default erreichbar).
- Vor riskanten Änderungen: `git commit` (Repo bereits vorhanden: `https://github.com/Pascho02/g1-humanoid-robot`, privat).
- `statusupdate_g1_projekt.md` laufend aktualisieren, mit ehrlicher Trennung "getestet" vs. "angenommen".
