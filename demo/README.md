# Demo: Kontrollrundgang im Lager

Zeigt die komplette Pipeline in der Simulation: PDDL-Planung (Fast Downward), Navigation mit Nav2 und
SLAM, Hindernisumfahrung per Lidar, Bürotür, Balancieren/Warten, Rückkehr zur Ladestation, anschließend NOT-AUS.

- Szene: `src/roboter_welten/welten/demo_lager/` (`szene.xml`, `orte.yaml`) (Halle mit Regalen und
  Kistenstapeln, Büro hinter einer 1,6 m breiten Tür)
- Mission: `src/roboter_task_planning/missions/demo_lager.yaml`
- Konfiguration: SLAM, realistische Sensorik (Rauschen, Latenz, Gyro-Bias), Beinodometrie statt Ground-Truth

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 demo/run_demo.py /tmp/demo_lauf          # fährt die Pipeline, schreibt pose.npz + ereignisse.json + lauf.json (~3 min)
MUJOCO_GL=glfw python3 demo/render_demo.py /tmp/demo_lauf demo/ausgabe/demo.mp4   # ~25 min Rendern
MUJOCO_GL=glfw python3 demo/render_demo.py /tmp/demo_lauf /tmp/bild.png --probe 20  # Standbild bei 20 s (zum Prüfen)
```
Andere Mission (die Welt kommt aus der Mission): `python3 demo/run_demo.py /tmp/lauf_3d --mission testwelt3d` –
Testwelt mit niedriger Palette, die nur das 3D-Lidar sieht (Video vom 14.10.2026: `demo/ausgabe/demo_testwelt3d.mp4`).

Darstellung: Treffer des **3D-Lidars** (derselbe Baustein wie in der Simulation, 24 Ringe), nach Höhe eingefärbt
(grün niedrig, orange hoch); die Leitstand-Karte sammelt die Hindernispunkte im Höhenband 0,12–1,9 m wie
`roboter_sensorik`. Auftrag und Ortsnamen werden aus Mission und Welt erzeugt.
Die Aufnahme der Pose läuft in `mujoco_node` (`record_path`), die Simulation bleibt in Echtzeit; das Bild
wird danach offline gerendert (Verfolgerkamera, Lidar-Treffer, Leitstand-Karte, Beschriftung).
Benötigt `imageio-ffmpeg` (`pip install --user imageio-ffmpeg`) und ein GLFW-fähiges Display (WSLg).
