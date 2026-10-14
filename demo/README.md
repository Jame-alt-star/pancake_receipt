# Demo: Kontrollrundgang im Lager

Zeigt die komplette Pipeline in der Simulation: PDDL-Planung (Fast Downward), Navigation mit Nav2 und
SLAM, Hindernisumfahrung per Lidar, Bürotür, Balancieren/Warten, Rückkehr zur Ladestation, anschließend NOT-AUS.

- Szene: `src/roboter_welten/welten/demo_lager/` (`szene.xml`, `orte.yaml`) (Halle mit Regalen und
  Kistenstapeln, Büro hinter einer 1,6 m breiten Tür)
- Mission: `src/roboter_task_planning/missions/demo_lager.yaml`
- Konfiguration: SLAM, realistische Sensorik (Rauschen, Latenz, Gyro-Bias), Beinodometrie statt Ground-Truth

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 demo/run_demo.py /tmp/demo_lauf          # fährt die Pipeline, schreibt pose.npz + ereignisse.json (~3 min)
MUJOCO_GL=glfw python3 demo/render_demo.py /tmp/demo_lauf demo/ausgabe/demo.mp4   # ~25 min Rendern
```
Die Aufnahme der Pose läuft in `mujoco_node` (`record_path`), die Simulation bleibt in Echtzeit; das Bild
wird danach offline gerendert (Verfolgerkamera, Lidar-Treffer, Leitstand-Karte, Beschriftung).
Benötigt `imageio-ffmpeg` (`pip install --user imageio-ffmpeg`) und ein GLFW-fähiges Display (WSLg).
