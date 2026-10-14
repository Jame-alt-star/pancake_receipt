# Archiv (nicht gebaut, nicht getestet)

Code, der nicht mehr zum aktiven Projekt gehört, aber nachvollziehbar bleiben soll. Seit 15.10.2026 hier.
colcon baut diesen Ordner nicht (kein Paket), pytest sammelt ihn nicht (`pytest.ini`).

| Ordner | Inhalt | Warum archiviert |
|---|---|---|
| `12dof/` | `g1_policy.yaml`, TorchScript-Policy `motion.pt`, `rl_locomotion_controller.py`, `p_balance_controller.py` + `i_balance_controller.py` (Ankle-Strategy, 12-DOF-Gelenkindizes), Szenen `scene_lauf/hindernisse/objekte.xml` | Projekt läuft nur noch mit 29 DOF; der 12-DOF-Pfad wurde seit 14.10.2026 nicht mehr getestet und brauchte PyTorch |
| `gazebo_r1/` | Gazebo-/RViz-Launches für den Unitree R1, `r1_welt.sdf`, `gazebo_bridge.py` | Frühe Phase (R1 in Gazebo) vor dem Wechsel auf G1 in MuJoCo |
| `ament_lint_vorlagen/` | `test_flake8/pep257/copyright.py` aus `ros2 pkg create` | Schlugen von Beginn an fehl (Stil, deutsche Docstrings), waren nie Teil der Testserie |

Zurückholen: Datei mit `git mv` an den alten Ort (siehe `git log --follow <datei>`), alter Stand vollständig unter Tag `v1.9-review-abgeschlossen`.
