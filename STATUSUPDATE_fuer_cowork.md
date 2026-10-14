# Statusupdate G1-Humanoid-Projekt für Cowork — Stand 14.10.2026 (v1.9)

Kurzfassung für Übergaben. Einstieg: `README.md` (Start, Tests, Grenzen). Verlauf mit allen Messwerten:
`docs/statusupdate_g1_projekt.md`. Alte Übergaben (u. a. die widerlegte Latenz-Hypothese): `docs/archiv/`.
Git: `main`, Rücksprungpunkte siehe `git tag`.

## Rahmen
- Arbeitsprojekt (keine Thesis). Nur noch 29 DOF (12-DOF-Pfad bleibt im Code, wird nicht getestet).
- Layer 2 (Navigation) gehört einem Kollegen; `roboter_navigation` ist ein funktionierender Platzhalter.
- Greifen/Hände bewusst zurückgestellt (Machbarkeit gemessen, siehe unten).

## Status je Schicht
| Schicht | Stand |
|---|---|
| Layer 4 (`r1_bringup/mujoco_node.py`) | Echtzeit-MuJoCo, Physik pausiert bis der Regler verbunden ist; IMU/Gelenke 100 Hz; `/odom` + TF (Ground-Truth, abschaltbar); 3D-Lidar `/points` (24 Ringe, 5 Hz) oder 2D-Scan; `realistic_sensors` (Rauschen, Bias, Gier-Drift, Latenz, Lidar-Fehler); `headless`; Posenaufzeichnung (`record_path`) |
| Layer 3 (`roboter_steuerung`) | Zustandsautomat Start/Bereitschaft/Balancieren/Gehen/NotAus; ONNX-Policy; Rampe, Kurshaltung, Positionshaltung (Balancieren), cmd-Begrenzung (vx −0,5…1,0, vy ±0,3, wz ±1,0, Kombigrenze), `/cmd_vel`-Watchdog; **Freigabe über eigenen Kanal `/freigabe`** (NotAus entzieht sie, während NotAus wird alles abgelehnt); Balance-Dienste (`/gleichgewicht`, `/balanciere`); Beinodometrie (`odom_quelle:=beinodometrie`, < 1 % der Strecke) |
| Layer 2 (`roboter_navigation`) | Nav2 (RPP, NavFn, VoxelLayer auf der Hinderniswolke), optional `slam_toolbox`; `punktwolke_zu_scan` leitet `/scan` und `/punkte_hindernis` aus `/points` ab; RViz-Ansicht `rviz/ansicht_3d.rviz` |
| Layer 1 (`roboter_task_planning`) | Mission (YAML) → PDDL → Fast Downward → Dispatcher über ROS-2-Actions; Ziele besucht/balanciert/gewartet/am_ort; Neuplanung mit dynamischem Ort (nur ausgehende Kanten), Position per TF im map-Frame; Missionsstart gibt frei |
| Zustände | AkkuSchwach/Laden fehlen bewusst (keine echten Batteriedaten; Platzhalter-Akku abgelehnt) |

## Tests (alle grün, 14.10.2026)
pytest `tests/` 33 · Layer-1 18 · `integration_ros.py` 32/32 (auch mit `--realistisch --beinodometrie`) (inkl. `/cmd_vel` und Freigabe-Versuch während/nach NotAus) · Watchdog 5/5 · `integration_mission.py` 14/14 (Testdouble, `--nav2`, `--welt`, `--patrouille`, `--drei-d`), Gegenprobe `--nur-2d` 13/13, `--slam --realistisch --beinodometrie` 22/22. Aufrufe: `tests/README.md`.

## Code-Review vom 14.10.2026 – abgearbeitet
- Paket 1 Sicherheit (B1, B5, L1, L3) → v1.5 · Paket 2 Layer 1 (B2, B3, B4, U4, L4, L5) → v1.6
- Paket 3 Übergabe (B6, B7, U1, Kat. 6) → v1.7 · Paket 4 Kommentare/Doku (U2, Kat. 7) → v1.8
- Paket 5 kleinere Punkte (B8, U3, U5–U7, L2, L7, Duplikate; L6/L8/U8 dokumentiert) → v1.9
- Offen: Kategorie 5 (SysML-Dateien liegen nicht im Repo, daher keine Rückverfolgbarkeit Modell ↔ Code).

## Für den Kollegen (Nav2)
Referenz: `src/roboter_navigation/config/nav2_params.yaml`. Besonderheiten der Policy: Drehen auf der Stelle erst ab |wz| ≈ 0,8 (kleine wz setzt sie im Stand kaum um); vx ≥ 0,3; RPP `max_angular_accel` hoch (bezieht sich auf die gemessene Drehrate); `PoseProgressChecker`; Hinderniserkennung in 3D (VoxelLayer), weil die 2D-ObstacleLayer niedrige Hindernisse wieder löscht. Vor `/cmd_vel` muss `/freigabe` gesendet sein (Missionen erledigen das selbst). In Jazzy brauchen `collision_monitor` und `docking_server` Konfiguration; `navigation_launch.py` mit `use_composition=True` startet den Container nicht selbst.

## Greifen im Stehen (zurückgestellt)
`experimente/arm_feasibility.py`: unveränderte Policy trägt Arme nur bis ~0,20 m vor dem Becken (Ursache: Arm-Beobachtungen außerhalb des Trainingsbereichs). Mit „blinder“ Policy für die Arme und Taille 0,15 rad nach hinten: 0,30 m Reichweite bis 1 kg je Hand stabil. Kein Hand-Modell, kein Zustand `Manipulieren`, keine Wahrnehmung.

## Bekannte Grenzen
Ground-Truth-Odometrie als Standard (Schätzer optional); Lidar als Raycast; keine Kamera/Objekterkennung; Orte als Koordinaten vorgegeben (keine gespeicherte Karte/Lokalisierung, kein Teach-in); WSL2 ~2,8 GB RAM (Nav2+SLAM headless, Testserien einzeln, vorher `bash tests/system/stop_all.sh`). Beobachtet: in 1 von ~6 SLAM-realistisch-Läufen Abbruch einer Nav2-Etappe mit Türkontakt (nicht reproduziert).

## Arbeitsweise
Messbare Behauptungen belegen, „getestet“ von „angenommen“ trennen. Vor riskanten Änderungen committen und taggen. Fast Downward liegt außerhalb des Repos (`~/tools/downward`, Anleitung in `src/roboter_task_planning/README.md`). Sprache der Kommentare/Doku: Deutsch.
