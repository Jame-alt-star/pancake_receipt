# Projektstand aktuell — G1-Humanoid (Stand 15.10.2026, v2.5)

**Wozu diese Datei:** der aktuelle Stand in Kurzform, für Übergaben und für jeden, der neu einsteigt. Sie wird bei jedem
größeren Schritt aktualisiert. Was sich seit dem 10.10. geändert hat, steht in `NEU_SEIT_10-10_FUER_NAV2_KOLLEGEN.md`.

Kurzfassung für Übergaben. Einstieg: `README.md` (Start, Tests, Grenzen), Aufbau und Schnittstellen:
`docs/architektur.md`. Verlauf mit allen Messwerten: `docs/statusupdate_g1_projekt.md`. Alte Übergaben, Review-Unterlagen und Altcode: nur noch unter Tag `v1.9-review-abgeschlossen`.
Git: `main`, Rücksprungpunkte siehe `git tag`.

## Rahmen
- Arbeitsprojekt (keine Thesis). Nur noch 29 DOF (12-DOF-Pfad bleibt im Code, wird nicht getestet).
- Layer 2 (Navigation) gehört einem Kollegen; `roboter_navigation` ist ein funktionierender Platzhalter.
- Greifen/Hände bewusst zurückgestellt (Machbarkeit gemessen, siehe unten).

## Status je Schicht
| Schicht | Paket | Stand |
|---|---|---|
| Layer 4 | `roboter_simulation` | Echtzeit-MuJoCo (`mujoco_node` + Bausteine `szene`, `sensormodell`, `lidar`, `aufnahme`); startet erst mit den Antriebsparametern der Steuerung, Physik pausiert bis zum ersten Kommando; IMU/Gelenke 100 Hz; `/odom` + TF (Ground-Truth, abschaltbar); 3D-Lidar `/points` oder 2D-Scan; `realistic_sensors`; `headless`; Posenaufzeichnung |
| Layer 3 | `roboter_steuerung` | Zustandsautomat Start/Bereitschaft/Balancieren/Gehen/NotAus; ONNX-Policy mit `config/policy_29dof.yaml`; `kommando.py` (Rampe, Begrenzung, Kombigrenze, Watchdog), `haltung.py` (Kurs-, Positionshaltung); Freigabe über `/freigabe`; Hardware-Schnittstelle `hardware/` (`IRobotHardwareBridge`, `TopicBridge`); Balance-Dienste; Beinodometrie (< 1 % der Strecke) |
| Sensorik | `roboter_sensorik` | `punktwolke_zu_scan`: `/scan` und `/punkte_hindernis` aus `/points` (unabhängig von der Nav2-Variante) |
| Layer 2 | `roboter_navigation` | Nav2 (RPP, NavFn, VoxelLayer), optional `slam_toolbox` – Platzhalter für den Kollegen |
| Layer 1 | `roboter_task_planning` | Mission (Auftrag, verweist per `welt:` auf die Orte) → PDDL → Fast Downward → Dispatcher; Neuplanung mit dynamischem Ort; Position per TF im map-Frame |
| Roboter/Umgebung | `roboter_beschreibung`, `roboter_welten` | G1-Modell + Geometrie; Welten `leer`, `testwelt`, `testwelt3d`, `demo_lager` (Szene + Orte) |
| Gesamt | `roboter_system` | `system.launch.py` (Schalter `mission`, `navigation`, `slam`, `lidar_modus`, `odom_quelle`, `realistic_sensors`, ...), RViz-Ansicht |
| Zustände | – | AkkuSchwach/Laden fehlen bewusst (keine echten Batteriedaten; Platzhalter-Akku abgelehnt) |

## Tests (alle grün, 15.10.2026)
`bash tests/alle_tests.sh [--mit-slam]`: Build, pytest 64 (Unit-Tests der Pakete, Regelkreis, Konfiguration, Simulationsbausteine,
Greif-Regression), Pipeline 32/32 (auch realistisch + Beinodometrie), Watchdog 5/5, Missionen 14/14 (Testdouble, Testwelt,
Patrouille, 3D-Palette), SLAM realistisch. Aufrufe und Ablage: `tests/README.md`.

## Code-Review vom 14.10.2026 – abgearbeitet
- Paket 1 Sicherheit (B1, B5, L1, L3) → v1.5 · Paket 2 Layer 1 (B2, B3, B4, U4, L4, L5) → v1.6
- Paket 3 Übergabe (B6, B7, U1, Kat. 6) → v1.7 · Paket 4 Kommentare/Doku (U2, Kat. 7) → v1.8
- Paket 5 kleinere Punkte (B8, U3, U5–U7, L2, L7, Duplikate; L6/L8/U8 dokumentiert) → v1.9
- Offen: Kategorie 5 (SysML-Dateien liegen nicht im Repo, daher keine Rückverfolgbarkeit Modell ↔ Code; vorgesehen: `modell/`).

## Strukturumbau 15.10.2026 (v2.0–v2.4)
Aufräumen (Altbestand zunächst archiviert, inzwischen gelöscht – siehe Tag `v1.9-review-abgeschlossen`; Tests nach Art) → Welten (`roboter_welten`) → Launch je Schicht (`roboter_system`,
`roboter_sensorik`, `r1_bringup` → `roboter_simulation`) → Config/Bridge entflechten (`roboter_beschreibung`, Policy und
Bridge in Layer 3, `/antrieb/parameter`, Parameterdateien je Schicht) → große Knoten zerlegt. Verhalten unverändert (gleiche Tests, gleiche Messwerte).

## Für den Kollegen (Nav2)
Referenz: `src/roboter_navigation/config/nav2_params.yaml`. Eigene Variante: `system.launch.py navigation:=nav2 nav2_params:=<datei>`
oder Nav2 selbst starten und unser System mit `navigation:=aus` plus `ros2 launch roboter_task_planning mission.launch.py`.
Eingänge `/scan` und `/punkte_hindernis` liefert `roboter_sensorik` unabhängig von seinem Paket. Besonderheiten der Policy:
Drehen auf der Stelle erst ab |wz| ≈ 0,8; vx ≥ 0,3; RPP `max_angular_accel` hoch; `PoseProgressChecker`; Hinderniserkennung in 3D
(VoxelLayer). Vor `/cmd_vel` muss `/freigabe` gesendet sein (Missionen erledigen das selbst). In Jazzy brauchen
`collision_monitor` und `docking_server` Konfiguration; `navigation_launch.py` mit `use_composition=True` startet den Container nicht selbst.

## Greifen im Stehen (zurückgestellt)
`experimente/arm_feasibility.py`: unveränderte Policy trägt Arme nur bis ~0,20 m vor dem Becken (Ursache: Arm-Beobachtungen außerhalb des Trainingsbereichs). Mit „blinder“ Policy für die Arme und Taille 0,15 rad nach hinten: 0,30 m Reichweite bis 1 kg je Hand stabil. Kein Hand-Modell, kein Zustand `Manipulieren`, keine Wahrnehmung.

## Bekannte Grenzen
Ground-Truth-Odometrie als Standard (Schätzer optional); Lidar als Raycast; keine Kamera/Objekterkennung; Orte als Koordinaten vorgegeben (keine gespeicherte Karte/Lokalisierung, kein Teach-in); WSL2 ~2,8 GB RAM (Nav2+SLAM headless, Testserien einzeln, vorher `bash tests/system/stop_all.sh`). Beobachtet: in 1 von ~6 SLAM-realistisch-Läufen Abbruch einer Nav2-Etappe mit Türkontakt (nicht reproduziert).

## Arbeitsweise
Messbare Behauptungen belegen, „getestet“ von „angenommen“ trennen. Vor riskanten Änderungen committen und taggen. Fast Downward liegt außerhalb des Repos (`~/tools/downward`, Anleitung in `src/roboter_task_planning/README.md`). Sprache der Kommentare/Doku: Deutsch.
