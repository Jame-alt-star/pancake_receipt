# Projektstand aktuell — G1-Humanoid (Stand 22.10.2026, v2.9)

**Wozu diese Datei:** der aktuelle Stand auf einer Seite, für Übergaben und für jeden, der neu einsteigt. Sie wird bei jedem
größeren Schritt aktualisiert und enthält nur den Stand und die offenen Punkte; alles Weitere steht genau einmal an
seinem Ort (Tabelle unten).

## Rahmen
- Arbeitsprojekt (keine Thesis). Unitree G1 mit 29 DOF in MuJoCo; 12 DOF und Gazebo sind entfernt (Tag `v1.9-review-abgeschlossen`).
- Layer 2 (Navigation) gehört einem Kollegen; `roboter_navigation` ist ein funktionierender Platzhalter.
- Greifen/Hände: geplant als eigene, per RL trainierte Greif-Policy (Training auf einer gemieteten GPU); die frühere
  Machbarkeitsmessung mit IK ist entfernt (Verlauf 18.11.2026).

## Stand
Die Pipeline läuft in der Simulation über alle vier Schichten: Mission planen (PDDL/Fast Downward, Wegkosten über
Nav2) → navigieren (Nav2 mit 3D-Lidar; ohne Karte, mit SLAM oder auf fester Karte mit AMCL) → gehen und stehen
(RL-Policy, Zustandsautomat, NotAus, Freigabe, Störungserkennung) → Simulation (MuJoCo, realistische Sensorik
optional, Beinodometrie optional). Ein Befehl startet alles: `ros2 launch roboter_system system.launch.py ...`.

**In Arbeit: RGBD-Kamera (Simulation).** Kamera am Kopf (`kamera:=true`, aus per Standard), gerendert in eigenem Thread,
5 Hz, 320×240: Farb- und Tiefenbild auf `/kamera/color|depth/image_raw` + `camera_info`, Frame `kamera_optical_frame`
im TF-Baum, optional Tiefenrauschen mit `realistic_sensors`. Montage in `roboter_beschreibung/config/roboter.yaml`
(Beispielwerte). Unit-Tests `tests/simulation/test_kamera.py` 8/8; noch nicht committet und nicht in der Systemtestserie.

## Tests (21.10.2026, alle grün)
`bash tests/alle_tests.sh --mit-slam`: pytest 88, Pipeline 37/37 (auch realistisch + Beinodometrie, mit Viewer),
Watchdog 8/8, Missionen 17/17 (Testdouble, Testwelt, Patrouille, 3D-Palette), feste Karte mit AMCL 21/21, SLAM
realistisch 25/25, Karte aufnehmen und damit arbeiten (wie in echt) 34/34. Einmal hing dabei der Nav2-Start
(`docking_server`), Wiederholung grün (Verlauf). Video aller Tests: `--video DIR` + `demo/video_tests.py`. Schnelle Tests zusätzlich
bei jedem Push auf GitHub (`.github/workflows/tests.yml`).

## Offen
| Punkt | Warum offen |
|---|---|
| Unitree-SDK-Bridge für echte Hardware | Schnittstelle (`IRobotHardwareBridge`, `/antrieb/parameter`) steht; Bestellung und Konfiguration: `docs/g1_hardware_checkliste.md` |
| Simulationsmodell auf `g1_29dof_rev_1_0` umstellen | Policy ist darauf trainiert, aktuelle Roboter sind Rev. 1.0; wir simulieren noch `g1_29dof` (älter, Hüft-Roll 88 statt 139 Nm, 1,8 kg schwerer) |
| Echte Lidar-Montage eintragen | Pose des MID-360 (Kopf, geneigt) aus Unitree-Datenblatt in `roboter_beschreibung/config/roboter.yaml` |
| Karte der echten Halle + Orte (Teach-in) | heute: Karten aus der Simulationsszene; für die Halle per SLAM aufnehmen (`docs/architektur.md`, Karte) |
| Nav2-Variante des Kollegen | liegt noch nicht vor; Einbindung: `navigation:=nav2 nav2_params:=...` oder `navigation:=extern` |
| SysML-Modell im Repo + Rückverfolgbarkeit | Modelldateien liegen nicht vor (vorgesehen: `modell/`) |
| Lizenzfrage eigener Code / Weitergabe | Entscheidung des Arbeitgebers; Fremdlizenzen in `THIRD_PARTY.md` |
| Kamera fertigstellen | Simulation steht (siehe Stand); offen: Abnehmer der Bilder (z. B. Objekterkennung), echte Montage, Bildgröße und Öffnungswinkel aus dem Unitree-Datenblatt, Systemtest. Mit Kamera ohne GPU (Software-Rendering) gemessen 0,87–0,9× Echtzeit und in etwa jedem zweiten Lauf einzelne Lücken in `/joint_states` bis ~350 ms (ohne Störung); Abhilfe falls nötig: Kamera als eigener Prozess |
| Greif-Policy | RL-Training auf einer gemieteten GPU (RTX-VM) geplant; Robotermodell mit Händen siehe `docs/g1_hardware_checkliste.md` |
| AkkuSchwach/Laden | bewusst zurückgestellt (keine echten Batteriedaten) |
| Rechner: WSL2 mit 2,8 GB, Uhr geht nach | läuft seit dem Abspecken der Steuerung stabil; mehr Speicher bzw. Neustart der WSL entschärft beides (Verlauf 21.10.) |

## Wo steht was
| Thema | Datei |
|---|---|
| Starten, Schalter, Tests | `README.md` |
| Pakete, Schnittstellen, TF-Baum, Karte, Sicherheit, Entscheidungen | `docs/architektur.md` |
| Nav2-Platzhalter, Hinweise für die Variante des Kollegen | `src/roboter_navigation/README.md` |
| Missionen, Planer | `src/roboter_task_planning/README.md` |
| Welten, Karten | `src/roboter_welten/README.md` |
| Testarten und Aufrufe | `tests/README.md` |
| Einrichten eines Rechners | `scripts/einrichten.sh --pruefen` |
| Verlauf mit allen Messwerten | `docs/statusupdate_g1_projekt.md` |
| Fremdbestandteile und Lizenzen | `THIRD_PARTY.md` |
| G1 bestellen und konfigurieren (Checkliste für Unitree/Händler) | `docs/g1_hardware_checkliste.md` |

## Arbeitsweise
Messbare Behauptungen belegen, „getestet“ von „angenommen“ trennen. Jeder Schritt: komplette Testserie, Commit, Tag
(`git tag`), Push. Sprache von Kommentaren und Doku: Deutsch.
