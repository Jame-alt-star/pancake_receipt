# Projektstand aktuell — G1-Humanoid (Stand 12.11.2026, Greifen Phase 1 in Arbeit; Basis v2.9 vom 22.10.)

**Wozu diese Datei:** der aktuelle Stand auf einer Seite, für Übergaben und für jeden, der neu einsteigt. Sie wird bei jedem
größeren Schritt aktualisiert und enthält nur den Stand und die offenen Punkte; alles Weitere steht genau einmal an
seinem Ort (Tabelle unten). Was sich seit dem 10.10. geändert hat: `NEU_SEIT_10-10_FUER_NAV2_KOLLEGEN.md`.

## Rahmen
- Arbeitsprojekt (keine Thesis). Unitree G1 mit 29 DOF in MuJoCo; 12 DOF und Gazebo sind entfernt (Tag `v1.9-review-abgeschlossen`).
- Layer 2 (Navigation) gehört einem Kollegen; `roboter_navigation` ist ein funktionierender Platzhalter.
- Greifen mit Dex3-Händen wird seit 11.11.2026 gebaut (Phase 1, Abschnitt „In Arbeit“); Machbarkeit gemessen: 0,30 m Reichweite, ≤ 1 kg je Hand im Stehen.

## Stand
Die Pipeline läuft in der Simulation über alle vier Schichten: Mission planen (PDDL/Fast Downward, Wegkosten über
Nav2) → navigieren (Nav2 mit 3D-Lidar; ohne Karte, mit SLAM oder auf fester Karte mit AMCL) → gehen und stehen
(RL-Policy, Zustandsautomat, NotAus, Freigabe, Störungserkennung) → Simulation (MuJoCo, realistische Sensorik
optional, Beinodometrie optional). Ein Befehl startet alles: `ros2 launch roboter_system system.launch.py ...`.

## In Arbeit: Greifen Phase 1 (Stand 12.11.2026, Branch `pascal/pipeline-v2.4`)
**Ziel:** der G1 holt eine Kiste (20 × 15 × 10 cm, 0,5 kg) vom Lagertisch, trägt sie mit Nav2 zum Arbeitsplatz und legt sie
auf der Markierung ab (Welt `greiftisch`, Mission `greiftisch`, Plan und Messwerte: `docs/plan_greifen_phase1.md`).
**Gebaut:** Handmodell (`scripts/erzeuge_handmodell.py`), Armführung neben dem Zustandsautomaten (`armfuehrung.py`, Arme der
Policy maskiert, Taille geneigt), Greif-Dienst (`greif_dienst_node.py`, Aktion `/greife`, Ablauf in `greifen/`), Hand-Kanal
und Objektposen in der Simulation, Layer 1 (`greife`/`ablegen`, `gegenstand` in der Mission), Lidar-Eigenfilter, Tests.
**Gemessen:** Regelkreis ohne ROS (`tests/simulation/greif_serie_zufall.py`, 12 zufällig verschobene Fälle): 12/12 gegriffen,
11/12 abgelegt (0–3 cm), kein Sturz. Greifen allein im ROS-System: 3/3. Voller ROS-Lauf mit Nav2
(`tests/system/integration_greifen.py --nav2`): einmal bis zum Ablegen durchgelaufen (2 cm), sonst Abbruch durch Nav2 unter Last.
**Nicht belegt / offen:** (1) ROS-Gesamtlauf ist noch nicht stabil: Nav2 bricht unter Last (2,8 GB RAM) ab („Failed to
transform end pose“, „Nav2 nicht aktiv“); keine Trefferquote über mehrere ROS-Läufe; (2) kein Testvideo (Vorbereitung steht,
`greif_aufnahme.py`, `--aufnahme`); (3) Finger-Kommandos (`/hand/kommando`) laufen am Layer 3 vorbei in die Simulation –
für den echten G1 braucht es einen geprüften Hand-Pfad (NotAus, Grenzen); (4) Mit Last dreht die Policy nur mit Seitschritt:
der Greif-Dienst dreht den Roboter selbst zum nächsten Ziel, bevor Nav2 übernimmt.
**Arbeitet gerade:** Nav2-Abbrüche auf dem Rückweg und Trefferquote im ROS-System.

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
| AkkuSchwach/Laden, Kamera | bewusst zurückgestellt (keine echten Batteriedaten bzw. nicht beauftragt) |
| Rechner: WSL2 mit 2,8 GB, Uhr geht nach | läuft seit dem Abspecken der Steuerung stabil; mehr Speicher bzw. Neustart der WSL entschärft beides (Verlauf 21.10.) |

## Wo steht was
| Thema | Datei |
|---|---|
| Starten, Schalter, Tests | `README.md` |
| Pakete, Schnittstellen, TF-Baum, Karte, Sicherheit, Entscheidungen | `docs/architektur.md` |
| Nav2-Platzhalter, Hinweise für die Variante des Kollegen | `src/roboter_navigation/README.md` |
| Missionen, Planer | `src/roboter_task_planning/README.md` |
| Greifen (Plan, Messwerte, Stand) | `docs/plan_greifen_phase1.md` |
| Welten, Karten | `src/roboter_welten/README.md` |
| Testarten und Aufrufe | `tests/README.md` |
| Einrichten eines Rechners | `scripts/einrichten.sh --pruefen` |
| Verlauf mit allen Messwerten | `docs/statusupdate_g1_projekt.md` |
| Fremdbestandteile und Lizenzen | `THIRD_PARTY.md` |
| G1 bestellen und konfigurieren (Checkliste für Unitree/Händler) | `docs/g1_hardware_checkliste.md` |

## Arbeitsweise
Messbare Behauptungen belegen, „getestet“ von „angenommen“ trennen. Jeder Schritt: komplette Testserie, Commit, Tag
(`git tag`), Push. Sprache von Kommentaren und Doku: Deutsch.
