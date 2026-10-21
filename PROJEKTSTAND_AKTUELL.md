# Projektstand aktuell — G1-Humanoid (Stand 21.10.2026, v2.7)

**Wozu diese Datei:** der aktuelle Stand auf einer Seite, für Übergaben und für jeden, der neu einsteigt. Sie wird bei jedem
größeren Schritt aktualisiert und enthält nur den Stand und die offenen Punkte; alles Weitere steht genau einmal an
seinem Ort (Tabelle unten). Was sich seit dem 10.10. geändert hat: `NEU_SEIT_10-10_FUER_NAV2_KOLLEGEN.md`.

## Rahmen
- Arbeitsprojekt (keine Thesis). Unitree G1 mit 29 DOF in MuJoCo; 12 DOF und Gazebo sind entfernt (Tag `v1.9-review-abgeschlossen`).
- Layer 2 (Navigation) gehört einem Kollegen; `roboter_navigation` ist ein funktionierender Platzhalter.
- Greifen/Hände bewusst zurückgestellt (Machbarkeit gemessen: 0,30 m Reichweite, ≤ 1 kg je Hand im Stehen).

## Stand
Die Pipeline läuft in der Simulation über alle vier Schichten: Mission planen (PDDL/Fast Downward, Wegkosten über
Nav2) → navigieren (Nav2 mit 3D-Lidar; ohne Karte, mit SLAM oder auf fester Karte mit AMCL) → gehen und stehen
(RL-Policy, Zustandsautomat, NotAus, Freigabe, Störungserkennung) → Simulation (MuJoCo, realistische Sensorik
optional, Beinodometrie optional). Ein Befehl startet alles: `ros2 launch roboter_system system.launch.py ...`.

## Tests (21.10.2026, alle grün)
`bash tests/alle_tests.sh --mit-slam`: pytest 86, Pipeline 37/37 (auch realistisch + Beinodometrie, mit Viewer),
Watchdog 8/8, Missionen 17/17 (Testdouble, Testwelt, Patrouille, 3D-Palette), feste Karte mit AMCL 21/21, SLAM
realistisch 25/25. Schnelle Tests zusätzlich
bei jedem Push auf GitHub (`.github/workflows/tests.yml`).

## Offen
| Punkt | Warum offen |
|---|---|
| Unitree-SDK-Bridge für echte Hardware | Schnittstelle (`IRobotHardwareBridge`, `/antrieb/parameter`) steht; Anforderungen in `docs/architektur.md` (Sicherheit) |
| Echte Lidar-Montage eintragen | Pose des MID-360 (Kopf, geneigt) aus Unitree-Datenblatt in `roboter_beschreibung/config/roboter.yaml` |
| Karte der echten Halle + Orte (Teach-in) | heute: Karten aus der Simulationsszene; für die Halle per SLAM aufnehmen (`docs/architektur.md`, Karte) |
| Nav2-Variante des Kollegen | liegt noch nicht vor; Einbindung: `navigation:=nav2 nav2_params:=...` oder `navigation:=extern` |
| SysML-Modell im Repo + Rückverfolgbarkeit | Modelldateien liegen nicht vor (vorgesehen: `modell/`) |
| Lizenzfrage eigener Code / Weitergabe | Entscheidung des Arbeitgebers; Fremdlizenzen in `THIRD_PARTY.md` |
| AkkuSchwach/Laden, Kamera, Greifen | bewusst zurückgestellt (keine echten Batteriedaten bzw. nicht beauftragt) |
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

## Arbeitsweise
Messbare Behauptungen belegen, „getestet“ von „angenommen“ trennen. Jeder Schritt: komplette Testserie, Commit, Tag
(`git tag`), Push. Sprache von Kommentaren und Doku: Deutsch.
