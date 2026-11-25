# Welt `robocasa_kueche7`

RoboCasa-Testküche **Layout 7 / Style 7** („U-shaped Island Layout / Mediterranean Style") aus
[RoboCasa](https://github.com/robocasa/robocasa). Der Code steht unter MIT-Lizenz, die Assets unter CC BY 4.0
(© RoboCasa-Autoren, UT Austin). Die Küche ist 6,4 × 5 m groß:
- U-förmige Küchenzeile mit Kochfeld, Dunstabzug und Fenster an der Rückwand
- links Backofen mit Mikrowelle und Toasterofen
- rechts Side-by-Side-Kühlschrank, Kaffeemaschine und Toaster
- in der Mitte eine Insel mit Spüle, Geschirrspüler und drei Barhockern

Insgesamt hat die Küche 121 Fixtures und 105 Gelenke (Türen, Schubladen, Klappen).

| Datei | Herkunft |
|---|---|
| `aufbau.yaml` | von Hand: Layout, Style, Seed, Startpose des Roboters in der Küche |
| `szene.xml` | **erzeugt** von `roboter_welten/robocasa_import.py` |
| `robocasa/` | **erzeugt**: die benutzten Meshes und Texturen aus RoboCasa (Texturen auf 512 px verkleinert, `aufbau.yaml: max_textur`; ~29 MB) |
| `fixtures.json` | **erzeugt**: alle Fixtures mit Klasse, Position und Größe (`pos` in Küchen-, `pos_map` in Kartenkoordinaten) |
| `orte.yaml` | von Hand: Standpunkte vor den Geräten, Ring um die Insel |
| `karte.yaml`, `karte.pgm` | **erzeugt** von `karte_aus_szene.py` |

## Wie die Szene entsteht

RoboCasa hat die Küche nicht als fertige Datei, sondern als Bauplan (Layout- und Style-YAML) und Einzelteile.
`robocasa_import.py` geht so vor:
1. Es ruft `robocasa_export.py` mit dem Python der RoboCasa-Umgebung auf. Das baut die Küche einmal mit
   `robosuite.make("Kitchen", layout_and_style_ids=[[7, 7]])` und speichert `env.sim.model.get_xml()`.
2. Es entfernt den RoboCasa-Roboter, die nicht benutzten Assets und die Physik-Optionen von RoboCasa. Gleiche
   Texturen, Materialien und Meshes führt es zusammen: RoboCasa legt sie je Fixture neu an, zum Beispiel 77 Texturen
   aus 28 Dateien.
3. Es kopiert die benutzten Dateien in den Ordner `robocasa/` und macht die Pfade relativ.
4. Es setzt die Gruppen (siehe unten), legt die Küche so, dass `aufbau.yaml: start` im Ursprung liegt, und schreibt `szene.xml`.

Zur Laufzeit wird RoboCasa nicht gebraucht.

## Gruppen

| Gruppe | Inhalt | sichtbar | Kollision | Lidar/Karte |
|---|---|---|---|---|
| 1 | Optik (Meshes, Texturen, Boden 2 mm angehoben) | ja | – | – |
| 3 | Kollisionskörper von RoboCasa | – | mit Roboter und Objekten (`contype 0`, `conaffinity 1`), **nicht untereinander** | – |
| 4 | je Küchenteil eine Ersatzbox um die Kollisionskörper (Türen zu) | – | – | ja (`lidar.py` tastet die Gruppen 2 und 4 ab) |

Gründe für diese Aufteilung, gemessen mit dem G1:
- Ohne diese Aufteilung dauerte ein Simulationsschritt etwa 1,1–1,5 ms und ein 3D-Lidar-Scan etwa 56 ms, mit den Optik-Meshes als Ziel.
- Mit ihr dauert ein Schritt etwa 0,65–0,85 ms und ein Scan etwa 21 ms. Der Scan hängt vor allem an der Gesamtzahl der Geoms. Zum Vergleich: in `demo_lager` dauert ein Schritt etwa 0,1 ms und ein Scan etwa 5 ms.
- Unsichtbare Geoms (rgba-Alpha 0) ignoriert der Raycast. Deshalb liegen die Ersatzboxen in einer eigenen Gruppe, die nicht gerendert wird.
- Türen und Schubladen bleiben auch ohne Kontakt zu ihrem Schrank zu, weil RoboCasa sie über Gelenkreibung und -dämpfung hält. Ihren Weg begrenzen die Gelenkanschläge.
- **Offene Türen sieht das Lidar nicht**, weil die Ersatzboxen den geschlossenen Zustand abbilden.

## Arbeitsspeicher

Dein WSL2 hat etwa 3,5 GB für die ganze Pipeline. So viel belegt der Simulationsprozess nach dem Laden (RSS):

| Stand | `robocasa_kueche7` | `demo_lager` |
|---|---|---|
| erster Import (Texturen doppelt, 1024 px) | 1656 MB | 385 MB |
| + Assets zusammengeführt | 1028 MB | – |
| + `malloc_trim` nach dem Kompilieren (`szene.py`) + Texturen 512 px | **718 MB** | **288 MB** |

## Neu erzeugen

Einmalig wird RoboCasa außerhalb des Projekts eingerichtet:
- Code: `~/tools/robocasa` und `~/tools/robosuite`
- Python-Umgebung: `~/tools/robocasa_venv` mit mujoco 3.3.1 und numpy 2.2.5, ohne PyTorch und LeRobot
- Assets: `python -m robocasa.scripts.download_kitchen_assets --type tex fixtures_lw objs_lw`, etwa 1,6 GB Download

Danach im Projekt-Root:

```bash
python3 src/roboter_welten/roboter_welten/robocasa_import.py robocasa_kueche7 [--neu]   # --neu: erneut exportieren
python3 src/roboter_welten/roboter_welten/karte_aus_szene.py robocasa_kueche7
colcon build --packages-select roboter_welten
```

Eine weitere RoboCasa-Küche legst du so an: neuen Ordner `welten/<name>/` mit `aufbau.yaml` (Layout, Style, Start),
dann `robocasa_import.py <name>` ausführen und `orte.yaml` schreiben.

## Test und Mission

- Systemtest: `python3 tests/system/integration_mission.py --robocasa`
- Mission: `roboter_task_planning/missions/robocasa_kueche7.yaml` (Ring um die Insel)
- Starten mit RViz: `ros2 launch roboter_system system.launch.py mission:=robocasa_kueche7 navigation:=nav2 auto_start:=true rviz:=true`
