# roboter_welten

Je Umgebung ein Ordner `welten/<name>/`:

| Datei | Wer liest sie | Inhalt |
|---|---|---|
| `szene.xml` | Simulation (Layer 4, `mujoco_node` Parameter `welt`) | nur die Umgebung (Wände, Regale, Hindernisse), **ohne Roboter**; das Robotermodell hängt die Simulation beim Laden dazu |
| `orte.yaml` | Missionsplanung (Layer 1) | benannte Orte `{x, y}` im map-Frame und ungerichtete Verbindungen |
| `karte.yaml` + `karte.pgm` | Navigation mit `karte:=true` (map_server + AMCL) | Belegungskarte, aus `szene.xml` gerechnet: `python3 src/roboter_welten/roboter_welten/karte_aus_szene.py [welt]`; für eine echte Halle stattdessen per SLAM aufnehmen und speichern |

Missionen (`roboter_task_planning/missions/*.yaml`) enthalten nur noch Ziele und Parameter und verweisen mit
`welt: <name>` auf die Welt. `roboter_system/launch/system.launch.py` startet die Simulation automatisch mit derselben Welt.

| Welt | Inhalt |
|---|---|
| `leer` | Ebene ohne Hindernisse (Pipeline-Tests, Nav2-Testdouble) |
| `testwelt` | zwei Räume mit Tür, ein Hindernis |
| `testwelt3d` | wie `testwelt`, zusätzlich niedrige Palette (nur mit 3D-Lidar sichtbar) |
| `demo_lager` | Lagerhalle mit Regalen und Kistenstapeln, Büro hinter einer Tür (Demo-Video) |

Konventionen in `szene.xml`: Hindernisse in geom group 2 (kollidieren und sind für das simulierte Lidar sichtbar),
reine Dekoration ohne Kollision (`contype`/`conaffinity` 0). Der Roboter startet bei (0, 0) mit Blick nach +x;
`map` = `odom` = diese Startpose, solange keine gespeicherte Karte genutzt wird.

Neue Welt: Ordner anlegen, beide Dateien schreiben, `colcon build`. Für die echte Halle kämen hier eine gespeicherte
Karte (`karte.yaml`/`.pgm`) und per Teach-in erfasste Orte hinzu; `szene.xml` entfällt dort.
