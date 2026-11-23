# Welt `libero_kueche`

Küche mit Fixtures und Objekten aus [LIBERO](https://github.com/Lifelong-Robot-Learning/LIBERO) (MIT-Lizenz,
Copyright (c) 2023 Lifelong Robot Learning). Gedacht als Umgebung zum Testen der Aufgabenplanung mit Aufgaben im
Stil der LIBERO-BDDL-Dateien. Die Teile heißen wie dort (`flat_stove_1`, `wooden_cabinet_1`, ...), damit sich die
Aufgaben 1:1 übertragen lassen.

| Datei | Herkunft |
|---|---|
| `raum.xml` | von Hand: Wände, Küchenzeile, Hängeschränke, Kühlschrank-Korpus, Esstisch, Ortsmarkierungen |
| `aufbau.yaml` | von Hand: welche LIBERO-Teile wo stehen |
| `szene.xml` | **erzeugt** von `roboter_welten/libero_import.py` aus `raum.xml` + `aufbau.yaml` |
| `libero/` | **erzeugt**: die benutzten Meshes und Texturen aus LIBERO (Texturen auf höchstens 1024 px verkleinert) |
| `orte.yaml` | von Hand: Orte für Missionen |
| `karte.yaml`, `karte.pgm` | **erzeugt** von `karte_aus_szene.py` |

## Inhalt

| Instanz | LIBERO-Asset | Gelenke |
|---|---|---|
| `flat_stove_1` | `articulated_objects/flat_stove.xml` | `flat_stove_1_button` (Drehknopf, an ab 0.5 rad) |
| `wooden_cabinet_1` | `articulated_objects/wooden_cabinet.xml` | `wooden_cabinet_1_{top,middle,bottom}_level` (Schubladen, offen ≤ −0.14 m) |
| `microwave_1` | `articulated_objects/microwave.xml` | `microwave_1_microjoint` (Tür, offen ≤ −1.3 rad) |
| `moka_pot_1`, `plate_1`, `akita_black_bowl_1`, `white_yellow_mug_1`, `ketchup_1`, `butter_1` | `stable_scanned_objects/`, `turbosquid_objects/`, `stable_hope_objects/` | je ein freies Gelenk `<name>_frei` |

Die Schwellen für offen, geschlossen, an und aus sind die Werte aus LIBERO (`envs/objects/articulated_objects.py`).
LIBERO ist auf Tischmaßstab gebaut. Der Schubladenschrank ist etwa 26 cm groß und die Mikrowelle etwa 35 cm, wie im Original.
Die Regionen aus den BDDL-Dateien (`top_region`, `cook_region`, `heating_region`, ...) sind als Sites mit Präfix
enthalten, z. B. `wooden_cabinet_1_top_region`.

## Neu erzeugen

Nach einer Änderung an `raum.xml` oder `aufbau.yaml` (aus dem Projekt-Root):

```bash
python3 src/roboter_welten/roboter_welten/libero_import.py libero_kueche   # braucht LIBERO unter ~/tools/LIBERO
python3 src/roboter_welten/roboter_welten/karte_aus_szene.py libero_kueche
colcon build --packages-select roboter_welten
```

Zur Laufzeit wird LIBERO nicht gebraucht, nur zum Erzeugen. Klonen mit
`git clone https://github.com/Lifelong-Robot-Learning/LIBERO ~/tools/LIBERO` oder Pfad über `LIBERO_PFAD` setzen.
Was der Import anpasst (Masse der Mikrowellentür, doppelter Texturname am Herd, Kollisionsgruppe, Texturgröße),
steht im Kopf von `libero_import.py`.

## Hinweise

- Fixtures und Objekte haben Gelenke. Das Modell hat dadurch mehr `qpos`-Einträge als der Roboter.
  `mujoco_node` liest die Robotergelenke deshalb über die Aktuatoren und nicht mehr als `qpos[7:]`.
- Die LIBERO-Kollisionsquader liegen in geom group 3. Sie sind für Viewer, Kamera, Lidar und Karte unsichtbar.
- Systemtest: `python3 tests/system/integration_mission.py --kueche`. Mission: `roboter_task_planning/missions/libero_kueche.yaml`.
