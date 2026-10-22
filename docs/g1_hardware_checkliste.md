# G1-Hardware: Bestellung und Konfiguration (Checkliste)

Stand 22.10.2026. Zum Weitergeben an Unitree, einen Händler oder den Kollegen. „Prüfen“ heißt: aus Unitree-Beispielen
oder -Dateien bekannt, aber noch nicht am Roboter bestätigt.

## Womit wir bisher arbeiten (Simulation)

| | Stand | Quelle |
|---|---|---|
| Lauf-Policy | vortrainiert von Unitree, nicht selbst trainiert: `unitree_rl_lab`, `deploy/robots/g1_29dof/config/policy/velocity/v0` | `THIRD_PARTY.md` |
| Trainiert auf | **`g1_29dof_rev_1_0`**: 29 Gelenke, Taille mit 3 Gelenken (frei), Arme mit 7 Gelenken, **ohne Hände** | `unitree_rl_lab/assets/robots/unitree.py` (`UNITREE_G1_29DOF_CFG`), geprüft 22.10.2026 |
| Verstärkungen kp/kd | identisch mit Unitrees `deploy.yaml` dieser Policy | `roboter_steuerung/config/policy_29dof.yaml`, geprüft 22.10.2026 |
| Modell unserer Simulation | **`g1_29dof`** (ältere Revision, bei Unitree „Beta“, `mode_machine` 2) | `roboter_beschreibung/config/roboter.yaml` |

Unterschied `g1_29dof` → `g1_29dof_rev_1_0` (beide Dateien liegen in `modelle/g1_description/`): Hüft-Rollgelenk 88 → 139 Nm
(Übersetzung 14,5 → 22,5), Gesamtmasse 35,1 → 33,3 kg (Torso 1,8 kg leichter), `mode_machine` 2 → 5. Die Simulation lief mit
dem älteren Modell stabil, sollte aber vor dem Einsatz auf `g1_29dof_rev_1_0` umgestellt werden (Modell des Trainings und
aktueller Roboter).

## Bestellkonfiguration

Bestellt wird über den Unitree-Vertrieb oder einen autorisierten Händler; die EDU-Ausführungen werden per Angebot
konfiguriert, nicht im Online-Shop. Die Ausführungen G1 und G1 Pro unterstützen laut Unitree **keine Weiterentwicklung**
(kein Low-Level-Zugriff) und sind für dieses Projekt ungeeignet.

Im Angebot ausdrücklich festhalten:

| Anforderung | Warum |
|---|---|
| **G1 EDU** mit Low-Level-Zugriff (`unitree_sdk2`, ROS 2) | Unsere Steuerung sendet Gelenkziele selbst (Low-Level), nicht über Unitrees Laufsteuerung |
| **29 Gelenke**: Beine je 6, **Taille 3 Gelenke, nicht gesperrt**, Arme je 7 | Policy und Modell sind dafür gebaut; eine gesperrte Taille („lock waist“) oder 23 Gelenke passen nicht |
| **Hardware-Revision 1.0** (`g1_29dof_rev_1_0`, `mode_machine` 5) bestätigen lassen | Auf dieses Modell ist die Policy trainiert |
| **Entwicklungsrechner** an Bord (Jetson-Klasse) inkl. Betriebssystem/ROS-Version | Entscheidet, ob unser Stack (Ubuntu 24.04, ROS 2 Jazzy) an Bord laufen kann oder extern |
| **Lidar Livox MID-360** (Kopf) | Unsere Navigation und Sensorik setzen ein 3D-Lidar voraus |
| **Mit Händen** (Entscheidung 22.10.2026): `g1_29dof_with_hand_rev_1_0`, je Hand 7 Gelenke, gleiche 29 Körpergelenke, `mode_machine` 5 | Für späteres Greifen. Die Lauf-Policy ist ohne Hände trainiert; die Hände wiegen zusammen ~1 kg (34,4 statt 33,3 kg). Vor dem Einsatz in unserer Simulation mit diesem Modell prüfen; trägt die Policy das nicht, braucht es eine eigene (`unitree_rl_lab`) |
| Ersatzakku, Ladegerät, Fernbedienung, **Hängegestell** | Erste Tests nur hängend; Not-Halt über die Fernbedienung |

Fragen an Unitree/Händler: welches Handmodell (z. B. Dex3-1) und ob eine Lauf-Policy für die Hand-Version mitgeliefert wird; ob sich eine Basis-Ausführung später zu EDU erweitern lässt
(laut einer Händlerübersicht nicht); Garantie und Bedingungen bei Low-Level-Betrieb; Lieferzeit.

## Vom Roboter zu ermitteln (vor der ersten Bewegung)

| Was | Wohin bei uns |
|---|---|
| Firmware- und SDK-Version; `LowCmd`/`LowState` (unitree_hg), `mode_machine`, `mode_pr` (Sprunggelenke), CRC | neue Unitree-Bridge (`IRobotHardwareBridge`) |
| Reihenfolge, Vorzeichen und Nullstellungen der 29 Motoren (laut Unitree = unsere Reihenfolge; prüfen, im Hängen Gelenk für Gelenk) | `MUJOCO_JOINT_ORDER` in `rl_locomotion_controller_29dof.py` |
| Takt von `LowState` (vermutlich 500 Hz; prüfen) | `steuerung.yaml`: `sensor_periode_s` |
| IMU in `LowState`: Becken oder Torso, Einbaulage, Quaternion-Reihenfolge | Unitree-Bridge (Torso → mit den Taillenwinkeln auf das Becken umrechnen) |
| Verhalten ohne Kommandos (dämpfen/halten, nach welcher Zeit); Ablauf für den Low-Level-Zugriff (Debug-Modus bzw. Sportmodus freigeben) | `docs/architektur.md`, Anforderungen an die Hardware-Anbindung |
| Physischer Not-Halt am Gerät | Betriebsanweisung |
| Einbaulage des MID-360 (Elternglied, Position, Neigung) | `roboter.yaml` → `lidar` |
| Livox-Treiber: IP, Topic, `PointCloud2`, `frame_id`, Zeitsynchronisation | `roboter_sensorik/launch/sensorik.launch.py` |
| Standhöhe des Beckens | `roboter.yaml` → `becken_hoehe_stand` |
| Netz: IP-Adressen, Netzwerkschnittstelle, DDS (Unitree: CycloneDDS; wir bisher Fast DDS), Zeitabgleich (chrony) | Startskripte, Umgebung |
| Später: Akkudaten (BMS), Fußkraftsensoren falls vorhanden | Zustände AkkuSchwach/Laden, Beinodometrie |

## Danach in der Software
- Unitree-SDK-Bridge schreiben (`LowState` lesen → Gelenke + IMU; `LowCmd` senden mit kp/kd aus `/antrieb/parameter`,
  CRC, Modus). Steuerung, Sicherheit und Navigation bleiben unverändert.
- Karte der Halle: einmal mit SLAM abfahren und speichern, dann `karte:=<datei>` (Ablauf getestet:
  `tests/system/integration_mission.py --aufnehmen`); Orte der Mission in dieser Karte festlegen.
