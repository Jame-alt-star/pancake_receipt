# Plan Greifen Phase 1 (Stand 11.11.2026)

Ziel: Der G1 holt in der MuJoCo-Simulation eine Kiste (0,20 x 0,15 x 0,10 m, 0,5 kg) mit beiden Dex3-Haenden von einem
Lagertisch, traegt sie zum Arbeitsplatz und stellt sie dort ab; gesteuert ueber die Mission (PDDL `greife`/`ablegen`).

## Messungen vor dem Bau (Regelkreis ohne ROS)
- Handmodell `g1_29dof_mit_haenden.xml` (erzeugt aus unserem Koerpermodell + Unitree-Dex3, `scripts/erzeuge_handmodell.py`):
  Lauf-Policy steht 20 s und geht 0,4/0,6 m/s praktisch wie ohne Haende.
- Tragen: Arme in Greif-/Hebepose (0,30 m vor dem Becken, y +-0,15 m, +0,15 m), Policy blind fuer Arme, Taille Pitch
  -0,15 rad: 12 s Gehen mit 0,25 und 0,5 kg je Hand bei 0,2/0,3/0,4 m/s ohne Sturz, max. Neigung 9,6 Grad.
  Ohne Maskierung Sturz. -> Maskierung auch beim Gehen, solange die Arme gefuehrt werden (abweichend von
  docs/statusupdate_g1_projekt.md, dort nur im Stehen gemessen).
- Dex3-Geometrie (Handgelenk-Frame): Finger zeigen nach +x, schliessen links nach -y, rechts nach +y; Handflaechen-
  Kollisionsbox 8,4 cm vor dem Handgelenk, Innenseite ~4 cm neben der Handgelenkachse. Alle 7 Fingerglieder je Hand
  haben eine Kollisionsgeometrie (je Glied ein Sicht- und ein Kollisionsnetz).

## Design
Layer 4 `mujoco_node`: Parameter `haende` (Modell mit Haenden), Gelenke ueber Aktuator->Gelenk-Adressen statt qpos[7:];
`/joint_states` und `/antrieb/parameter` bleiben bei den 29 Koerpergelenken; neu `/hand/joint_states` (14),
`/hand/kommando` (`roboter_interfaces/Handkommando`: q, kp, kd je Fingergelenk, wie Unitrees Dex3-HandCmd),
`/objekt/pose` (Ground-Truth der Kiste, nur Simulation, wie `/base_position`).

Layer 3 `roboter_steuerung`:
- ROS-freier Kern `greifen/` (IK 6D mit gedaempften kleinsten Quadraten auf dem MuJoCo-Kinematikmodell, Ablauf der
  Greifphasen, Feinpositionierung mit Totzone der Policy: Gehen erst ab ~0,3 m/s, Drehen erst ab ~0,8 rad/s).
- Zustand `Manipulieren` in `steuereinheit_node`: aktiv, solange `/arme/kommando` (JointState, 14 Armgelenke)
  kommt; Policy blind fuer Arme, Armziele ueberschreiben die Policy, Taille -0,15 rad proportional zur Armauslenkung,
  Gehen erlaubt (Tragen); Positionshaltung im Stand; beim Ende Arme weich zurueck (Ausblenden). NotAus haelt die Arme.
- `greif_dienst_node`: Aktion `/greife` (`GreifAktion`: objekt, modus greifen|ablegen, Ablagepose), Feinpositionierung
  ueber Vorrang-Eingang `/cmd_vel/greifen`, sendet `/arme/kommando` und `/hand/kommando`, Erfolg ueber `/objekt/pose`.

Layer 1: Domaene um `greife ?o`/`ablegen ?o` (Praedikate `liegt_bei`, `haelt`, `haende_frei`, `abgelegt`) erweitert;
Missionsblock `gegenstand: {name, ort, ablage}`; Zielart `abgelegt`; `GreifBridge`. Fehlgriff -> Neuplanung
(gehe_zu zum Ort, erneut greifen) ueber das vorhandene `max_replans`.

Welt `greiftisch`: Lagertisch und Arbeitstisch (Tischplatte 0,74 m), Kiste auf dem Lagertisch; Orte = Vorpositionen
0,85 m vor den Tischen (Nav2: xy-Toleranz 0,25 m, Roboterradius 0,36 m, Inflation 0,55 m -> naeher geht nicht), den
Rest macht die Feinpositionierung.

## Schritte
1. Handmodell + roboter.yaml (erledigt: Modell, Pruefung Policy).
2. Welt greiftisch.
3. ROS-freier Kern + Regelkreistest (Greifen, Heben, Tragen, Ablegen mit echter Kiste).
4. Zustand Manipulieren + Unit-Tests.
5. Layer 4 Haende/Objektpose, Interfaces, greif_dienst_node.
6. Layer 1 + GreifBridge + Missionsdatei.
7. Systemtest mit Video; Doku.

Nach jedem Schritt: Review durch einen unabhaengigen Pruef-Agenten, Tests gruen, nichts pushen.
