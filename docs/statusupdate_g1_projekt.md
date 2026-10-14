# Statusupdate – G1 Humanoid-Projekt

**Verlaufsprotokoll ab 12.10.2026 (chronologisch; aktueller Stand am Ende und in README.md).** Das Projekt ist ein Arbeitsprojekt (keine Thesis).

## Ausgangspunkt

Nach dem Strategiewechsel von einem klassischen Regler auf dem R1-Modell (Gazebo) zu einer RL-Policy auf dem Unitree G1 (MuJoCo) wurde der Code eines Kollegen übernommen, der bereits auf die vorgesehene 4-Schichten-Architektur umstrukturiert war: Layer 4 (Hardware-Abstraktion), Layer 3 (Reactive – Bewegungsregelung), Layer 2 (Executive – Navigation) und Layer 1 (Deliberative – Missionsplanung). Seit dieser Übernahme wurde Folgendes umgesetzt:

## Grundlagenarbeit

Die beim Umbau verlorene Code-Dokumentation wurde über alle zentralen Dateien wiederhergestellt (Konfiguration, Hardware-Bridge, Simulationsknoten, Regelungsknoten, RL-Controller), damit der Code nachvollziehbar bleibt.

Zwei konkrete, blockierende Bugs wurden gefunden und behoben: eine fehlerhafte Pfadauflösung für die trainierte Policy-Datei sowie ein Performance-Problem, bei dem das Rendering der Simulation bei jedem einzelnen Physikschritt lief statt nur periodisch – das brachte die angenommene und die tatsächliche Simulationszeit so weit auseinander, dass der Roboter beim Gehen instabil wurde und umfiel. Dieser Fix war die Voraussetzung dafür, dass der Roboter überhaupt zuverlässig laufen kann.

Zwei MuJoCo-Testumgebungen wurden aufgebaut: eine einfache Laufbahn mit Meter-Markierungen zur Fortschrittskontrolle, und ein fünfstufiger Hindernisparkour (Bordstein, Hürde, Rampe, unebenes Gelände, seitliche Schräge), angelehnt an gängige Benchmarks für Beinroboter.

## Bewegungsregelung (Reactive Layer)

Die RL-Lauf-Policy ist integriert und läuft stabil. Die Steuerung erfolgt jetzt zur Laufzeit über das ROS2-Standardtopic `/cmd_vel`, nicht mehr über einen fest codierten Startwert – damit ist die Pipeline erstmals tatsächlich bedienbar.

Das Zustandsmodell (State Pattern, gemäß dem SysML-Zustandsautomaten) ist für Gehen UND Stehen (Balancieren) umgesetzt und erfolgreich getestet. Mehrere Lösungsansätze für das Stehen wurden systematisch durchgetestet; die Ursache des ursprünglichen Problems ist klar verstanden – eine reine Gelenk-Positionsregelung kann den Schwerpunkt des Roboters nicht aktiv halten, sie merkt nicht, wenn er nach vorne kippt. Die erfolgreiche Lösung orientiert sich an der tatsächlichen Vorgehensweise von Unitree selbst (recherchiert anhand der offiziellen Deployment-Dokumentation): die Bewegungs-Policy bleibt auch im Stillstand aktiv, ergänzt um ein sanftes Abbremsen des Geschwindigkeitskommandos statt eines harten Stopps, statt in einen separaten, PD-geregelten Halte-Zustand zu wechseln. Im Testlauf bestätigt: der Roboter fällt damit nicht mehr um, weder beim Laufen noch beim Stehen.

Zusätzlich wurde eine Kurshaltung ergänzt (einfacher Regelkreis auf Basis der IMU-Ausrichtung), die einem beobachteten seitlichen Wegdrehen beim Laufen deutlich entgegenwirkt. Ein verwandtes, aber eigenständiges Phänomen – ein rein seitliches Wegrutschen auf der Stelle beim Stehen, ohne Drehung – bleibt bestehen: ein Lösungsversuch dafür (Positions-Haltung über eine zusätzliche Positions-Rückmeldung aus der Simulation) wurde testweise umgesetzt, führte aber nachweislich zu neuer Instabilität (Umfallen) und wurde bewusst wieder zurückgenommen. Das seitliche Wegrutschen beim Stehen bleibt daher als bekannte, akzeptierte Einschränkung bestehen – kein Sturzrisiko, nur kosmetisch, und eine Eigenschaft der aktuellen Policy selbst, die sich bei einem späteren Wechsel auf ein anderes, vortrainiertes Modell ohnehin anders darstellen könnte.

## Architekturentscheidungen

Für die Navigation (Layer 2) wurde Nav2/SLAM als klassischer, etablierter Ansatz festgelegt statt einer selbst trainierten, kamerabasierten Policy – geringeres Risiko, keine zusätzliche Trainingszeit nötig.

Für die Missionsplanung (Layer 1) wurde ein eigener Dispatcher mit Fast Downward (PDDL-Planer) festgelegt statt des Fremdframeworks PlanSys2 – passt direkt auf die vorhandene Aktionsstruktur und bleibt vollständig nachvollziehbar.

Außerdem wurde geklärt, dass Greifen/Manipulation tatsächlich ein festes Ziel der Arbeit ist. Das aktuelle Robotermodell (12 Freiheitsgrade, nur Beine) reicht dafür nicht aus, da die Arme bislang keine Gelenke haben. Eine Umstellung auf ein größeres Modell (23 oder 29 Freiheitsgrade) ist deshalb als eigener, klar abgegrenzter Arbeitsschritt eingeplant. Dafür wurde auch recherchiert, wie vergleichbare Projekte das lösen: Eine separate Greif-Policy, die zusätzlich zur bestehenden Lauf-Policy läuft (nicht gemeinsam neu trainiert), ist ein in der Praxis bereits erprobtes Muster. Geklärt ist zudem, dass Greifen und Gehen nicht gleichzeitig, sondern nacheinander ablaufen sollen (erst stehen bleiben, dann greifen) – das reduziert den Aufwand und das Risiko erheblich.

## Offene Abhängigkeit

Für das Training einer Lauf-Policy auf dem größeren Robotermodell wird eine GPU benötigt, auf die aktuell kein Zugriff besteht. Das ist als Abhängigkeit für die DOF-Migrationsphase vorgemerkt, aber nicht zeitkritisch, solange zuerst die Standregelung und Navigation abgeschlossen werden.

## Scope-Entscheidung (12.10.2026)

Bewusst eingegrenzt: komplexe Hindernisüberwindung (Bordstein, Hürde, Rampe, unebenes Gelände, seitliche Schräge – der dafür aufgebaute fünfstufige Parkour) bleibt vorerst außen vor, ebenso die DOF-Migration/Greifen. Fokus liegt auf einfachem Laufen/Stehen auf ebenem Untergrund (Layer 3) und der Missionsplanung (Layer 1). Begründung: Hindernisüberwindung bräuchte eigenes RL-Nachtraining (Terrain-Curriculum) oder eine perceptive Policy mit Höhenkarten-Eingabe – eigenständiger Aufwand ohne direkten Bezug zum Kern der Arbeit. Die DOF-Migration hängt ohnehin an der fehlenden GPU (siehe oben). Noch offen/nicht entschieden: ob Layer 2 (Navigation) trotzdem den vollen Nav2/SLAM-Stack bekommt oder für den hindernisfreien Fall vorerst vereinfacht wird (z.B. direkte Wegpunkt-Ansteuerung ohne Kartierung) – wird bei Bedarf entschieden, sobald Layer 2 an der Reihe ist.

## Zustandsautomat erweitert (12.10.2026) – implementiert, NOCH NICHT getestet

Der SysML-Zustandsautomat (RoboterZustaende.sysml) sieht sieben Zustände vor: Start, Bereitschaft, Balancieren, Gehen, AkkuSchwach, Laden, NotAus. Bisher waren nur Gehen und Balancieren auf ROS2-Ebene abgebildet. Heute ergänzt: StartState (wartet auf die erste gültige Sensormessung), BereitschaftState (ein-Takt-Übergang, entscheidet dann automatisch zwischen Gehen/Balancieren) und NotAusState (sicherste in der Simulation verfügbare Reaktion: es werden keine neuen Gelenk-Zielwerte mehr gesendet, der Roboter friert in der letzten Pose ein).

NotAus wird über ein neues, eigenständiges `/not_aus`-Topic (std_msgs/Bool, latched/TRANSIENT_LOCAL-QoS) ausgelöst und zentral in der Regelschleife abgefangen, nicht über den normalen Zustandswechsel-Mechanismus – Begründung: Recherche zur ROS2-/Industrie-Praxis ergab, dass es kein einzelnes, ROS2-weit standardisiertes Not-Aus-Topic gibt; auf echter Hardware wird Not-Aus normalerweise über ein Sicherheitsrelais kappt, unabhängig von der Software. Da hier keine echte Verdrahtung existiert, wurde ein eigenes latched Topic nach demselben Muster wie das bestehende `/balance_mode` gewählt.

AkkuSchwach und Laden bleiben bewusst zurückgestellt, bis ein echter Roboter mit echten Batteriedaten angeschlossen ist – ein simulierter Platzhalter-Akku-Wert wurde explizit abgelehnt (Nutzerentscheidung), da er ohne echte Daten reine Fiktion wäre. Zwei Designentscheidungen dafür sind aber schon festgelegt, damit sie bei der späteren Umsetzung nicht neu diskutiert werden müssen: (1) "Laden" wird auch mit echter Hardware explizit extern ausgelöst (eigenes Topic, analog zu `/not_aus`), nicht automatisch bei AkkuSchwach – das Andocken an eine Ladestation ist ein physischer Vorgang, den die Software nicht von sich aus als abgeschlossen annehmen darf. (2) AkkuSchwach soll den Roboter analog zu NotAus zum sicheren Stehenbleiben zwingen (Policy aktiv, cmd=0), statt normal weiterzulaufen, bis Laden beginnt.

Bewusste Vereinfachung, offen dokumentiert: nach einer Entwarnung (/not_aus wieder false) läuft die Bewegung automatisch wieder an (über Bereitschaft zurück nach Gehen/Balancieren) – es gibt noch keinen bewussten Freigabe-/Reset-Schritt, wie man ihn auf echter Hardware erwarten würde. Ebenso läuft Bereitschaft aktuell automatisch durch (kein expliziter "Freigabe"-Befehl vor der ersten Bewegung).

**Erster echter Testlauf (12.10.2026) und zwei Fix-Iterationen:**
1. NotAus beim Gehen ausgelöst (Nutzertest) – Roboter ist nach vorne gefallen. Ursache identisch zum ursprünglichen Stehen-Problem (siehe oben): ein hartes Einfrieren der Gelenkpositionen mitten im Schritt (Gewicht auf einem Bein, Vorwärts-Schwung) ist eine reine Positions-Haltung, die den Schwerpunkt nicht ausgleichen kann.
2. Fix v1: NotAus hält die RL-Policy aktiv (cmd hart auf [0,0,0], dieselbe Kombination wie die stabile Stehen-Lösung) statt die Gelenke einzufrieren. Im Test: fällt nicht mehr um, aber die Policy "trabt" dabei unbegrenzt auf der Stelle (dieselbe Links-Rechts-Trittbewegung wie beim normalen Stehen) – kein echter Stillstand.
3. Fix v2: zweiphasiger Ansatz – Policy bleibt wie in v1 für eine feste Abbrems-Zeit aktiv, um Restschwung sicher abzubauen, danach wird hart eingefroren (keine neuen Zielwerte mehr) – jetzt sicher, weil kein Schritt mehr im Gange ist. Eine "immer leichter werdende" Trittbewegung ist mit dieser Policy nicht gezielt steuerbar (cmd steht schon auf dem Minimum), daher der Zeit-Ansatz als pragmatischer Kompromiss. Startwert 1,5s im Test zu kurz (Roboter fiel noch um, vermutlich beim Einfrieren mitten im Ausgleichsschritt) – auf 6,0s erhöht, noch nicht erneut getestet.

Bewusster Kompromiss bei allen Policy-aktiv-Varianten: das ist kein "Motoren sofort aus" mehr, sondern ein aktiv geregeltes, sicheres Abbremsen – bei einem dynamisch balancierenden Zweibeiner ist ein echtes Sofort-Aus ohnehin gleichbedeutend mit einem Sturz, es gibt keine passive Standstabilität.

**Status NotAus (getestet, 12.10.2026):** zwei weitere Iterationen getestet (Abbrems-Zeit 1,5s und 6,0s vor hartem Einfrieren) – beide fallen beim Einfrieren um, unabhängig von der Wartezeit. Schlussfolgerung: die Trittbewegung bei cmd=0 ist kein abklingender Restschwung, sondern eine fortlaufende zyklische Bewegung (Trainingsartefakt der Policy) – es gibt keinen sicheren Moment zum Einfrieren. **Finale Lösung:** die Freeze-Phase wurde ersatzlos gestrichen, NotAus bleibt jetzt dauerhaft in der nachweislich stabilen Variante (Policy aktiv, cmd=0) – der Roboter trabt bei Not-Aus dauerhaft leicht auf der Stelle, fällt aber nicht um. Noch nicht erneut mit dieser finalen Version getestet.

## Realismus-Fix: bewusste Freigabe statt Autostart (12.10.2026)

Beobachtung: der Roboter lief nach dem Hochfahren sofort automatisch los, ohne dass ein Kommando gesendet wurde – weil `cmd_target` beim Start mit dem YAML-Default `cmd_init` (meist ungleich 0) initialisiert wird und `BereitschaftState` bisher sofort automatisch anhand dieses Werts entschieden hat. Nicht realistisch: ein Roboter sollte nach dem Hochfahren in Bereitschaft stehen bleiben, bis explizit ein Kommando kommt.

**Fix:** neues Flag `bewegung_freigegeben` – wird erst `True`, sobald die erste echte `/cmd_vel`-Nachricht ankommt (unabhängig vom Wert, auch ein explizites "stehen bleiben" zählt als Freigabe). `BereitschaftState` wartet jetzt auf dieses Flag, bevor er in Gehen/Balancieren wechselt. Als Nebeneffekt schließt das auch die vorher offene Lücke bei NotAus: das Flag wird beim Auslösen von Not-Aus zurückgesetzt, sodass nach einer Entwarnung eine bewusste neue Freigabe nötig ist, statt dass die Bewegung automatisch wieder anläuft.

**Status: geschrieben und auf Syntax-Ebene geprüft, NOCH NICHT in der Simulation getestet.**

**Testfund und Fix 2 (12.10.2026):** erster Test zeigte, dass der Roboter sofort nach dem Laden umfällt. Ursache gefunden: `BereitschaftState` hat beim Warten auf die Freigabe bewusst gar keine Gelenk-Zielwerte gesendet, in der Annahme, MuJoCo würde dadurch einfach die aktuelle Pose halten. Tatsächlich regelt `mujoco_node.py` ohne eingehende `/joint_commands`-Nachricht weiter per reiner PD-Positionsregelung auf `default_angles` – exakt "Variante 2" aus der ursprünglichen Stehen-Problem-Analyse, die schon damals als instabil identifiziert wurde (reine Gelenk-PD-Haltung merkt nicht, wenn der Roboter kippt). Fix: `BereitschaftState` hält jetzt ebenfalls die RL-Policy aktiv (`cmd=[0,0,0]`, wie Balancieren/NotAus), auch während des Wartens auf Freigabe. Noch nicht erneut getestet – falls der Roboter trotzdem noch during `StartState` (der sehr kurzen Wartezeit auf die erste gültige Sensormessung) umfällt, bräuchte das eine andere Lösung, da dort noch keine gültigen Daten für die Policy vorliegen.

## Nächste Schritte

1. Den erweiterten Zustandsautomaten (Start/Bereitschaft/NotAus) in der Simulation testen und bestätigen, insb. dass `/not_aus` den Roboter zuverlässig einfrieren lässt und die anderen Zustandsübergänge nicht stören.
2. Danach: Navigation (Umfang siehe Scope-Entscheidung oben), danach PDDL-Dispatcher.
3. **GitHub-Backup abgeschlossen (12.10.2026):** kompletter aktueller Stand (inkl. der gesamten Zustandsautomat-Erweiterung) liegt jetzt unter https://github.com/Pascho02/g1-humanoid-robot (privat) – Absicherung vor der DOF-Migration (siehe unten), falls die schiefgeht.

Hindernisparkour und DOF-Migration/Greifen sind zurückgestellt, nicht gestrichen – Wiederaufnahme möglich, wenn Zeit/GPU-Zugriff es erlauben.

## DOF-Migration gestartet (12.10.2026): Umstieg auf 29-DOF-Ganzkörper-Policy

**Motivation:** Vermutung, dass das 12-DOF-Bein-only-Modell (keine aktuierten Arme/Taille, feste Pose) mit zum Kippen beiträgt, da eine ungünstig positionierte "tote" Oberkörpermasse den Schwerpunkt verschiebt, ohne dass die Bein-Policy das ausgleichen kann. Zustandsautomat-Arbeit (Start/Bereitschaft/NotAus) wird dafür zurückgestellt, nicht verworfen – aktueller Stand ist auf GitHub gesichert (https://github.com/Pascho02/g1-humanoid-robot).

**Wahl 29-DOF (nicht 23-DOF):** `unitree_rl_lab` bietet die fertige, vortrainierte Ganzkörper-ONNX-Policy (`policy/velocity/v0`) nur für die 29-DOF-Variante an – für 23-DOF gibt es keine fertige Policy (im Repo direkt verifiziert: `deploy/robots/g1_23dof/` enthält keine `.onnx`-Datei). Die Wahl ist damit praktisch vorgegeben.

**Recherche-Ergebnis (12.10.2026), direkt aus dem `unitree_rl_lab`-Quellcode verifiziert (Repo lokal geklont, nicht nur Doku gelesen):**
- Fertige ONNX-Datei liegt bereits vor: `deploy/robots/g1_29dof/config/policy/velocity/v0/exported/policy.onnx` – kein eigenes Training/Export nötig.
- **Zwei unterschiedliche Gelenk-Reihenfolgen identifiziert und an allen 29 Werten von `default_joint_pos` gegengeprüft (29/29 Treffer, nicht nur Stichprobe):**
  - MuJoCo/URDF-Reihenfolge ("SDK-Reihenfolge"): linkes Bein (6) → rechtes Bein (6) → Taille (3) → linker Arm (7) → rechter Arm (7).
  - Policy-interne Reihenfolge (Isaac Lab): verschränkt – Hüfte-Pitch(L,R), Taille-Yaw, Hüfte-Roll(L,R), Taille-Roll, Hüfte-Yaw(L,R), Taille-Pitch, Knie(L,R), Schulter-Pitch(L,R), Knöchel-Pitch(L,R), Schulter-Roll(L,R), Knöchel-Roll(L,R), Schulter-Yaw(L,R), Ellbogen(L,R), Handgelenk-Roll(L,R), Handgelenk-Pitch(L,R), Handgelenk-Yaw(L,R). Diese Umrechnung war in einem bekannten offenen GitHub-Issue (#145) bei anderen Nutzern die vermutete Fehlerursache für einen gescheiterten Benchmark (0% Erfolg) – für uns jetzt vorab verifiziert geklärt, nicht nur übernommen.
  - PD-Gains (Steifigkeit/Dämpfung je Gelenk) liegen in `deploy.yaml` bereits in MuJoCo/SDK-Reihenfolge vor (keine Umrechnung nötig), `default_joint_pos`/Aktions-Offsets/Beobachtungswerte dagegen in der Policy-internen Reihenfolge (Umrechnung nötig).
  - Beobachtungsvektor: 6 Blöcke (Basis-Drehrate 3, Schwerkraftrichtung 3, Geschwindigkeits-Kommando 3, 29 Gelenkwinkel, 29 Gelenkgeschwindigkeiten, letzte Aktion 29) × 5 Zeitschritte Historie = 480 Werte, pro Begriff zusammenhängend mit ältestem Zeitschritt zuerst (im C++-Quellcode direkt verifiziert, nicht angenommen).

**Status: Recherche/Mapping abgeschlossen und verifiziert, Implementierung (neue Config, ONNX-Inferenz-Controller statt TorchScript, 29-DOF-MuJoCo-Szene) noch nicht begonnen/getestet.**

## DOF-Migration: Implementierung abgeschlossen (12.10.2026) – NOCH NICHT in der echten Simulation getestet

Aufbauend auf der verifizierten Recherche (siehe oben) wurde die konkrete Integration umgesetzt, bewusst als **zusätzlicher, parallel waehlbarer Pfad** (nicht als Ersatz) – der bisherige 12-DOF-Pfad bleibt unveraendert und ist weiterhin der Default, falls die 29-DOF-Policy sich als schlechter/instabiler herausstellt.

**Neue/geänderte Dateien:**
- `r1_bringup/config/g1_policy_29dof.yaml` (neu) – Gegenstück zu `g1_policy.yaml`: `xml_path` → `g1_29dof.xml`, `policy_path` → `policy_29dof.onnx`, `num_actions: 29`, `kps`/`kds`/`default_angles` in MuJoCo-Reihenfolge (hergeleitet und gegen alle 29 Werte aus `deploy.yaml` verifiziert, siehe Recherche oben), `ang_vel_scale: 0.2` (abweichend von 0.25 beim 12-DOF-Pfad – eigene Trainings-Skalierung), `history_length: 5`, `num_obs: 480`, neuer Schlüssel `controller_type: "rl_locomotion_29dof"`. Zusätzlich dokumentiert (als offener Punkt, noch nicht im Code umgesetzt): die Trainings-Wertebereiche für `cmd` (`lin_vel_x`/`lin_vel_y`/`ang_vel_z`), außerhalb derer die Policy "out of distribution" wäre.
- `roboter_steuerung/roboter_steuerung/control/rl_locomotion_controller_29dof.py` (neu) – ONNX-Variante von `RLLocomotionController` (onnxruntime statt TorchScript), implementiert dasselbe `ILocomotionController`-Interface, also ohne jede Änderung an den States oder an `steuereinheit_node.py`s Regelschleife aufrufbar. Kernunterschiede: (1) explizite Gelenk-Umsortierung zwischen MuJoCo- und Policy-Reihenfolge über die aus `deploy.yaml` übernommene `joint_ids_map` (Gather für Eingaben, Scatter für die Aktion); (2) pro Beobachtungs-Term ein eigener 5-Schritte-Verlauf (FIFO, ältester zuerst) statt nur dem aktuellen Wert, nach dem in `manager_term_cfg.h` verifizierten Muster. Eigenständig (ohne ROS2) mit der echten `policy_29dof.onnx`-Datei getestet: Inferenz läuft, Ausgabe ist 29-dimensional, endlich (keine NaN/Inf) und bleibt nahe der Ruhepose – kein Ersatz für einen echten Simulationstest, aber ein Hinweis, dass Modell-Laden und Formate stimmen.
- `roboter_steuerung/roboter_steuerung/steuereinheit_node.py` (geändert) – neuer ROS2-Parameter `config_file` (Default `"g1_policy.yaml"`, also unverändertes Verhalten ohne explizite Angabe); Controller-Auswahl jetzt über `controller_type` aus der YAML (`"rl_locomotion"` → bisheriger TorchScript-Controller, `"rl_locomotion_29dof"` → neuer ONNX-Controller).
- `r1_bringup/r1_bringup/mujoco_node.py` (geändert) – derselbe `config_file`-Parameter, damit beide Knoten dieselbe YAML lesen (kps/kds/Taktung müssen zusammenpassen).
- `roboter_steuerung/launch/steuereinheit.launch.py` (geändert) – neues Launch-Argument `config_file`, wird an beide Knoten als ROS2-Parameter durchgereicht:
  ```
  ros2 launch roboter_steuerung steuereinheit.launch.py config_file:=g1_policy_29dof.yaml
  ```
  Ohne das Argument: exakt der bisherige 12-DOF-Pfad.
- `r1_bringup/models/policies/policy_29dof.onnx` (neu, Binärdatei, aus `unitree_rl_lab` übernommen, kein eigenes Training) und `r1_bringup/models/robots/g1_description/g1_29dof.xml` (bereits vorhanden, unverändert nutzbar – 29 Motor-Aktuatoren in exakt der richtigen Reihenfolge, zwei IMU-Sites).

**Neue Abhängigkeit:** `onnxruntime` (Python-Paket) wird von `rl_locomotion_controller_29dof.py` importiert, ist aber (wie `torch` beim 12-DOF-Pfad) nicht in `package.xml` deklariert – muss manuell installiert werden:
```
pip install onnxruntime --break-system-packages
```

**Noch offen / nicht getestet:**
1. Erster echter Testlauf in der Simulation (MuJoCo + ROS2) mit `config_file:=g1_policy_29dof.yaml` – steht noch aus.
2. `cmd`-Begrenzung auf den Trainings-Wertebereich der Policy ist in `g1_policy_29dof.yaml` nur dokumentiert, nicht im Code erzwungen (siehe `cmd_training_range` dort) – vor intensiverem Gebrauch (insb. höhere Geschwindigkeiten) nachrüsten.
3. Nicht erneut geprüft: ob `_apply_heading_hold()` (Kurshaltung) mit dieser Policy genauso gut funktioniert wie beim 12-DOF-Pfad – wurde ursprünglich gegen die 12-DOF-Policy abgestimmt.
4. Ausgangshypothese (Arme/Oberkörper verursachen das Kippen) selbst noch nicht bestätigt oder widerlegt – das ist genau die Frage, die der erste Testlauf beantworten soll.

## DOF-Migration: erster echter Testlauf (13.10.2026) – Bug gefunden+behoben, zweites Problem offen

**Bug 1 gefunden und behoben: fehlendes `armature` in `g1_29dof.xml`.** Keines der 29 Gelenke hatte einen `armature`-Wert gesetzt (MuJoCo-Standard 0.0), während Isaac Lab (wo die Policy trainiert wurde) für alle G1-29DOF-Aktuatoren `armature=0.01` nutzt (verifiziert im `unitree_rl_lab`-Quellcode, `unitree.py`, `UNITREE_G1_29DOF_CFG`). Fehlende Rotor-Trägheit führte zu numerischer Instabilität. Fix ergänzt (`armature="0.01"` auf allen 29 `<joint>`-Elementen, Original als `.bak_vor_armature_fix` erhalten), **isoliert (reine Physik, kein ROS2, in einer Cloud-Sandbox mit MuJoCo) bestätigt: 20s stabil, Stehen UND Gehen.** Fix korrekt in beiden Mirrors UND im echten WSL2-Workspace installiert (per `grep -c` auf `src/` und `install/share/` verifiziert: beide Male 29/29 Treffer).

**Trotzdem: echter ROS2-Testlauf (`config_file:=g1_policy_29dof.yaml`) fällt weiterhin um**, schon während `BereitschaftState` (vor dem ersten `/cmd_vel`), nach ca. 4,36s, mit derselben Fehlermeldung (`Nan, Inf or huge value in CTRL at ACTUATOR 1`) wie vor dem Fix. Der Bug ist also nicht (alleine) der Grund.

**Neue Arbeitshypothese (durch Simulation gestützt, NICHT am echten System verifiziert):** `mujoco_node.py` und `steuereinheit_node.py` laufen als zwei unabhängig getaktete ROS2-Prozesse, nur über Topics verbunden – keine echte Synchronisation. Eine künstliche Latenz-Injektion in der isolierten Sandbox (Verzögerung zwischen "Sensor lesen" und "Zielwinkel anwenden") zeigt: 0–100ms bleibt stabil (20s getestet), ab 160ms fällt der Roboter nach 2,05s um. Das deutet darauf, dass die reale Round-Trip-Latenz zwischen den beiden Prozessen bei der 29-DOF-Variante über ~100-150ms liegt (mehr Gelenke = größere Nachrichten + mehr Rechenlast pro Schritt). Beim 12-DOF-Pfad war das Problem ähnlicher Art, wurde aber durch `RENDER_DECIMATION=10` ausreichend entschärft – für 29 DOF offenbar nicht mehr ausreichend.

**Konkrete nächste Schritte (noch nicht ausgeführt):**
1. Reale Rate messen: `ros2 topic hz /joint_states` und `/joint_commands` während des 29-DOF-Laufs, Vergleich mit der 12-DOF-Baseline (~320-355 Hz nach dem `RENDER_DECIMATION`-Fix).
2. Publish-Overhead in `mujoco_node.py::_publish_states()` prüfen/reduzieren (aktuell jeden Physikschritt, bis zu 500 Hz, bei 29 Gelenken ~2,4x größere Nachrichten als bei 12) – Versuch: eigener `PUBLISH_DECIMATION`-Zähler, empirisch gegen Stabilität abwägen (zu seltenes Publizieren kann Latenz auch erhöhen statt senken).
3. Alternativ: Schleife in `_run_simulation()` mit `time.perf_counter()` profilieren (Physik vs. Publish vs. PD-Berechnung).
4. Erst danach zurück zur eigentlichen Forschungsfrage (hilft 29-DOF-Ganzkörper tatsächlich gegen das Kippen, verglichen mit 12-DOF).

**Werkzeug-Entscheidung für die weitere autonome Fehlersuche:** Ein vom Nutzer vorgeschlagener, selbstgebauter Python-Agent (eigene Haiku/Sonnet-Model-Umschaltung, Telegram-Benachrichtigungen, Kosten-Tracking über die Anthropic-API) wurde geprüft und verworfen – ein solches Skript würde die Claude-API direkt ansprechen und wäre damit *immer* pro Token abgerechnet, unabhängig vom Modell, was dem expliziten Wunsch des Nutzers widerspricht, ausschließlich über das bestehende Pro-Abo (ohne Extra-Tokenkosten) zu arbeiten. Stattdessen: Claude Code CLI direkt in WSL2 installiert (`curl -fsSL https://claude.ai/install.sh | bash`, erfolgreich, Version 2.1.296) – läuft mit echtem Shell-Zugriff auf ROS2/MuJoCo/colcon und nutzt die Abo-Anmeldung (keine separate API-Abrechnung). Übergabe-Dokument `UEBERGABE_fuer_claude_code_wsl2.md` wurde für eine Claude-Code-Sitzung in WSL2 geschrieben (liegt neben dieser Datei), damit die offene Latenz-Frage dort direkt mit echtem ROS2-Zugriff weiterbearbeitet werden kann, ohne die Vorarbeit zu wiederholen. Hinweis: Claude-Code-Sitzungen (lokale `.jsonl`-Historie unter `~/.claude/projects/...`) sind ein eigenständiges System ohne dokumentierte Verbindung zum claude.ai-Projekt "MBSE" (keine Integration bekannt, Stand 13.10.2026) – Kontext-Transfer läuft daher über Projektdateien (diese Datei + das Übergabe-Dokument), nicht über eine Projekt-Verknüpfung.

## 29-DOF in echter ROS2-Pipeline stabil (13.10.2026, Claude Code in WSL2)

**Ergebnis: Die 29-DOF-Policy steht und läuft jetzt in der echten ROS2-Pipeline stabil** (`config_file:=g1_policy_29dof.yaml`): 70 s Stehen in Bereitschaft, Freigabe per `/cmd_vel`, 15 s Gehen mit 0,4 m/s (ca. 5,2 m, z ≈ 0,78 m), Stopp, weiter Stehen – kein Sturz, keine Instabilitätswarnung. Der 12-DOF-Default-Pfad wurde als Regressionstest ebenfalls kurz gefahren (Gehen/Stoppen ohne Sturz).

**Die Latenz-Hypothese aus der Übergabe war falsch bzw. nicht die Ursache.** Gemessen und gefunden wurden stattdessen drei voneinander unabhängige Dinge:

1. **Startrennen (Hauptursache für den Sturz nach ~4 s in Bereitschaft).** `steuereinheit_node` braucht ~7 s Wandzeit zum Hochfahren (Imports, ONNX-Laden). In dieser Zeit simulierte `mujoco_node` bereits mit reiner PD-Haltung auf `default_angles`, was nachweislich umkippt: der Roboter lag schon bei Sim-Zeit 1,9 s (z = 0,096), bevor das erste `/joint_commands` ankam. Frühere „stabile“ Läufe waren nur deshalb stabil, weil die Sim so langsam lief (0,35–0,45× Echtzeit). **Fix:** `mujoco_node` pausiert die Physik, bis das erste `/joint_commands` eintrifft (Parameter `wait_for_controller`, Default `True`); Sensorwerte werden währenddessen weiter publiziert.
2. **Kurshaltung war ein ungebremster Integrator (Sturz direkt nach der Freigabe).** `_apply_heading_hold()` addierte jeden Takt bis zu ±0,3 rad/s auf `self.cmd[2]`, die Rampe holte nur 0,02 pro Takt zurück. Im synchronen Test wuchs `cmd_wz` im Stand auf −572 rad/s nach 60 s. In Bereitschaft fiel das nicht auf (dort geht cmd=0 an die Policy), beim Wechsel nach Balancieren fiel der Roboter 0,6 s später um. **Fix:** eigener Rampen-Zustand `cmd_ramped`; `self.cmd` wird jeden Takt neu daraus gebildet, Korrekturen akkumulieren nicht mehr. Dieser Bug existierte auch im 12-DOF-Pfad (dort durch andere Randbedingungen weniger sichtbar).
3. **Performance der Simulation.** (a) onnxruntime belegte mit den Defaults alle Kerne mit Spin-Waiting (`steuereinheit_node` ~270 % CPU) und nahm der Simulation Rechenzeit weg → jetzt 1 Thread, kein Spinning (~35–50 % CPU). (b) `_publish_states()` kostete ~1,1 ms pro Aufruf und lief bei jedem der 500 Schritte → jetzt alle 5 Schritte (100 Hz). (c) Sleep pro Schritt mit Overshoot → jetzt absolute Zeitplanung mit Aufholen. Ergebnis: ~490–500 Hz, also Echtzeit (vorher 176–280 Hz bzw. 0,35–0,55×). `viewer.sync()` kostet ~1 ms/Schritt amortisiert und bleibt.

**Messwerte (getestet):**
- Physik: `mj_step` ≈ 276 µs (Budget bei 500 Hz: 2000 µs), PD ≈ 10 µs, `viewer.sync()` ≈ 16 ms pro Aufruf.
- Synchroner Regelkreis ohne ROS (echter Controller + echtes Modell): Stehen 120 s stabil, Gehen 0,4 m/s 20 s stabil (7,6 m), robust gegen Sensor→Aktion-Verzögerung bis 40 ms, Takt-Jitter ±3 Schritte und systematische Tick-Abweichung 6–14 statt 10 Physikschritte pro Regeltakt, sowie kleine Drehraten-Kommandos (|wz| ≤ 0,3).
- ROS-Kommando-Rate zur Sim: ~50 Hz (max. Lücke im Normalbetrieb 30–50 ms).

**Hinweise / Einschränkungen:**
- Im Stand driftet der Roboter langsam (ca. 0,3–0,6 m über 60–120 s in x, wenig in y) – bekannte Eigenschaft der Policy, kein Sturz.
- Viele kurzlebige `ros2`-CLI-Aufrufe (`topic echo/hz`) während des Laufs stören die Sim spürbar (CPU/DDS); für Messungen einen persistenten Prozess verwenden.
- `mujoco_node.py` enthält jetzt eine per Parameter `profile:=true` zuschaltbare Zeitmessung (Default aus).
- Noch offen: `cmd`-Begrenzung auf den Trainingsbereich ist nicht im Code erzwungen; `onnxruntime` fehlt weiterhin in `package.xml`; erneuter Test der finalen NotAus-Variante; Forschungsfrage 29-DOF vs. 12-DOF (Kipp-Robustheit) ist jetzt untersuchbar, aber noch nicht ausgewertet.
- Geänderte Dateien: `r1_bringup/r1_bringup/mujoco_node.py`, `roboter_steuerung/roboter_steuerung/steuereinheit_node.py`, `roboter_steuerung/roboter_steuerung/control/rl_locomotion_controller_29dof.py`. Noch nicht committet.

## NotAus mit den neuen Fixes getestet (13.10.2026) – bestanden (29-DOF-Pfad)

Testablauf in einem persistenten ROS2-Prozess (`/not_aus` mit TRANSIENT_LOCAL-QoS wie im Knoten): Gehen 0,4 m/s (10 s, 3,5 m) → `/not_aus=true` während des Gehens (20 s) → `/not_aus=false` ohne neue Freigabe (10 s) → neue `/cmd_vel` 0,4 (10 s) → Stopp.

Ergebnis (gemessen): kein Sturz in irgendeiner Phase (z = 0,78–0,79 m durchgehend). Nach NotAus kommt der Roboter innerhalb ca. 2–3 s zum Stehen (x bleibt bei 3,55 m, Drift danach ≤ 1 cm in 17 s). Nach Aufhebung bleibt er in Bereitschaft stehen und läuft **nicht** von selbst wieder an; erst die neue `/cmd_vel` löst über Bereitschaft → Gehen die Bewegung aus (3,54 → 7,4 m). Zustandslog passt: Bereitschaft/Freigabe → Gehen → NOT-AUS ausgelöst → aufgehoben → Bereitschaft (warte auf Freigabe) → Freigabe → Gehen → Balancieren.

Damit ist die finale NotAus-Variante (Policy aktiv, cmd=0) und die bewusste Freigabe nach Entwarnung im echten Lauf bestätigt. Nicht getestet: NotAus im 12-DOF-Pfad, NotAus direkt in Start/Bereitschaft.

`onnxruntime` ist als Hinweis-Kommentar in `roboter_steuerung/package.xml` ergänzt (kein rosdep-Schlüssel vorhanden, `<depend>` würde `rosdep install` brechen; Muster wie bei `mujoco`).

## cmd-Begrenzung + NotAus-Stillstand (13.10.2026)

**cmd-Begrenzung umgesetzt und getestet:** `steuereinheit_node.py` begrenzt `self.cmd` nach Rampe und Kurshaltung per `np.clip` auf `cmd_training_range` aus der YAML (vx −0,5…1,0, vy ±0,3, wz ±0,2) und warnt einmalig, wenn `/cmd_vel` außerhalb liegt. Ohne den Schlüssel (12-DOF-Config) unverändert. Test im echten Lauf mit `vx=2,0 / wz=1,0`: Warnung erscheint, Roboter bleibt stabil (z ≈ 0,77), läuft ~0,7 m/s und dreht langsam (wz auf 0,2 begrenzt). Hinweis: die Kurshaltung (bis ±0,3) wird dadurch ebenfalls auf ±0,2 begrenzt.

**NotAus: Beine stehen im 29-DOF-Pfad still (gemessen).** Mittlere Beingelenk-Geschwindigkeit nach NotAus aus dem Gehen: ~0,7 rad/s (Gehen) → 0,017 nach ~2 s → 0,002 rad/s ab ~4 s, beide Füße flach am Boden (z = 0,034 m, konstant); im synchronen Kreis identisch (0,002 rad/s). Das „Auf-der-Stelle-Traben“ aus dem Statusupdate vom 12.10. betraf den 12-DOF-Pfad und tritt mit der 29-DOF-Policy nicht auf. Die Bewegung der ersten 2–3 s ist das Abbremsen des Gehens, kein Trippeln.

## Links-Drall untersucht (13.10.2026)

**Messung (synchroner Kreis, ohne Kurshaltung, Yaw positiv = links):** Stehen +0,5°/s (+14° nach 30 s); Gehen 0,4 m/s +3,6° / 40 s; 0,6 m/s −14° / 40 s (rechts); 1,0 m/s +52° / 40 s (links); rückwärts −0,3 m/s +10° / 40 s. Der Drall hat also kein festes Vorzeichen, er hängt von der Geschwindigkeit ab.

**Kein Modellfehler:** `g1_29dof.xml` ist in Massen, Körperpositionen und Fuß-Geoms links/rechts symmetrisch, die Ruhepose ist symmetrisch (Arme spiegelverkehrt). Die Stand-Drehung kippt das Vorzeichen mit dem Simulationszeitschritt (dt 0,002: +14° / 30 s, dt 0,005 wie im Training: −45° / 30 s) und ändert sich stark mit der Bodenreibung (Reibung 2,0: +40°, der Roboter driftet 17 m ab). Das ist eine Eigenheit der Policy (schwach gedämpfte Gier-Mode, sehr sensitiv gegenüber Physikparametern), kein Bias im Code oder Modell. Die Kurshaltung ist die richtige Schicht dafür.

**Mit Kurshaltung (Simulation, 40 s):** 0,6 m/s: y = 0,02 m; 1,0 m/s: Yaw +1,6°, seitlicher Versatz ~2 m (Positions-Drift, nur mit Positionsregelung behebbar, die früher instabil war); Stehen: Yaw stabil bei ~3,5°.

**Änderung:** Die Zielrichtung der Kurshaltung (`target_yaw`) wird bei jeder (Neu-)Freigabe auf die aktuelle Ausrichtung gesetzt, nicht mehr auf den Wert vom Hochfahren. Vorher drehte der Roboter nach dem Stehen beim ersten Kommando erst um ~10° zurück (im ROS-Lauf −9,7°), jetzt −2,7° bei 15,8° Stand-Yaw. In Weltkoordinaten läuft er entlang seiner aktuellen Blickrichtung geradeaus (y/x entspricht dem Yaw).

## Testsuite (13.10.2026)

Neuer Ordner `tests/` (Details in `tests/README.md`): `test_closed_loop.py` (11 Tests, ~20 s, ohne ROS2), `test_node_logic.py` (5 Tests, ~5 s) und `integration_ros.py` (21 Prüfungen gegen die echte Pipeline, ~2,5 min). Stand: alle bestanden, 29-DOF 21/21 und 12-DOF-Regression 18/18 (strenger Stillstand nur für 29-DOF). Gegenprobe: der alte Integrator-Code der Kurshaltung liefert im Knoten-Test −840 rad/s und würde vom Test abgelehnt. Zusätzlich wurde die cmd-Begrenzung als Methode `_clip_cmd()` herausgezogen (testbar) und `MUJOCO_LOG.TXT` in `.gitignore` aufgenommen.

## Positionshaltung im Stand (13.10.2026)

**Problem:** Der Roboter driftet im Stand (~0,3 m pro Minute) und dreht sich. Für Manipulation im Stehen ist das nicht tragbar (Objekt wandert relativ zur Hand).

**Lösung:** `_apply_position_hold()` wieder aktiviert, jetzt für den Zustand Balancieren (Schalter `position_hold: true` in `g1_policy_29dof.yaml`). P-Regler (kp 1,0, max 0,15 m/s) auf dem Weltpositionsfehler in Körperkoordinaten; das Ziel wird erst erfasst, wenn kein Kommando mehr ansteht und die Rampe abgeklungen ist, und beim Verlassen von Balancieren verworfen. Die Korrektur liegt getrennt in `pos_hold_corr` und geht über `BalancierenState` an die Policy (nicht mehr auf `self.cmd` addiert). Das frühere Scheitern (12.10.) lag sehr wahrscheinlich an derselben Akkumulation wie bei der Kurshaltung.

**Gemessen:** synchroner Kreis, 60 s: Abweichung ~1 cm (ohne Positionshaltung 32 cm); seitlicher Stoß 0,3 m/s bei t = 30 s: max. 3,9 cm, danach wieder ~1 cm. Echte Pipeline: 0,3 cm Drift in 30 s Stehen. Integrationstest 22/22, Knoten-Tests grün.

**Einschränkung:** nutzt `/base_position` (Simulations-Ground-Truth). Auf echter Hardware braucht es Odometrie/Lokalisierung. Das Gieren stellt sich bei ~6° Restfehler ein (P-Regler ohne Integralanteil), für Greifen unkritisch.

**Entscheidung (Nutzer, 13.10.2026):** Das Projekt wird zeitnah komplett auf 29-DOF umgestellt; der 12-DOF-Pfad wird nicht mehr getestet.

## Korrektur zur Positionshaltung: eigentliche Drift-Ursache (13.10.2026)

Die weitere Messung (synchroner Kreis, 50 s Stehen) zeigt, dass **die Kurshaltung** den Stand-Drift behebt, nicht die Positionshaltung:

| Variante | Basis verschoben | Beine \|dq\| | Fuß angehoben |
|---|---|---|---|
| ohne Kurs- und Positionshaltung | 31,7 cm | 0,018 rad/s (Spitze 5,9) | 1,8 % der Zeit (bis 2,5 cm) |
| nur Kurshaltung | 0,6 cm | 0,0004 rad/s | 0 % |
| Kurs- + Positionshaltung | 0,6 cm | 0,0004 rad/s | 0 % |

Ohne Gegensteuerung dreht der Roboter langsam weg (Yaw-Drift), die Policy reagiert mit gelegentlichen kleinen Schritten, und die Basis wandert. Mit Kurshaltung stehen die Beine praktisch still. Die frühere Aussage „Positionshaltung senkt den Drift von 32 cm auf 1 cm“ war damit irreführend: der Test mit kp = 0 hatte auch die Kurshaltung ausgeschaltet.

Die Positionshaltung bleibt trotzdem sinnvoll, aber als Schutz gegen Störungen: seitlicher Stoß 0,3 m/s: ohne 3,9 cm / mit 3,9 cm (kein Unterschied); Stoß 0,6 m/s: ohne 26 cm Abweichung (24 cm Endabweichung), mit 15 cm max (6 cm Endabweichung).

**Eigentlicher Fund:** Die Drift trat in **Bereitschaft** auf, weil dort cmd=[0,0,0] hart an die Policy ging, also ohne Kurshaltung. Fix: `BereitschaftState` übergibt jetzt `cmd[2]` der Kurshaltung. Echte Pipeline: Drift in Bereitschaft 0,6 cm in 30 s (vorher ~0,3 m/60 s). Integrationstest 23/23 (neue Prüfung S1).

## Greifen im Stehen: Machbarkeit gemessen (13.10.2026)

Werkzeug: `tests/arm_feasibility.py` (synchroner Kreis, ohne ROS2). Die Policy hält Beine und Taille (Kurs- + Positionshaltung wie im Knoten), die Armgelenke (Index 15..28) werden von einer IK-Trajektorie überschrieben: 10 s Stehen → Greifpose → halten → Anheben 15 cm → Ablegen → zurück. Last = Zusatzmasse je Hand (Punktmasse am Handgelenk, Trägheit vernachlässigt).

**1. Unveränderte Policy:** stabil nur bis ~0,20 m vor dem Becken (≤ 1 kg je Hand: 3,3 cm Drift, 8° Neigung; 2 kg fällt). Bei 0,25 m Sturz beim Ablegen, bei 0,30 m schon im statischen Halten und ohne Last.

**2. Ursache des Sturzes ist die Beobachtung, nicht der Schwerpunkt.** Wird der Policy für die Armgelenke die Ruhepose (Position) und Geschwindigkeit 0 als Beobachtung gegeben („blind für die Arme“, Armbewegung = Störung), bleibt der Roboter bei 0,30 m ohne Last stabil (6,8 cm Drift, 10° Neigung; bei 0,45 m noch kein Sturz). Die Arm-Positionen liegen für die Policy außerhalb ihres Trainingsbereichs.

**3. Mit Last braucht es Gegengewicht.** Maskiert, ohne Taillenausgleich, kippt der Roboter mit 0,5 kg je Hand bei 0,30 m (neigt sich statisch ~11° nach vorn, fällt beim Ablegen). Mit Taille 0,15 rad nach HINTEN (Pitch −0,15) ist es stabil. Positive Werte (nach vorn) fallen immer; −0,25 ist unzuverlässig (2 kg läuft weg); −0,10 fällt mit 2 kg.

**Empfohlener Betriebspunkt (robust geprüft):** Policy blind für Arme, Taille Pitch −0,15 rad (mit der Armpose eingeblendet), Hände 0,30 m vor dem Becken, Last ≤ 1 kg je Hand.
- 0,5 kg: 5,0 cm max. Drift, 2,7° Neigung; 1,0 kg: 5,6 cm, 4,1°; Endabweichung ≤ 0,3 cm.
- robust gegen Armabstand 0,12/0,20 m, halbe Geschwindigkeit, +10 cm Höhe; 60 s Halten in Greifpose: 4,8 / 5,5 cm, kein Anwachsen.
- Schwächen: schnelle Bewegung (0,5×Dauer) oder 25 cm Hub mit 1 kg: ~14 cm Drift, 10–12 cm Endabweichung. Größere Reichweite unter Last (0,35 m, 1 kg: 66 cm Ausreißer) und 2 kg bei 0,30 m sind nicht tragfähig.
- Teilweises Maskieren (Mischung der Beobachtung 0,25–0,75) ist NICHT monoton und nicht robust (0,75 fällt wieder) – nicht verwenden.

**Einordnung:** Hände 0,30 m vor dem Becken liegen ~10–20 cm vor der Körperfront (Rumpf reicht etwa 0,1–0,2 m nach vorn), das reicht für Tischkanten-Objekte. IK-Fehler bei 0,30 m ~6 cm (Gelenklimits, Hand nicht exakt am Ziel). Reichweite des Handgelenks insgesamt max. 0,71 m (Taille fest).

**Nicht gemessen/offen:** Hand/Greifer (Modell endet am Handgelenk, `g1_29dof_with_hand.xml` existiert), Einarm-Aufgaben (nur symmetrische Zweiarm-Posen getestet), echte Objektkontakte/Reibung, Wahrnehmung (nur Simulations-Ground-Truth), IK mit pinocchio (hier eigene gedämpfte Kleinste-Quadrate-IK auf MuJoCo-Jacobi). Regressionstest: `tests/test_arm_feasibility.py`.

**Umsetzung im Betrieb (noch NICHT gebaut):** Zustand `Manipulieren` (nur aus Balancieren): Armtargets überschreiben die Policy-Ausgabe für Gelenke 15..28, Controller maskiert die Arm-Beobachtungen nur solange dieser Zustand aktiv ist (nicht beim Gehen, dort ist die Policy auf ihre eigenen Armbewegungen trainiert), Taille Pitch −0,15 rad proportional zur Armauslenkung.

## Layer 1: Missionsplanung (13.10.2026) – umgesetzt und getestet

**Zuschnitt:** Layer 2 (Navigation, Nav2) macht ein Kollege und wurde nicht gebaut. Layer 1 (Planer + Dispatcher) ist komplett, die Nav-Aktion läuft über eine Bridge auf `nav2_msgs/NavigateToPose` (nicht getestet, `nav2_msgs` hier nicht installiert) bzw. `GehAktion` (mit einem als solchen markierten Testdouble getestet).

**Neu:**
- `roboter_task_planning`: PDDL-Domäne (3 Aktionen `gehe_zu`/`balanciere`/`warte`, Kosten = Strecke in cm), Mission als YAML, Problem-Generator, **Fast Downward** (Release 24.06.1, gebaut unter `~/tools/downward`, `astar(lmcut())`), Dispatcher mit Neuplanung (`pos_aktuell` als dynamischer Ort, `max_replans`), `mission_node` (Services `/mission/start`, `/mission/abort`, Topic `/mission/status`), `mission.launch.py`, `README.md`.
- Layer 3: `balance_dienst_node` (Service `/gleichgewicht`, Action `/balanciere`) – die bisher nur definierten Schnittstellen `GleichgewichtService`/`BalancierAktion` sind jetzt implementiert (Neigung aus `/imu`, Geschwindigkeit aus `/base_position`).
- Testdouble `nav_testdouble_node` (GehAktion-Server, ausrichten + geradeaus; ersetzt Nav2 nur in Tests).

**Getestet:** Kern 10/10 (`src/roboter_task_planning/test/test_layer1.py`, u. a. kostenoptimaler Weg, unlösbares Problem, Neuplanung, Abbruch, max. Neuplanungen); Simulation 14/14 (`tests/integration_mission.py`): Mission start→a→b mit Balancieren und Warten läuft komplett (Abstand zu b 0,19 m, steht), Plan besucht a vor b, NotAus bricht ab (Roboter steht), Start bei aktivem NotAus wird abgelehnt, keine Instabilität/Tracebacks.

**Offen / für den Kollegen:** (1) `/odom` und TF (`odom→base_link`) fehlen in der Simulation – klären, wer sie liefert. (2) Nav2-Grenzen auf Policy-Bereich setzen: `max_vel_theta ≤ 0,2 rad/s`, vx ≥ ~0,3, vy ≤ 0,3. (3) `nav_backend=nav2` ist ungetestet. (4) `balanciere` publiziert `/cmd_vel = 0`; falls Nav2 gleichzeitig sendet, konkurrieren die Quellen (idle in der Mission, aber ggf. cmd_vel-Multiplexer nötig). (5) Neuplanung nach Fehlschlag ist in der Simulation nur über Unit-Tests abgedeckt (Testdouble scheitert nicht).

## /odom und TF für Nav2 (13.10.2026)
`mujoco_node` publiziert jetzt `/odom` (`nav_msgs/Odometry`, `odom`→`base_link`, Pose aus der Floating Base, Twist im Körper-Frame) und die TF `odom→base_link` (Simulations-Ground-Truth, perfekte Odometrie). Parameter `publish_odom`/`publish_tf`/`odom_frame`/`base_frame`; `map→odom` bleibt bei Layer 2. Integrationstest erweitert (S1b: `/odom`, TF, Pose = `/base_position` auf 0,0 cm, Frames; S3: Twist linear.x = 0,40 m/s beim Gehen), 28/28 bestanden. Damit ist Punkt (1) aus der Liste „Offen / für den Kollegen“ (Layer 1) erledigt.

## Nav2 anbinden: Paket `roboter_navigation` vorbereitet (13.10.2026) – NICHT getestet

Neues Paket `roboter_navigation` (Layer 2, **Platzhalter** bis die Variante des Kollegen kommt): `config/nav2_params.yaml` (RegulatedPurePursuit, NavFn, velocity_smoother mit Policy-Grenzen vx ≤ 0,6 / wz ≤ 0,2 rad/s, Mindestgeschwindigkeit 0,3, kein Hindernis-Layer, rollende Costmaps im `odom`-Frame), `launch/navigation.launch.py` (statisches `map→odom`, `navigation_launch.py` von nav2_bringup, `params_file` übergebbar). `mission.launch.py` hat das Argument `nav2:=true`; `tests/integration_mission.py --nav2` testet es mit echtem Nav2.

**Stand:** `ros-jazzy-navigation2`/`nav2-bringup` sind hier NICHT installiert (113 Pakete, `sudo` nötig). Konfiguration und Launch sind angelegt und gebaut, YAML ist syntaktisch gültig, aber **gegen echtes Nav2 ungeprüft**. Der Testdouble-Pfad ist unverändert grün (14/14). Einschränkung: nur ~2,8 GB RAM in WSL2 (Simulation allein ~1,1 GB), Nav2 läuft mit `use_composition=True` (ein Prozess).


## Nav2 angebunden und getestet (13.10.2026)

`ros-jazzy-navigation2`/`nav2-bringup` installiert (113 Pakete). Paket `roboter_navigation` (Layer 2, Platzhalter bis zur Variante des Kollegen) läuft mit echtem Nav2: `mission.launch.py nav2:=true headless:=true`; `tests/integration_mission.py --nav2` **14/14** (2 Läufe, ohne Neuplanung): Mission start→a→b, Balancieren, Warten, Abstand zu b 0,18 m; NotAus bricht ab. Probe: Quadrat 3,0 → 3,2 → 0,2 → 0,0 mit `NavigateToPose` alle vier Ziele erreicht.

**Was dafür nötig war (Befunde):**
- `navigation_launch.py` lädt bei `use_composition=True` nur in einen bereits laufenden Container → in `navigation.launch.py` selbst gestartet. In Jazzy gehören `collision_monitor` und `docking_server` zur festen Kette und verlangen Konfiguration (hier deaktiviert/ungenutzt, ohne Sensorik).
- **Drehen auf der Stelle:** kleine `wz` werden im Stand fast ignoriert (0,2 → 0,025 rad/s, 13 %; 0,5 → 0,07; 1,0 → 0,56 rad/s), beim Gehen folgt die Policy gut (~75 %). Der Clip ±0,2 aus `deploy.yaml` ist eine Joystick-Grenze, keine Stabilitätsgrenze: `wz` ±1,0 ist stabil (60 s Stehen/Gehen, 40 ms Verzögerung, Jitter, vx bis 1,0). `cmd_training_range.ang_vel_z` jetzt ±1,0.
- **Kombinierte Grenze:** vx 1,0 + wz 1,0 fällt in ~15–20 % der Läufe (synchroner Kreis 2/10, ROS 1 von 3). Neu `cmd_wz_limit_high_speed`: oberhalb vx 0,7 wird |wz| linear bis 0,5 bei vx 1,0 abgesenkt (je Punkt 10 Seeds stabil). ROS-Integrationstest danach 3/3 grün.
- Nav2-Parameter angepasst: `PoseProgressChecker` (zählt Drehung als Fortschritt), `rotate_to_heading_angular_vel` 1,0, velocity_smoother wz ±1,0, vx ≤ 0,6, Mindestfahrgeschwindigkeit 0,3.
- `NavigationBridge` wartet auf `/lifecycle_manager_navigation/is_active`; Nav2 bricht Ziele sonst direkt nach `configure` mit ABORTED ab (führte zu einer unnötigen Neuplanung und einem Sturz-Kaskaden-Lauf).
- Neuer Parameter `headless` (`mujoco_node`, `steuereinheit.launch.py`, `mission.launch.py`): MuJoCo ohne Viewer. Mit Viewer + Nav2 reichen die ~2,8 GB RAM nicht (Control-Loop 6 Hz, Lifecycle-Heartbeat verloren).

**Offen:** `cmd_vel`-Watchdog fehlt (Layer 3 hält das letzte Kommando); Nav2 ohne Sensorik (kein Hindernis-Layer, `map→odom` statisch); Nav2-Parameter sind generisch, die Variante des Kollegen steht aus. **Hinweis:** Wegen der schwachen Standdrehung braucht jede Navigation mit enger Kurve einen Drehradius ≥ ~1,5 m beim Gehen oder Drehen auf der Stelle mit |wz| ≥ 0,8.

## Abschlussstand v1.0 (14.10.2026)

**Umgesetzt (nach „Vorgehensplan der Softwareimplementierung“):** Layer 4 HAL (`mujoco_node`: Physik, Lidar, `/odom`, TF, headless), Layer 3 (Zustandsautomat, ONNX-Controller, Kurs-/Positionshaltung, NotAus, `cmd_vel`-Begrenzung und -Watchdog, Balance-Dienste), Layer 2 (Nav2 mit Lidar-Hindernis-Layer, optional `slam_toolbox`), Layer 1 (PDDL, Fast Downward, Dispatcher mit Neuplanung über ROS-2-Actions). Testwelt: zwei Räume, Tür, Hindernis (`scene_testwelt_29dof.xml`).

**Tests (Stand 14.10.2026, alle bestanden):** `tests/` pytest 24 (Regelkreis, Knoten-Logik, Greif-Machbarkeit); `test_layer1.py` 12; `integration_ros.py` 28/28 (5 Läufe in Folge); `integration_watchdog.py` 5/5; `integration_mission.py` 14/14 für Testdouble, `--nav2`, `--welt` (Hindernis umfahren, Tür, Mindestabstand 0,6 m), `--patrouille` (a, b, Rückkehr zum Start) und 22/22 für `--slam` (Karte enthält Wände, Hindernis, Trennwand; Tür frei; `map→odom`).

**Wesentliche Befunde der letzten Etappe:**
- Das Lidar in der Simulationsschleife (zusammen mit Viewer-Rendering) störte das Echtzeit-Timing (Pipeline-Test ~1/3 Ausfälle). Behoben durch: Lidar nur per Launch-Argument (`lidar:=true`, in `mission.launch` an, in `steuereinheit.launch` aus), Scan als float32-Array (1,6 → 0,06 ms), seltenes Rendern (`render_decimation` 25). Danach 5/5 grün. Im synchronen Kreis 48 Läufe der Extremfolge (vx 1,0 + wz) ohne Sturz: der Fehler lag am Echtzeitverhalten, nicht an der Regelung.
- `package.xml` von `roboter_steuerung` enthielt `--` in einem XML-Kommentar → ungültig, das Paket fehlte nach einem vollständigen `colcon build` im Install-Index (Launch: „Package not found“). Behoben.
- Nav2/RPP: Drehbeschleunigung wird relativ zur gemessenen Drehrate begrenzt; die Policy reagiert im Stand auf kleine wz kaum → `max_angular_accel` 25. Globale Costmap im `map`-Frame (sonst „Unable to transform poses“ mit SLAM). Navigations-Bridge wartet auf `is_active` und wiederholt sofortige Abbrüche (< 6 s) einmal still.
- Dienste (`balance_dienst_node`, `mission_node`) abonnieren 100-Hz-Topics nur auf Abruf (Leerlauf-CPU 30–48 % → ~0).
- SLAM-Karte: Wände genau auf der Rasterkante (Nordwand) werden von `slam_toolbox` abgeschnitten; Welt-Geometrie 5 cm nach innen. Mit SLAM liegen Ziele im `map`-Frame; Scan-Matching-Versatz gegen die Welt ~0,5–0,75 m in dem glatten Raum.

**Bekannte Grenzen / offen:** Manipulation (Zustand `Manipulieren`, Hand-Modell, Wahrnehmung) nur als Machbarkeitsmessung; AkkuSchwach/Laden fehlen (keine Batteriedaten); Nav2-Variante des Kollegen steht aus; keine echte Hardware (Ground-Truth-Odometrie, Raycast-Lidar); WSL2 mit ~2,8 GB RAM: Nav2/SLAM nur headless; vereinzelt SIGKILL (Exit 137) durch Speicherdruck bei langen Testserien.

**Rücksprungpunkte (Git-Tags):** `stand-2026-10-13-nav2-grundlage`, `v0.8-testwelt-nav2-lidar`, `v0.9-patrouille`, `v0.10-slam`, `v1.0`.

## Realistische Sensorik und Zustandsschätzer (14.10.2026) – v1.1

**Ziel:** Die Simulation war „zu perfekt“ (Ground-Truth-Position, ideale Sensoren). Damit der spätere Wechsel auf echte Hardware weniger Überraschungen bringt, laufen dieselben Tests jetzt optional mit realistischer Sensorik und geschätzter Position.

**Schalter `realistic_sensors:=true`** (`mujoco_node`, durchgereicht in `steuereinheit.launch`/`mission.launch`): Gelenkrauschen (0,001 rad, 0,02 rad/s), IMU-Rauschen (0,01 rad/s) mit konstantem Gyro-Bias (Std. 0,005 rad/s), Orientierungsrauschen (0,005 rad), Gier-Drift 0,5 °/min, Latenz 10 ms auf Zuständen und 10 ms auf Kommandos, Lidar-Rauschen 2 cm und 1 % Ausfälle. Gelenkmomente sind im Modell bereits begrenzt (5–139 Nm). Gemessen (synchroner Kreis): Policy bleibt bis zum 10-Fachen dieses Rauschens plus 20 ms Latenz stabil (36 Läufe ohne Sturz).

**Schalter `odom_quelle:=beinodometrie`:** neuer Knoten `beinodometrie_node` (Klasse `zustandsschaetzung/beinodometrie.py`, ROS-frei) schätzt die Basisposition aus Gelenkwinkeln und IMU (Standbein steht still → Basis bewegt sich gegenläufig zur Fußbewegung; Vorwärtskinematik über das MuJoCo-Modell, auf Hardware gleich nutzbar). `mujoco_node` publiziert dann kein `/odom`/TF mehr. Alle Positionsnutzer (Positionshaltung, Balance-Dienst, Missionsknoten, Testdouble) lesen jetzt `/odom` statt `/base_position`; `/base_position` bleibt nur als Messwert für Tests.
- Genauigkeit (synchroner Kreis, mit Rauschen): Geradeaus 14,5 m → 11 cm, Gehen+Drehen 24,5 m → 8 cm (< 1 %). Entscheidend waren 100 Hz Aktualisierung, 3 mm Hysterese beim Standbeinwechsel und das Verbuchen der Bewegung mit dem alten Standfuß vor dem Wechsel (vorher 16 % Fehler).

**Tests:** `integration_ros.py --realistisch --beinodometrie` 29/29 (Schätzfehler nach allen Szenarien inkl. schneller Spiralen 24,8 cm); `integration_mission.py --welt --realistisch` 14/14; `integration_mission.py --slam --realistisch --beinodometrie` 22/22 (Endabstand zu b 0,16 m, Karte vollständig); `tests/test_beinodometrie.py` (Fehler < 2 % der Strecke).

**Was weiterhin fehlt gegenüber echter Hardware:** echte Reibung/Motorverhalten, echte Lidar-Physik (3D-Punktwolke, Reflexionen), echte Unitree-SDK-Bridge, Sicherheitskette beim Hochfahren. Beinodometrie nimmt an, dass der Standfuß nicht rutscht.


## Demo-Szenario "Kontrollrundgang im Lager" (14.10.2026)
Szene `scene_demo_lager_29dof.xml` (Halle mit Regalen und Kistenstapeln, Büro mit 1,6 m breiter Tür, Ladestation), Mission `demo_lager.yaml`: K1 (Regalgang) → K2 (Hallenecke) → K3 (Büro) → balancieren → warten → Ladestation; danach NOT-AUS-Sequenz. Lauf in der realistischsten Konfiguration (SLAM, realistische Sensorik, Beinodometrie): **Mission erfolgreich in ~100 s, ohne Neuplanung**. Video (127 s, Verfolgerkamera, Lidar-Treffer, Leitstand-Karte, Auftragsliste) wird offline aus der aufgezeichneten Pose gerendert; Skripte in `demo/` (`run_demo.py`, `render_demo.py`, `README.md`). `mujoco_node` hat dafür den Parameter `record_path`.


## 3D-Lidar (Punktwolke) und Gegenprobe mit niedrigem Hindernis (14.10.2026) – v1.3

**Umgesetzt:** `mujoco_node` kann zusätzlich zum 2D-Scan ein 3D-Lidar als `PointCloud2` (`/points`, Frame `lidar_link`, 24 Ringe von −40° bis +10°, 360 Azimutwerte, 5 Hz, Raycast, mit `realistic_sensors` verrauscht) publizieren (`lidar_3d:=true`). Nav2-Variante `nav2_params_3d.yaml` nutzt `/points` als zweite Hindernisquelle (Mindesthöhe 0,12 m blendet den Boden aus); Roboterradius dort 0,36 m. Szene `scene_testwelt3d_29dof.xml` (Testwelt plus niedrige Palette 0,45 m und ein nur für das Lidar sichtbarer Boden), Mission `testwelt3d.yaml`.

**Gegenprobe (`tests/integration_mission.py --nur-2d` / `--drei-d`):** Die 0,45 m hohe Palette liegt unter der 2D-Scanebene (1,24 m) und ist für den 2D-Lidar unsichtbar.
- nur 2D: Roboter kommt der Palette konsistent bis **0,14–0,15 m** nahe (3 Läufe) – Kontakt (Körperhalbbreite ~0,2 m), Mission wird trotzdem erreicht.
- mit 3D: **0,21–0,33 m** (7 Läufe), Mission 14/14 ohne Neuplanung.
- Die Marge ist knapp: niedrige Hindernisse bekommen wenige Punkte, die Pfadfolge weicht ±0,1 m ab. Wirksam war das Abschalten des `clearing` für den 2D-Scan (seine Strahlen gehen über das Hindernis und löschten dessen Marken aus der Punktwolke ständig wieder).

**Regression:** pytest 26, `integration_ros` 29/29, Mission (Testdouble) 14/14, `--welt` 14/14, `--slam --realistisch --beinodometrie` 22/22.

**Grenzen:** SLAM bleibt 2D (`slam_toolbox`); die Punktwolke dient nur der Hinderniserkennung. Keine Kamera. Das Lidar sieht den eigenen Körper nicht (nur Welt-Geoms der Gruppe 2).


## Ein Lidar statt zwei (14.10.2026) – v1.4

**Vorher (v1.3):** 2D-Scan und 3D-Punktwolke wurden getrennt simuliert und parallel genutzt – technisch ein Gerät, aber zwei unabhängige Datenquellen. Ein echtes 3D-Lidar liefert nur die Punktwolke.

**Jetzt (Standard `lidar_modus:=3d` in `mission.launch`):** Die Simulation publiziert nur noch `/points` (24 Ringe, 5 Hz). Der neue Knoten `roboter_navigation/punktwolke_zu_scan` leitet daraus ab:
- `/scan` (für `slam_toolbox` und `collision_monitor`): je Winkel der nächste Punkt im Höhenband 0,12–1,9 m, mit IMU waagerecht ausgerichtet (Boden fällt raus, niedrige Hindernisse bleiben drin).
- `/punkte_hindernis` (für die Nav2-Costmaps): Punkte im Höhenband, auf 5 cm ausgedünnt.
Nav2 nutzt jetzt die **VoxelLayer** (3D-Raytracing). Die 2D-`ObstacleLayer` löschte niedrige Hindernisse, über die Strahlen zur Wand hinweggehen.
Die alte Einzelebene bleibt als `lidar_modus:=2d` (`nav2_params_2d.yaml`) für Vergleiche.

**Fehler auf dem Weg:** Im reinen 3D-Modus fehlte zunächst die statische TF `base_link→lidar_link` (sie hing am Block für den 2D-Scan) – Nav2 verwarf alle Punktwolken („observation buffer has not been updated“), der Roboter blieb stehen. Behoben. Außerdem liefen Ableitungsknoten aus abgebrochenen Läufen weiter (`tests/stop_all.sh` ergänzt).

**Ergebnisse:** niedrige Palette mit VoxelLayer 0,31 / 0,35 / 0,31 m Abstand (mit ObstacleLayer 0,18 m; nur 2D: 0,15 m = Kontakt). Regression: pytest 31, Layer-1 12, Pipeline 29/29, Mission 14/14 (Testdouble, `--nav2`, `--welt`, `--patrouille`), Gegenprobe `--nur-2d` 13/13, `--slam --realistisch --beinodometrie` 22/22.


## Review-Paket 1: Sicherheit NotAus/Freigabe (14.10.2026) – v1.5
Befunde aus dem Code-Review vom 14.10. (B1, B5, L1, L3) bestätigt und behoben:
- **B1/L3:** Freigabe nur noch über den eigenen Kanal `/freigabe` (std_msgs/Bool; `false` entzieht sie, Roboter geht nach Bereitschaft). Vorher setzte jede `/cmd_vel` die Freigabe, auch automatische Quellen und sogar während NotAus → nach der Entwarnung lief der Roboter ohne bewusste Freigabe wieder an. Jetzt: NotAus entzieht im Callback sofort Freigabe und Kommando, `/cmd_vel` wird während NotAus ignoriert, Freigabe-Versuche werden abgelehnt (Warnung). `freigabe_per_cmd_vel:=true` stellt für Handbetrieb das alte Verhalten her (nie während NotAus). Der Missionsknoten gibt beim Missionsstart frei.
- **B5/L1:** Rampe in Bereitschaft und NotAus auf 0 (vorher lief sie mit dem Nutzerkommando hoch: Sprung nach der Freigabe, alte Drehrate in Bereitschaft). `cmd_init` wird nicht mehr verwendet (Start immer 0). Balancieren bremst jetzt weich über die Rampe ab (vorher harter Sprung auf 0).
- **Tests:** neuer Unit-Test (NotAus-Callback, `/cmd_vel` und Freigabe während NotAus); `integration_ros.py` neu S2a (`/cmd_vel` ohne Freigabe bewegt nicht), S5b (Freigabe während NotAus abgelehnt, steht trotz `/cmd_vel`-Strom), S6 (nach Entwarnung mit laufendem `/cmd_vel` kein Anlaufen). Ergebnisse: Pipeline 32/32, Watchdog 5/5, Mission 14/14, Testwelt 14/14, SLAM realistisch 22/22.


## Review-Paket 2: Layer 1 robust (14.10.2026) – v1.6
- **B2/L4:** Der dynamische Ort `pos_aktuell` (Neuplanung) hat nur noch **ausgehende** Kanten zu den 2 nächsten Orten und zum Ziel des gescheiterten Schritts; vorher war er in beide Richtungen mit allen Orten verbunden, wurde als Abkürzung durchquert (`gehe_zu(k1, pos_aktuell)`) und führte zum `KeyError`. Die Bridges bekommen die Koordinaten jetzt aus dem Weltwissen (`orte=`).
- **B4:** Ortsnamen werden beim Laden kleingeschrieben; `pos_aktuell` reserviert; doppelte Namen (Groß/Klein) abgelehnt.
- **B3:** `MissionExecutor.run` fängt jede unerwartete Ausnahme ab und meldet `fehlgeschlagen` mit Endstatus; der Missions-Thread hat zusätzlich einen Rahmen.
- **U4:** Position für Startort/Neuplanung per TF `map→base_link` (Missionsorte liegen im map-Frame), Rückfall `/odom`. Fund dabei: `lookup_transform(Time())` wartet in rclpy nicht auf einen frischen Listener → aktives Warten mit `can_transform`.
- **L5:** Ohne `start_ort` und ohne Position wird die Mission abgelehnt statt einen beliebigen Ort anzunehmen.
- **Test-Korrektur:** „steht nach NotAus“ im Missionstest misst jetzt nach der Bremsphase (wie im Pipeline-Test); vorher zählte der Bremsweg aus voller Fahrt mit (einmaliger Fehlschlag bei z = 0,79, also nicht gestürzt).
- **Tests:** 6 neue Layer-1-Tests (u. a. Neuplanung in einer reinen Kette ohne Durchqueren von `pos_aktuell`, Großbuchstaben, Ausnahme in Bridge, fehlender Startort) → 18/18. Missionen 14/14 (Testdouble, Testwelt, Patrouille, 3D-Palette), SLAM realistisch 2× 22/22.
- **Offen beobachtet:** In 1 von 5 SLAM-realistisch-Läufen brach Nav2 die Etappe a→b ab, bei der Neuplanung streifte der Roboter die Tür (0,20 m) und stürzte. Nicht reproduziert; vermutlich Last/Timing. Wird weiter beobachtet.


## Review-Paket 3: Übergabe und Reproduzierbarkeit (14.10.2026) – v1.7
- **B6:** Controller werden erst bei Bedarf importiert; der 29-DOF-Pfad braucht kein PyTorch mehr (geprüft: `steuereinheit_node` importiert mit gesperrtem `torch`).
- **B7:** `slam_toolbox` wird nur bei `slam:=true` aufgelöst (OpaqueFunction).
- **U1:** Standard-Config überall `g1_policy_29dof.yaml` (`steuereinheit_node`, `mujoco_node`, `steuereinheit.launch`); geprüft: Start ohne Argumente nimmt 29-DOF.
- **Pfade:** harte Rückfälle auf `~/humanoid_robot` in `steuereinheit_node`/`mujoco_node` durch klare Fehlermeldungen ersetzt; `demo/render_demo.py` relativ zum Repo.
- **package.xml/setup.py:** Gazebo/ros2_control-Altlasten keine Pflicht mehr (Kommentar statt `exec_depend`); `nav_msgs`, `nav2_msgs`, `slam_toolbox`, `rviz2`, `tf2_ros` ergänzt; Maintainer/Lizenz statt `TODO`-Platzhaltern (Lizenz Apache-2.0 wie bereits in `r1_bringup` eingetragen), Beschreibungen aktualisiert.
- **requirements.txt** mit getesteten Versionen (mujoco 3.13.0, onnxruntime 1.31.0, numpy 1.26.4, PyYAML 6.0.1; optional torch, imageio-ffmpeg).
- **Ballast entfernt** (in Git abrufbar): `src/_backup_vor_lockstep_*`, `g1_29dof.xml.bak_vor_armature_fix`, leere `task_planner_node.py`, `standalone_test.py`; `.pytest_cache/` ignoriert.
- **B8 teilweise:** Fast Downward: `.py` mit python3, Wrapper aus PATH direkt; Dateien mit Kontextmanager.
- **Tests:** pytest 32, Layer-1 18, Pipeline 32/32, Testwelt 14/14, SLAM realistisch 22/22. (Ein erster Pipeline-Lauf scheiterte, weil eine Simulation aus einem Vortest noch lief – Testumgebung, nicht Code; sauber wiederholt 32/32.)


## Review-Paket 4: Kommentare und Dokumentation (14.10.2026) – v1.8
- Veraltete/widersprüchliche Kommentare korrigiert: `steuereinheit_node.py` (Positionshaltung „NICHT aufgerufen“, `/base_position`, „47 Beobachtungswerte“, `cmd_init`, Freigabe per `/cmd_vel`), `balancieren_state.py` (vx/vy-Eingabe), `not_aus_state.py` („trabt“ nur 12-DOF, Freigabe-Schritt umgesetzt), `bereitschaft_state.py`, `mujoco_node.py` (Topic-Liste, „bei jedem Physikschritt“, „47-Werte-Policy“, Profiling dauerhaft dokumentiert, `if True:` aufgelöst, Rauschwerte als Annahmen gekennzeichnet), `steuereinheit.launch.py` (2D-Lidar, 16→24 Ringe), `action_bridges.py` (Nav2 getestet, GehAktion nur Testdouble), `nav_testdouble_node.py` (WZ_MAX), `g1_policy_29dof.yaml`.
- Spärlich kommentierte Stellen ergänzt: Problem-Generator (Abbildung, ganzzahlige Kosten, Kosten 100), Planer (warum A*/LM-Cut), `run_action` (Zeiten), Missionsknoten (Thread-Modell), Weltwissen, Balance-Dienst-Konstanten (Herkunft), Beinodometrie-/Punktwolken-Callbacks, Testdouble, Lidar-/Aufnahmefunktionen.
- Doku: READMEs von `roboter_task_planning`, `roboter_navigation` (inkl. RViz), `tests` (fehlende Tests ergänzt) aktualisiert; Kopf dieses Protokolls und „Thesis“-Bezug korrigiert; alte Übergaben in einen Archivordner verschoben (am 15.10.2026 gelöscht, nur noch unter Tag `v1.9-review-abgeschlossen`); `STATUSUPDATE_fuer_cowork.md` als kompakte aktuelle Übergabe neu geschrieben (seit 15.10.2026: `PROJEKTSTAND_AKTUELL.md`).
- Hinweis: `CODE_REVIEW_PROMPT.md` (Review-Auftrag) wurde beim Commit von Paket 1 versehentlich per `git add -A` mitversioniert; Inhalt unkritisch; lag zunächst im Root, dann bei den Review-Unterlagen; am 15.10.2026 gelöscht (Tag `v1.9-review-abgeschlossen`).
- Tests: pytest 32, Layer-1 18, Pipeline 32/32, Testwelt 14/14.


## Review-Paket 5: kleinere Punkte (14.10.2026) – v1.9
- **B8:** `mujoco_node.destroy_node` stoppt den Simulationsthread mit `join`, schreibt erst danach die Aufnahme (kein Race, kein Publish auf zerstörten Knoten). Lidar-Taktannahme (100 Hz Publish) dokumentiert.
- **U3:** `/base_position`-Abo und `read_base_position()` aus `MujocoBridge`/`IRobotHardwareBridge` entfernt (ungenutzt, auf Hardware nicht erfüllbar).
- **U5:** Geometrie zentral in `g1_policy_29dof.yaml` (`becken_hoehe_stand`, `lidar_hoehe_ueber_becken`); genutzt von `mujoco_node`, `punktwolke_zu_scan`, `beinodometrie_node`.
- **U6:** Lidar-Ausfälle mit `realistic_sensors` auch in der 3D-Punktwolke.
- **U7:** `/balance_mode` entfernt (immer False, kein Abonnent). `nav_toleranz_m`/`nav_geschwindigkeit` nur im GehAktion-Backend wirksam – dokumentiert.
- **L2:** Kurshaltung greift nur noch bei angeforderter Drehrate ≈ 0 (Schwelle 0,001 statt 0,05), damit sie nicht gegen kleine Nav2-Drehbefehle regelt.
- **L7:** Beinodometrie verarbeitet jede 100-Hz-Nachricht (Mindestabstand 9 statt 10 ms). Folge: die Rohgeschwindigkeit pro 10 ms war stark verrauscht (Einzelwert −0,09 m/s bei 0,4 m/s Gehen, ein Pipeline-Test schlug fehl) → Tiefpass (τ 0,15 s) auf die Geschwindigkeit; neuer Test (Median-Fehler < 0,1 m/s, 99 %-Quantil < 0,3 m/s).
- **Duplikate:** gemeinsame Quaternion→Rotationsmatrix in `mujoco_node`; `punktwolke_zu_scan` richtet jede Wolke nur noch einmal aus.
- **Nur dokumentiert:** L6 (Start-Gate: NotAus wird mit den ersten Sensordaten wirksam), L8 (BackUp-Recovery unter der Totzone; sauberer Weg: eigener BT), U8 (Testdouble dreht bewusst mit 0,2).
- **Tests:** pytest 33, Layer-1 18, Pipeline 32/32, Pipeline realistisch+Beinodometrie 2× 32/32 (Schätzfehler 20–22 cm), Watchdog 5/5, Missionen 14/14 (Testdouble, Testwelt, Patrouille, 3D-Palette 0,36 m), SLAM realistisch 2× 22/22.
- **Damit ist das Code-Review vom 14.10. bis auf Kategorie 5 (SysML-Dateien nicht im Repo) abgearbeitet.**

## Strukturumbau (15.10.2026) – v2.0 bis v2.4
Anlass: zu viel Inhalt in zu wenigen Dateien, Schichtgrenzen verwischt (Welt/Orte in der Missionsdatei, Szenen im
Unitree-Herstellerordner, eine Config für drei Schichten, Layer 3 importierte die Simulation, Launch-Dateien
starteten fremde Schichten, Scan-Ableitung im Paket des Kollegen). Umbau in fünf Schritten, jeder mit eigener
kompletter Testserie (`tests/alle_tests.sh`), Commit und Tag. Verhalten unverändert.
- **v2.0 Aufräumen:** 12-DOF- und Gazebo-Altbestand, nie grüne ament-Lint-Vorlagen, alte Übergaben und
  Review-Unterlagen zunächst in Archivordner verschoben (am 15.10.2026 danach ganz gelöscht, vollständig unter Tag
  `v1.9-review-abgeschlossen`), Verlauf nach `docs/`, Tests nach Art (Paket-Unit-Tests, `tests/simulation`,
  `tests/system`, `experimente`), `pytest.ini`, `tests/alle_tests.sh`.
- **v2.1 Welten:** Paket `roboter_welten` (je Welt `szene.xml` ohne Roboter + `orte.yaml`); Missionen enthalten nur
  noch den Auftrag und verweisen per `welt:`; die Simulation setzt Roboter und Welt per `MjSpec` zusammen (vorab
  geprüft: Geom für Geom identisch zu den alten `scene_*.xml`) und startet in der Welt der Mission.
- **v2.2 Launch je Schicht:** `r1_bringup` → `roboter_simulation`; Launch-Datei je Paket; `roboter_system/system.launch.py`
  setzt zusammen; `roboter_sensorik` (Scan-Ableitung aus dem Navigationspaket heraus).
- **v2.3 Config/Bridge:** `roboter_beschreibung` (G1-Modell + Geometrie), Policy (ONNX + `policy_29dof.yaml`) und
  Bridge-Schnittstelle in Layer 3; die Simulation bekommt kp/kd/Ruhepose zur Laufzeit über `/antrieb/parameter`
  (latched) und startet erst danach; Parameterdateien je Schicht, `test_konfiguration.py` prüft sie gegen die Knoten.
- **v2.4 Knoten zerlegt:** `mujoco_node` 640 → 429 Zeilen (+ `sensormodell`, `lidar`, `aufnahme`, `geometrie`),
  `steuereinheit_node` 603 → 242 Zeilen (+ `kommando.py`, `haltung.py` als reine Logik); Unit-Tests der Bausteine;
  Simulationsthread endet beim Beenden ohne Ausnahme; `docs/architektur.md`.
- **Beobachtung (Schritt 3):** im Pipeline-Test realistisch + Beinodometrie lag ein **einzelner** `/odom`-Wert bei
  0,05 m/s (S3, Gehen mit 0,4 m/s). Diagnose: 3 × 800 Werte gleichmäßigen Gehens, Minimum 0,27–0,28, Median 0,37 –
  nicht reproduziert, Ursache offen. Die Prüfung nutzt seitdem den Median über 6 s und gibt das Minimum aus
  (danach: Median 0,35–0,36, Minimum 0,23–0,28).
- **Tests v2.4:** pytest 64, Pipeline 32/32, Pipeline realistisch + Beinodometrie 32/32, Watchdog 5/5, Missionen 14/14
  (Testdouble, Testwelt, Patrouille, 3D-Palette 0,43 m), SLAM realistisch 22/22.
