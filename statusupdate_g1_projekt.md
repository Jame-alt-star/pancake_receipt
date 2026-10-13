# Statusupdate – G1 Humanoid-Projekt

**Stand: 12.10.2026**

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

Für die Missionsplanung (Layer 1) wurde ein eigener Dispatcher mit Fast Downward (PDDL-Planer) festgelegt statt des Fremdframeworks PlanSys2 – passt direkt auf die vorhandene Aktionsstruktur und bleibt vollständig nachvollziehbar für die Thesis.

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
