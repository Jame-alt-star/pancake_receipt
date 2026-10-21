# Fremdbestandteile und Lizenzen

Stand 21.10.2026, Lizenzen per GitHub-API bei den Original-Repositories abgefragt. **Keine Rechtsberatung** - vor einer
Weitergabe ausserhalb des Teams (Kunde, Veroeffentlichung) mit der Rechtsabteilung klaeren.

## Im Repository enthalten (wird mit weitergegeben)

| Bestandteil | Ort im Repo | Herkunft | Lizenz | Pflichten |
|---|---|---|---|---|
| G1-Modelle (MJCF, URDF, Meshes, Bilder) | `src/roboter_beschreibung/modelle/g1_description/` | `unitreerobotics/unitree_rl_gym`, `resources/robots/g1_description` (Dateien byte-identisch; einzige Aenderung: `armature="0.01"` an allen Gelenken in `g1_29dof.xml`) | BSD-3-Clause | Lizenztext und Copyright mitgeben: `LICENSE_unitree_rl_gym` liegt daneben |
| Lauf-Policy `policy_29dof.onnx` | `src/roboter_steuerung/modelle/` | `unitreerobotics/unitree_rl_lab` (`deploy/robots/g1_29dof/config/policy/velocity/v0/`) | Apache-2.0 | Lizenztext mitgeben: `LICENSE_unitree_rl_lab` liegt daneben; Aenderungen kennzeichnen (keine) |

## Benutzt, aber nicht im Repository

| Bestandteil | Lizenz | Hinweis |
|---|---|---|
| MuJoCo | Apache-2.0 | pip-Paket |
| ONNX Runtime | MIT | pip-Paket |
| ROS 2 Jazzy, Nav2 | ueberwiegend Apache-2.0/BSD (Nav2: gemischt, je Paket) | apt-Pakete |
| slam_toolbox | **LGPL-2.1** | als eigener Prozess gestartet, nicht eingebunden; bei Auslieferung LGPL-Pflichten beachten |
| Fast Downward 24.06.1 | **GPL-3.0** | als eigener Prozess aufgerufen (`subprocess`), nicht eingebunden und nicht im Repo; bei Auslieferung zusammen mit dem Planer GPL-Pflichten beachten |

## Eigener Code

Die Pakete erklaeren Apache-2.0 (`package.xml`). Eine `LICENSE`-Datei fuer das Projekt selbst fehlt bewusst: ob und
unter welcher Lizenz der eigene Code weitergegeben wird, entscheidet der Arbeitgeber.
