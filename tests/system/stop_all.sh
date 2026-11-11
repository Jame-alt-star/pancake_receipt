#!/bin/bash
# Beendet Pipeline, Nav2/SLAM, Hilfsknoten und uebrig gebliebene Testskripte (Reste nach abgebrochenen Tests;
# ein weiterlaufendes Testskript sendete sonst Freigaben und Befehle in die naechste Pipeline).
# Aufruf als eigener Befehl: bash tests/system/stop_all.sh  (die Muster duerfen nicht in der Kommandozeile
# des aufrufenden Befehls stehen, sonst trifft pgrep -f den Aufrufer).
pats="tests/system/integration_|demo/run_demo|ros2 launch|mujoco_node|steuereinheit_node|balance_dienst_node|mission_node|component_container|static_transform_publisher|nav_testdouble|async_slam_toolbox|punktwolke_zu_scan|beinodometrie_node|greif_dienst_node"
for p in $(pgrep -f "$pats"); do [ "$p" != "$$" ] && [ "$p" != "$PPID" ] && kill -INT "$p" 2>/dev/null; done
sleep 3
for p in $(pgrep -f "$pats"); do [ "$p" != "$$" ] && [ "$p" != "$PPID" ] && kill -9 "$p" 2>/dev/null; done
# Verwaiste Fast-DDS-Speichersegmente beendeter Prozesse entfernen (vor allem nach kill -9): sie bremsten die
# Kommunikation nachfolgender Laeufe aus. Laufende Prozesse sind davon nicht betroffen.
command -v fastdds >/dev/null 2>&1 || source /opt/ros/jazzy/setup.bash 2>/dev/null
command -v fastdds >/dev/null 2>&1 && fastdds shm clean >/dev/null 2>&1
exit 0
