#!/bin/bash
# Komplette Testserie in fester Reihenfolge (Reproduzierbarkeit: dieselben Tests, dieselben Optionen).
# Aufruf aus dem Projekt-Root:  bash tests/alle_tests.sh [--schnell] [--mit-slam] [--video DIR]
#   --schnell   nur Build + pytest (ohne ros2-launch-Systemtests, ~1 min)
#   --mit-slam  zusaetzlich Mission mit SLAM sowie "Karte aufnehmen und damit arbeiten" (wie in echt), jeweils mit
#               realistischen Sensoren und Beinodometrie (~16 min)
#   --video DIR jeder Systemtest zeichnet Pose, Schritte und Ergebnisse in DIR/<nr>_<name> auf; danach
#               python3 demo/video_tests.py DIR <video.mp4>
# Gesamtdauer ohne Optionen ~25 min. WSL2 hat wenig RAM: die Systemtests laufen nacheinander, vor jedem
# werden Reste beendet. Ergebnis: Tabelle am Ende, Exit-Code 0 nur wenn alles bestanden ist.
cd "$(dirname "$0")/.." || exit 1
source /opt/ros/jazzy/setup.bash
SCHNELL=0; SLAM=0; VIDEO=""
while [ $# -gt 0 ]; do
    case "$1" in --schnell) SCHNELL=1 ;; --mit-slam) SLAM=1 ;; --video) VIDEO="$(realpath -m "$2")"; shift ;; esac
    shift
done
aufn() {   # aufn <ordnername> -> Argumente fuer die Aufzeichnung (nur mit --video)
    [ -n "$VIDEO" ] && echo "--aufnahme $VIDEO/$1"
}

declare -a NAMEN ERGEB
lauf() {   # lauf <Name> <Befehl...>
    local name="$1"; shift
    echo; echo "=================== $name ==================="
    bash tests/system/stop_all.sh
    "$@" 2>&1 | tail -n 25
    local rc=${PIPESTATUS[0]}
    NAMEN+=("$name"); ERGEB+=("$([ "$rc" = 0 ] && echo OK || echo "FEHLER ($rc)")")
}

lauf "colcon build" colcon build
source install/setup.bash
lauf "pytest (Unit + Regelkreis)" python3 -m pytest -q
if [ "$SCHNELL" = 0 ]; then
    lauf "Pipeline" python3 tests/system/integration_ros.py $(aufn 01_pipeline)
    lauf "Pipeline realistisch + Beinodometrie" python3 tests/system/integration_ros.py --realistisch --beinodometrie \
        $(aufn 02_pipeline_realistisch)
    lauf "Watchdog" python3 tests/system/integration_watchdog.py $(aufn 03_watchdog)
    lauf "Mission Testdouble" python3 tests/system/integration_mission.py $(aufn 04_testdouble)
    lauf "Mission Testwelt (Nav2)" python3 tests/system/integration_mission.py --welt $(aufn 05_testwelt)
    lauf "Mission Patrouille" python3 tests/system/integration_mission.py --patrouille $(aufn 06_patrouille)
    lauf "Mission 3D-Palette" python3 tests/system/integration_mission.py --drei-d $(aufn 07_palette)
    lauf "Mission feste Karte (AMCL)" python3 tests/system/integration_mission.py --karte $(aufn 08_feste_karte)
    lauf "Mission LIBERO-Kueche" python3 tests/system/integration_mission.py --kueche $(aufn 08b_libero_kueche)
    if [ "$SLAM" = 1 ]; then
        lauf "Mission SLAM realistisch" python3 tests/system/integration_mission.py --slam --realistisch --beinodometrie \
            $(aufn 09_slam)
        lauf "Karte aufnehmen + damit arbeiten" python3 tests/system/integration_mission.py --aufnehmen --realistisch \
            --beinodometrie $(aufn 10_karte_aufnehmen)
    fi
    bash tests/system/stop_all.sh
fi

echo; echo "=================== Ergebnis ==================="
fehler=0
for i in "${!NAMEN[@]}"; do
    printf '%-40s %s\n' "${NAMEN[$i]}" "${ERGEB[$i]}"
    [ "${ERGEB[$i]}" = OK ] || fehler=1
done
if [ -n "$VIDEO" ]; then           # Ergebnistabelle fuer die Einleitung des Testvideos
    for i in "${!NAMEN[@]}"; do printf '%-40s %s\n' "${NAMEN[$i]}" "${ERGEB[$i]}"; done > "$VIDEO/ergebnis.txt"
fi
exit $fehler
