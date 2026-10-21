#!/bin/bash
# Richtet einen frischen Rechner (Ubuntu 24.04 + ROS 2 Jazzy) fuer dieses Projekt ein, oder prueft nur, was fehlt.
#
#   bash scripts/einrichten.sh --pruefen     # nur pruefen, nichts installieren (kein sudo noetig)
#   bash scripts/einrichten.sh               # fehlende Teile installieren (apt mit sudo, pip --user, Fast Downward)
#
# ROS 2 Jazzy selbst (ros-jazzy-desktop) setzt das Skript voraus: https://docs.ros.org/en/jazzy/Installation.html
# Ergebnis: am Ende eine Liste OK/FEHLT; danach `colcon build` und `bash tests/alle_tests.sh --schnell`.
cd "$(dirname "$0")/.." || exit 1
PRUEFEN=0; [ "$1" = "--pruefen" ] && PRUEFEN=1

APT_PAKETE="ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-robot-state-publisher
            ros-jazzy-tf2-ros ros-jazzy-rviz2 python3-colcon-common-extensions python3-numpy python3-yaml
            python3-pytest python3-opencv"
FD_VERSION="release-24.06.1"
FD_ORDNER="$HOME/tools/downward"

fehlt=0
meldung() { printf '%-45s %s\n' "$1" "$2"; [ "$2" = OK ] || fehlt=1; }

# --- ROS 2 Jazzy
if [ -f /opt/ros/jazzy/setup.bash ]; then meldung "ROS 2 Jazzy" OK; else meldung "ROS 2 Jazzy" "FEHLT (siehe Kopf dieses Skripts)"; fi

# --- apt-Pakete
fehlende_apt=""
for p in $APT_PAKETE; do
    dpkg -s "$p" >/dev/null 2>&1 || fehlende_apt="$fehlende_apt $p"
done
if [ -z "$fehlende_apt" ]; then
    meldung "apt-Pakete (Nav2, SLAM, ...)" OK
elif [ "$PRUEFEN" = 1 ]; then
    meldung "apt-Pakete" "FEHLT:$fehlende_apt"
else
    sudo apt-get update && sudo apt-get install -y $fehlende_apt && meldung "apt-Pakete" OK
fi

# --- Python-Pakete (requirements.txt; mujoco/onnxruntime gibt es nicht ueber apt)
if python3 -c "import mujoco, onnxruntime" 2>/dev/null; then
    meldung "Python: mujoco, onnxruntime" OK
elif [ "$PRUEFEN" = 1 ]; then
    meldung "Python: mujoco, onnxruntime" "FEHLT (pip install --user -r requirements.txt)"
else
    pip install --user --break-system-packages -r requirements.txt && meldung "Python-Pakete" OK
fi

# --- Fast Downward (Planer fuer Layer 1, ausserhalb des Repos gebaut)
if [ -x "$FD_ORDNER/fast-downward.py" ] && [ -d "$FD_ORDNER/builds/release" ]; then
    meldung "Fast Downward ($FD_ORDNER)" OK
elif [ "$PRUEFEN" = 1 ]; then
    meldung "Fast Downward" "FEHLT (Skript ohne --pruefen baut $FD_VERSION nach $FD_ORDNER)"
else
    mkdir -p "$(dirname "$FD_ORDNER")"
    git clone --depth 1 --branch "$FD_VERSION" https://github.com/aibasel/downward.git "$FD_ORDNER" \
        && (cd "$FD_ORDNER" && python3 build.py release) && meldung "Fast Downward" OK
fi

# --- Speicher (WSL2): Nav2 + SLAM + Simulation brauchen mehr als die Voreinstellung
mem_gb=$(awk '/MemTotal/ {printf "%.1f", $2/1048576}' /proc/meminfo)
if awk "BEGIN {exit !($mem_gb >= 5.5)}"; then
    meldung "Arbeitsspeicher ($mem_gb GB)" OK
else
    printf '%-45s %s\n' "Arbeitsspeicher ($mem_gb GB)" "HINWEIS: knapp - Nav2 nur headless; WSL2: memory=6GB in .wslconfig"
fi

echo
if [ "$fehlt" = 0 ]; then
    echo "Alles da. Weiter: source /opt/ros/jazzy/setup.bash && colcon build && bash tests/alle_tests.sh --schnell"
else
    echo "Es fehlt etwas (siehe oben)."
fi
exit $fehlt
