"""Gemeinsame pytest-Einstellung: die schnellen Tests pruefen den QUELLBAUM.

Alle Paketverzeichnisse unter src/ kommen vorne in den Suchpfad. Vorher importierten einige Tests aus dem
Install-Verzeichnis (nach colcon build) und andere aus dem Quellbaum - ohne Build testeten sie verschiedenen Code.
Erzeugte Schnittstellen (roboter_interfaces) gibt es nur im Install-Verzeichnis; dafuer muss der Workspace
gesourct sein. Die Systemtests (tests/system) starten dagegen bewusst das gebaute System.
"""
import os
import sys

_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "src")
for _paket in sorted(os.listdir(_SRC), reverse=True):
    _pfad = os.path.join(_SRC, _paket)
    if os.path.isfile(os.path.join(_pfad, "setup.py")) and _pfad not in sys.path:
        sys.path.insert(0, _pfad)
