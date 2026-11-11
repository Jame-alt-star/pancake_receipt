"""Posenaufzeichnung fuer Videos (Offline-Rendering, demo/render_demo.py und demo/video_tests.py): (Wandzeit, Simzeit,
qpos) mit 25 Hz in eine .npz-Datei; alle 5 s und beim Beenden geschrieben, damit ein Abbruch wenig verliert.

Geschrieben wird erst in eine temporaere Datei und dann umbenannt, unter einer Sperre: mit Viewer-Fenster endete der
Simulationsthread beim Beenden nicht immer rechtzeitig, schrieb dann gleichzeitig mit destroy_node, und der Prozess wurde
mitten im Schreiben hart beendet - beide Aufzeichnungen der Pipeline-Tests waren danach unlesbar (21.10.2026).
"""
import os
import threading
import time

import numpy as np

TAKT_S = 0.04          # 25 Hz Simzeit
SCHREIBEN_S = 5.0      # Wandzeit


class Posenaufnahme:

    def __init__(self, pfad: str, modell: str = "", welt: str = ""):
        """modell (Pfad des Robotermodells) und welt werden mitgespeichert: mit Haenden oder frei beweglichen
        Gegenstaenden hat qpos eine andere Laenge, das Video muss dasselbe Modell laden."""
        self.pfad = pfad
        self.modell, self.welt = modell, welt
        self._daten = {"wall": [], "t": [], "qpos": []}
        self._naechste = 0.0
        self._geschrieben = time.time()
        self._sperre = threading.Lock()

    def schritt(self, sim_zeit: float, qpos):
        if not self.pfad or sim_zeit < self._naechste:
            return
        self._naechste = sim_zeit + TAKT_S
        with self._sperre:
            self._daten["wall"].append(time.time())
            self._daten["t"].append(float(sim_zeit))
            self._daten["qpos"].append(np.array(qpos, copy=True))
        if time.time() - self._geschrieben > SCHREIBEN_S:
            self.schreiben()

    def schreiben(self):
        if not self.pfad:
            return
        with self._sperre:
            if not self._daten["t"]:
                return
            self._geschrieben = time.time()
            # Unkomprimiert: schneller (der Simulationsthread schreibt selbst), die Dateien bleiben wenige MB.
            tmp = f"{self.pfad}.{os.getpid()}.tmp.npz"
            np.savez(tmp, wall=np.array(self._daten["wall"]), t=np.array(self._daten["t"]),
                     qpos=np.array(self._daten["qpos"]), modell=np.array(self.modell), welt=np.array(self.welt))
            os.replace(tmp, self.pfad)
