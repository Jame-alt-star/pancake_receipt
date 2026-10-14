"""Posenaufzeichnung fuer Demo-Videos (Offline-Rendering, demo/render_demo.py): (Wandzeit, Simzeit, qpos) mit
25 Hz in eine .npz-Datei; alle 15 s und beim Beenden geschrieben, damit ein Abbruch wenig verliert."""
import time

import numpy as np

TAKT_S = 0.04          # 25 Hz Simzeit
SCHREIBEN_S = 15.0     # Wandzeit


class Posenaufnahme:

    def __init__(self, pfad: str):
        self.pfad = pfad
        self._daten = {"wall": [], "t": [], "qpos": []}
        self._naechste = 0.0
        self._geschrieben = time.time()

    def schritt(self, sim_zeit: float, qpos):
        if not self.pfad or sim_zeit < self._naechste:
            return
        self._naechste = sim_zeit + TAKT_S
        self._daten["wall"].append(time.time())
        self._daten["t"].append(float(sim_zeit))
        self._daten["qpos"].append(np.array(qpos, copy=True))
        if time.time() - self._geschrieben > SCHREIBEN_S:
            self.schreiben()

    def schreiben(self):
        if not self.pfad or not self._daten["t"]:
            return
        self._geschrieben = time.time()
        np.savez_compressed(self.pfad, wall=np.array(self._daten["wall"]), t=np.array(self._daten["t"]),
                            qpos=np.array(self._daten["qpos"]))
