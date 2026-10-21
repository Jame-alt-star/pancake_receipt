"""Optionale Aufzeichnung eines Systemtests fuer das Testvideo (demo/video_tests.py).

Mit --aufnahme <ordner> schreibt ein Systemtest
  pose.npz          Roboterpose aus der Simulation (record_path, 25 Hz Simzeit, mit Wandzeit)
  ereignisse.json   Testschritte, Pruefergebnisse und Missionsstatus mit Wandzeit (time.time(), dieselbe Uhr wie
                    die Posenaufnahme - so lassen sich beide zuordnen)
  lauf.json         Titel, Welt, Mission, Konfiguration und Ergebnis
Ohne --aufnahme tut der Baustein nichts; der Test laeuft wie immer.
"""
import json
import os
import time


class Aufnahme:
    def __init__(self, ordner, titel, **lauf):
        self.ordner = ordner
        self.ereignisse = []
        self.lauf = {"titel": titel, **lauf}
        if ordner:
            os.makedirs(ordner, exist_ok=True)

    @staticmethod
    def ordner_aus(argv):
        return argv[argv.index("--aufnahme") + 1] if "--aufnahme" in argv else None

    def launch_args(self):
        return [f"record_path:={os.path.join(self.ordner, 'pose.npz')}"] if self.ordner else []

    def ereignis(self, art, **daten):
        if self.ordner:
            self.ereignisse.append({"wall": time.time(), "art": art, **daten})

    def schritt(self, text):
        print(text, flush=True)
        self.ereignis("schritt", text=text.lstrip("= ").strip())

    def check(self, name, ok, detail=""):
        self.ereignis("check", name=name, ok=bool(ok), detail=str(detail))

    def speichern(self, bestanden, gesamt):
        if not self.ordner:
            return
        self.lauf.update(bestanden=bestanden, gesamt=gesamt)
        with open(os.path.join(self.ordner, "ereignisse.json"), "w") as f:
            json.dump(self.ereignisse, f, ensure_ascii=False, indent=1)
        with open(os.path.join(self.ordner, "lauf.json"), "w") as f:
            json.dump(self.lauf, f, ensure_ascii=False, indent=1)
