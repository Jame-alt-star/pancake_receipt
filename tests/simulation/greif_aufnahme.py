#!/usr/bin/env python3
"""Nimmt den vollstaendigen Greif-Ablauf im Regelkreis ohne ROS fuer das Testvideo auf (demo/video_tests.py).

Aufruf: python3 tests/simulation/greif_aufnahme.py <ordner> [fall]     (fall: Name aus greif_serie_zufall, Standard nominal)
Schreibt pose.npz, ereignisse.json, lauf.json in <ordner> (Format wie tests/system/testaufnahme.py).
Nav2 wird hier durch die Ersatz-Anfahrt der Serie ersetzt; der Rest (Greif-Dienst, Armfuehrung, Policy, Physik) ist
derselbe Code wie im ROS-System.
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src", "roboter_beschreibung"))
import numpy as np  # noqa: E402

from greif_helpers import Kreis, roboter_config  # noqa: E402
from roboter_steuerung.greifen.anfahrt import Anfahrt  # noqa: E402
from roboter_steuerung.greifen.dienst import FUEHREN  # noqa: E402

T0 = 1.0e9          # Wandzeit-Ursprung der Aufnahme (nur zur Zuordnung von Ereignissen und Pose)
TITEL = "Greifen Phase 1: Kiste holen und ablegen (Regelkreis ohne ROS)"


def main():
    ordner = sys.argv[1]
    os.makedirs(ordner, exist_ok=True)
    k = Kreis(start=(0.7, 0.0, 0.0))
    ts, qs = [], []
    n = [0]

    def aufnehmen(kk):
        n[0] += 1
        if n[0] % 2 == 0:               # 25 Hz
            ts.append(kk.t)
            qs.append(kk.d.qpos.copy())
    k.beobachter = aufnehmen
    ev = []

    def e(art, **d):
        ev.append({"wall": T0 + k.t, "art": art, **d})

    plan = ["greife(kiste, lager)", "gehe_zu(lager, arbeitsplatz)", "ablegen(kiste, arbeitsplatz)"]
    k.laufe(2.0)
    e("schritt", text="G1 Roboter steht am Lagertisch-Vorplatz (lager), Kiste auf dem Tisch")
    e("mission", phase="plan", plan=plan, replans=0)
    e("mission", phase="schritt", schritt=plan[0], index=0, von=3)
    g, a = k.auftrag("greifen")
    e("check", name="Kiste gegriffen", ok=a.erfolg, detail=a.meldung)
    e("mission", phase="schritt", schritt=plan[1], index=1, von=3)
    e("schritt", text="Transport zum Arbeitsplatz (Ersatz fuer Nav2)")
    af = Anfahrt((0.7, 2.4, 0.0), tol_pos=0.25, tol_yaw=np.pi, v=0.5, w=1.0, nah=0.0, quer_beim_drehen=0.3,
                 v_bogen=0.3, linie=0.0)

    def takt(kk):
        vx, vy, wz, ok = af.befehl(kk.basis(), kk.t)
        kk.cmd[:] = (vx, vy, wz)
        kk.arm_kommando(FUEHREN, g.arme, g.taille)
        return ok
    k.laufe(40, takt)
    k.cmd[:] = 0
    e("mission", phase="schritt", schritt=plan[2], index=2, von=3)
    g2, a2 = k.auftrag("ablegen", ablage_site="ablage_arbeitsplatz", vorher=g)
    k.laufe(3.0)
    kp, _ = k.kiste()
    d = (kp - np.array([1.95, 2.40, 0.79])) * 100
    e("check", name="Kiste an der Ablagestelle", ok=a2.erfolg, detail=f"Abweichung {np.round(d, 1)} cm")
    e("check", name="Roboter nicht gestuerzt", ok=k.gestuerzt is None, detail=f"max. Neigung {k.max_neigung:.1f} Grad")
    e("mission", phase="ende", status="erfolg" if a2.erfolg and a.erfolg else "fehlgeschlagen", replans=0)
    ok = [x["ok"] for x in ev if x["art"] == "check"]
    np.savez(os.path.join(ordner, "pose.npz"), wall=T0 + np.array(ts), t=np.array(ts), qpos=np.array(qs),
             modell=np.array(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "src",
                                                          "roboter_beschreibung",
                                                          roboter_config()["modell_xml_haende"]))),
             welt=np.array("greiftisch"))
    json.dump(ev, open(os.path.join(ordner, "ereignisse.json"), "w"), ensure_ascii=False, indent=1)
    json.dump({"titel": TITEL, "welt": "greiftisch", "mission": "greiftisch", "navigation": "testdouble",
               "realistisch": False, "beinodometrie": False, "bestanden": sum(ok), "gesamt": len(ok)},
              open(os.path.join(ordner, "lauf.json"), "w"), ensure_ascii=False, indent=1)
    print(f"{len(ts)} Bilder, Ergebnis {a.meldung} / {a2.meldung}, Abweichung {np.round(d, 1)} cm")


if __name__ == "__main__":
    main()
