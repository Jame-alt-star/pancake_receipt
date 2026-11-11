"""Robustheitsreihe: ganzer Ablauf mit veraenderter Kisten- und Startlage."""
import sys, time
sys.path.insert(0, 'tests/simulation'); sys.path.insert(0, 'src/roboter_beschreibung')
import numpy as np
from greif_helpers import Kreis
from roboter_steuerung.greifen.anfahrt import Anfahrt
from roboter_steuerung.greifen.dienst import FUEHREN

FAELLE = {
    "nominal":        dict(start=(0.7, 0.0, 0.0), kiste=None),
    "kiste +5cm y":   dict(start=(0.7, 0.0, 0.0), kiste=(1.95, 0.05, 0.0)),
    "kiste -6cm y":   dict(start=(0.7, 0.0, 0.0), kiste=(1.95, -0.06, 0.0)),
    "kiste +10 Grad": dict(start=(0.7, 0.0, 0.0), kiste=(1.95, 0.0, np.radians(10))),
    "kiste -12 Grad": dict(start=(0.7, 0.0, 0.0), kiste=(1.97, 0.03, np.radians(-12))),
    "kiste hinten":   dict(start=(0.7, 0.0, 0.0), kiste=(2.02, 0.0, 0.0)),
    "start schraeg":  dict(start=(0.6, 0.2, 0.6), kiste=None),
    "start weit":     dict(start=(0.45, -0.2, -0.4), kiste=None),
}
for name in (sys.argv[1:] or FAELLE):
    f = FAELLE[name]
    t0 = time.time()
    k = Kreis(start=f["start"], kiste=f["kiste"])
    k.laufe(2.0)
    g, a = k.auftrag("greifen")
    zeile = f"{name:15s} greifen: {a.erfolg} ({a.meldung})"
    if a.erfolg:
        af = Anfahrt((0.7, 2.4, 0.0), tol_pos=0.25, tol_yaw=np.pi, v=0.5, w=1.0, nah=0.0, quer_beim_drehen=0.3, v_bogen=0.3, linie=0.0)

        def takt(kk):
            vx, vy, wz, ok = af.befehl(kk.basis(), kk.t)
            kk.cmd[:] = (vx, vy, wz)
            kk.arm_kommando(FUEHREN, g.arme, g.taille)
            return ok
        k.laufe(40, takt); k.cmd[:] = 0
        g2, a2 = k.auftrag("ablegen", ablage_site="ablage_arbeitsplatz", vorher=g)
        k.laufe(2.0)
        kp, _ = k.kiste()
        d = (kp - np.array([1.95, 2.40, 0.79])) * 100
        zeile += f" | ablegen: {a2.erfolg} ({a2.meldung}), Abweichung {np.round(d, 1)} cm"
    print(f"{zeile} | Neigung max {k.max_neigung:.1f}, Sturz {k.gestuerzt}, Simzeit {k.t:.0f} s, "
          f"Rechenzeit {time.time() - t0:.0f} s", flush=True)
