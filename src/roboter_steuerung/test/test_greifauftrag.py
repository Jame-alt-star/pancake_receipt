"""Greifauftrag (ROS-frei): Schutz gegen verlorenen Gegenstand beim Ablegen."""
import numpy as np

from roboter_steuerung.greifen.dienst import Eingang, Greifauftrag, Greifparameter


def eingang(t, objekt):
    return Eingang(t=t, pos=np.zeros(3), quat=np.array([1.0, 0, 0, 0]), gelenke=np.zeros(29, dtype=np.float32),
                   objekt=objekt, ablage=([1.0, 2.0, -0.1], np.array([1.0, 0, 0, 0])))


def test_ablegen_ohne_gegenstand_in_den_haenden_bricht_ab_statt_zu_planen():
    ruhe = np.zeros(29, dtype=np.float32)
    g = Greifauftrag("ablegen", None, ruhe, Greifparameter(), start_arme=np.zeros(14), start_taille=0.3)
    verloren = ([-0.36, 0.47, -0.76], np.array([1.0, 0, 0, 0]))        # Kiste liegt hinter dem Roboter am Boden
    a = g.takt(eingang(0.0, verloren))
    t = 0.0
    while not a.fertig and t < 30.0:
        t += 0.02
        a = g.takt(eingang(t, verloren))
    assert a.fertig and not a.erfolg and not a.wiederholbar and "verloren" in a.meldung
    assert g.frei


def test_ablegen_mit_gegenstand_in_der_tragepose_laeuft_los():
    ruhe = np.zeros(29, dtype=np.float32)
    g = Greifauftrag("ablegen", None, ruhe, Greifparameter(), start_arme=np.zeros(14), start_taille=0.3)
    gehalten = ([0.25, 0.0, 0.11], np.array([1.0, 0, 0, 0]))
    a = g.takt(eingang(0.0, gehalten))
    assert not a.fertig and g.phase == "anfahren"
