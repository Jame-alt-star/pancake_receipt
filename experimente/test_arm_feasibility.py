"""Regression: Greifen im Stehen innerhalb des gemessenen Betriebspunkts.

Betriebspunkt (13.10.2026, synchroner Kreis): Policy sieht die Arme in Ruhepose (Armbewegung =
Stoerung), Taille 0.15 rad nach hinten (Gegengewicht), Haende 0.30 m vor dem Becken,
Last bis 1 kg je Hand. Siehe docs/statusupdate_g1_projekt.md.
"""
import pytest
from arm_feasibility import run

KW = dict(obs_mask=True, obs_blend=0.0, waist=(0.0, 0.0, -0.15), y_lat=0.15)


@pytest.mark.parametrize("load", [0.0, 0.5, 1.0])
def test_greifen_im_stehen_bis_1kg(load):
    r = run(0.30, load, 0.15, 0.0, **KW)
    assert r["fell"] is None
    assert r["drift"] < 0.10, f"Drift {r['drift']*100:.1f} cm"
    assert r["end_drift"] < 0.05, f"Endabweichung {r['end_drift']*100:.1f} cm"
    assert r["tilt"] < 10, f"Neigung {r['tilt']:.1f} Grad"


def test_ohne_maskierung_faellt_bei_grosser_reichweite():
    """Dokumentiert die Ursache: unmaskierte Arm-Beobachtungen fuehren bei 0.30 m zum Sturz."""
    r = run(0.30, 0.0, 0.15, 0.0)
    assert r["fell"] is not None
