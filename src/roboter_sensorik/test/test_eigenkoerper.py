"""Eigenkoerper-Maske: getragener Gegenstand vor dem Becken ist kein Hindernis, Tischkante dahinter schon."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from roboter_sensorik.wolke import eigenkoerper_maske  # noqa: E402


def test_kiste_in_tragepose_wird_maskiert_tischkante_nicht():
    kiste = np.array([[0.14, 0.0, 0.10], [0.34, 0.07, 0.15], [0.24, -0.10, 0.05]])     # Vorder-/Seiten-/Unterseite
    tisch = np.array([[0.90, 0.0, -0.05], [0.60, 0.0, 0.0]])                          # ausserhalb
    seite = np.array([[0.25, 0.60, 0.1]])                                             # Mensch seitlich daneben
    assert eigenkoerper_maske(kiste).all()
    assert not eigenkoerper_maske(tisch).any()
    assert not eigenkoerper_maske(seite).any()
