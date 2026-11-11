"""Greifparameter: alle Felder muessen Zahlen sein, weil der Knoten sie als ROS-Parameter deklariert."""
import dataclasses

from roboter_steuerung.greifen.dienst import Greifparameter


def test_alle_felder_sind_zahlen():
    d = Greifparameter()
    for f in dataclasses.fields(Greifparameter):
        assert isinstance(float(getattr(d, f.name)), float), f.name
