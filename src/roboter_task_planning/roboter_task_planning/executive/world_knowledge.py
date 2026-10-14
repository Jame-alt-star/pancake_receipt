"""Weltwissen und Mission (Layer 1): Orte, Verbindungen, Ziele, erreichte Fakten."""
import itertools
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

import yaml

ZIEL_ARTEN = ("besucht", "balanciert", "gewartet", "am_ort")   # am_ort = Endposition
DYNAMISCHER_ORT = "pos_aktuell"      # wird bei Neuplanung an der aktuellen Position eingefuegt


@dataclass
class Mission:
    orte: Dict[str, Tuple[float, float]]
    verbindungen: Set[frozenset]
    ziele: List[Tuple[str, str]]               # (art, ort)
    start_ort: Optional[str] = None
    parameter: Dict[str, float] = field(default_factory=dict)

    PARAMETER_DEFAULTS = {
        "warte_s": 3.0, "balancier_toleranz_grad": 5.0, "nav_toleranz_m": 0.3,
        "nav_geschwindigkeit": 0.4, "max_replans": 2,
    }

    def param(self, name):
        return self.parameter.get(name, self.PARAMETER_DEFAULTS[name])

    @staticmethod
    def from_dict(d: dict) -> "Mission":
        # Ortsnamen werden kleingeschrieben (Review B4): Fast Downward gibt Objektnamen klein aus,
        # mit "K1" in der YAML schlug sonst das Nachschlagen der Koordinaten fehl.
        def name(n):
            return str(n).lower()
        orte = {}
        for k, v in d["orte"].items():
            n = name(k)
            if not n.replace("_", "").isalnum() or not n.isascii():
                raise ValueError(f"Ortsname {k!r}: nur ASCII-Buchstaben, Ziffern und _ erlaubt")
            if n == DYNAMISCHER_ORT:
                raise ValueError(f"Ortsname {k!r} ist reserviert (Neuplanung)")
            if n in orte:
                raise ValueError(f"Ortsname {k!r} doppelt (Gross-/Kleinschreibung zaehlt nicht)")
            orte[n] = (float(v["x"]), float(v["y"]))
        if d.get("verbindungen"):
            verb = {frozenset(name(x) for x in p) for p in d["verbindungen"]}
            for p in verb:
                if len(p) != 2 or not p <= set(orte):
                    raise ValueError(f"ungueltige Verbindung {sorted(p)}")
        else:
            verb = {frozenset(p) for p in itertools.combinations(orte, 2)}
        ziele = []
        for z in d.get("ziele", []):
            art, ort = str(z["art"]), name(z["ort"])
            if art not in ZIEL_ARTEN:
                raise ValueError(f"unbekannte Zielart {art!r}, erlaubt: {ZIEL_ARTEN}")
            if ort not in orte:
                raise ValueError(f"Ziel referenziert unbekannten Ort {ort!r}")
            ziele.append((art, ort))
        if not ziele:
            raise ValueError("Mission ohne Ziele")
        start = d.get("start_ort")
        start = name(start) if start else None
        if start is not None and start not in orte:
            raise ValueError(f"start_ort {start!r} unbekannt")
        return Mission(orte, verb, ziele, start, dict(d.get("parameter", {})))

    @staticmethod
    def from_yaml(path: str) -> "Mission":
        with open(path, "r") as f:
            return Mission.from_dict(yaml.safe_load(f))


class WorldKnowledge:
    """Veraenderlicher Zustand waehrend der Missionsausfuehrung."""

    def __init__(self, mission: Mission, current_ort: str):
        self.mission = mission
        self.orte = dict(mission.orte)
        self.verbindungen = set(mission.verbindungen)
        self.current_ort = current_ort
        self.fakten: Set[Tuple[str, str]] = {("besucht", current_ort)}
        # gerichtete Kanten NUR vom dynamischen Ort weg (siehe aktualisiere_position)
        self.dyn_ziele: Set[str] = set()

    @staticmethod
    def naechster_ort(orte: Dict[str, Tuple[float, float]], pos) -> str:
        return min(orte, key=lambda n: math.hypot(orte[n][0] - pos[0], orte[n][1] - pos[1]))

    def strecke_cm(self, a: str, b: str) -> int:
        (ax, ay), (bx, by) = self.orte[a], self.orte[b]
        return max(1, int(round(100.0 * math.hypot(ax - bx, ay - by))))

    def aktualisiere_position(self, pos, gescheitertes_ziel: Optional[str] = None, n_naechste: int = 2):
        """Nach einem Fehlschlag steht der Roboter evtl. zwischen Orten: dynamischer Ort an pos.

        Er bekommt nur AUSGEHENDE Kanten (Review B2/L4): zu den n naechsten Orten und zum Ziel des
        gescheiterten Schritts. Vorher war er in beide Richtungen mit ALLEN Orten verbunden; der
        Planer konnte ihn dann als Abkuerzung durchqueren (gehe_zu(k1, pos_aktuell)) und die
        Topologie der Mission (z. B. 'nur durch die Tuer') ging verloren.
        """
        self.orte[DYNAMISCHER_ORT] = (float(pos[0]), float(pos[1]))
        nah = sorted(self.mission.orte, key=lambda n: math.hypot(self.mission.orte[n][0] - pos[0],
                                                                  self.mission.orte[n][1] - pos[1]))
        self.dyn_ziele = set(nah[:n_naechste])
        if gescheitertes_ziel in self.mission.orte:
            self.dyn_ziele.add(gescheitertes_ziel)
        self.current_ort = DYNAMISCHER_ORT

    def schritt_erledigt(self, name: str, args: List[str]):
        if name == "gehe_zu":
            self.current_ort = args[1]
            self.fakten.add(("besucht", args[1]))
        elif name == "balanciere":
            self.fakten.add(("balanciert", args[0]))
        elif name == "warte":
            self.fakten.add(("gewartet", args[0]))

    def offene_ziele(self):
        def erfuellt(z):
            art, ort = z
            if art == "am_ort":
                return self.current_ort == ort
            return z in self.fakten
        return [z for z in self.mission.ziele if not erfuellt(z)]
