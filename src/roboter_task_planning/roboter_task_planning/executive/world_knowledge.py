"""Weltwissen und Mission (Layer 1): Orte, Verbindungen, Ziele, erreichte Fakten."""
import itertools
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple

import yaml

ZIEL_ARTEN = ("besucht", "balanciert", "gewartet", "am_ort", "abgelegt")   # am_ort = Endposition
DYNAMISCHER_ORT = "pos_aktuell"      # wird bei Neuplanung an der aktuellen Position eingefuegt


@dataclass
class Mission:
    """Unveraenderliche Missionsbeschreibung: Orte (map-Frame, m), ungerichtete Verbindungen,
    Ziele (art, ort), optionaler Startort und Parameter (Defaults unten).

    Orte und Verbindungen gehoeren zur Umgebung, nicht zum Auftrag: eine Missionsdatei verweist mit
    'welt: <name>' auf roboter_welten/welten/<name>/orte.yaml. Direkt angegebene 'orte' bleiben fuer
    Unit-Tests und schnelle Versuche erlaubt, aber nicht zusammen mit 'welt'."""
    orte: Dict[str, Tuple[float, float]]
    verbindungen: Set[frozenset]
    ziele: List[Tuple[str, str]]               # (art, ort)
    start_ort: Optional[str] = None
    parameter: Dict[str, float] = field(default_factory=dict)
    welt: Optional[str] = None
    gegenstand: Optional[Tuple[str, str]] = None    # (name, ort): der eine Gegenstand, den der Roboter bewegt


    # nav_geschwindigkeit wirkt nur im GehAktion-Backend (Testdouble); mit Nav2 gelten dessen Parameter.
    PARAMETER_DEFAULTS = {
        "warte_s": 3.0, "balancier_toleranz_grad": 5.0, "nav_geschwindigkeit": 0.4, "max_replans": 2,
    }

    def param(self, name):
        return self.parameter.get(name, self.PARAMETER_DEFAULTS[name])

    @staticmethod
    def from_dict(d: dict, welt_lader=None) -> "Mission":
        """welt_lader(name) -> dict mit 'orte'/'verbindungen'; Standard: roboter_welten.lade_orte."""
        welt = d.get("welt")
        if welt:
            if "orte" in d or "verbindungen" in d:
                raise ValueError("Mission mit 'welt' darf keine eigenen 'orte'/'verbindungen' enthalten")
            if welt_lader is None:
                from roboter_welten import lade_orte as welt_lader
            w = welt_lader(str(welt))
            d = dict(d, orte=w["orte"], verbindungen=w.get("verbindungen"))
        elif "orte" not in d:
            raise ValueError("Mission braucht 'welt' (oder fuer Tests direkt 'orte')")
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
        parameter = dict(d.get("parameter") or {})
        unbekannt = set(parameter) - set(Mission.PARAMETER_DEFAULTS)
        if unbekannt:
            # Frueher still ignoriert: ein Tippfehler (oder das entfernte nav_toleranz_m) blieb ohne Wirkung.
            raise ValueError(f"unbekannte Parameter {sorted(unbekannt)}, erlaubt: {sorted(Mission.PARAMETER_DEFAULTS)}")
        gegenstand = None
        g = d.get("gegenstand")
        if g is not None:
            if not isinstance(g, dict) or set(g) != {"name", "ort"}:
                raise ValueError("'gegenstand' braucht genau 'name' und 'ort'")
            gname, gort = name(g["name"]), name(g["ort"])
            if not gname.replace("_", "").isalnum() or not gname.isascii():
                raise ValueError(f"Gegenstandsname {g['name']!r}: nur ASCII-Buchstaben, Ziffern und _ erlaubt")
            if gname in orte:
                raise ValueError(f"Gegenstandsname {gname!r} kollidiert mit einem Ortsnamen")
            if gort not in orte:
                raise ValueError(f"Gegenstand liegt an unbekanntem Ort {gort!r}")
            gegenstand = (gname, gort)
        if any(art == "abgelegt" for art, _ in ziele) and gegenstand is None:
            raise ValueError("Ziel 'abgelegt' braucht einen 'gegenstand' in der Mission")
        return Mission(orte, verb, ziele, start, parameter, str(welt) if welt else None, gegenstand)

    @staticmethod
    def from_yaml(path: str) -> "Mission":
        with open(path, "r") as f:
            return Mission.from_dict(yaml.safe_load(f))


class WorldKnowledge:
    """Veraenderlicher Zustand waehrend der Missionsausfuehrung: aktueller Ort, erreichte Fakten,
    Orte inkl. dynamischem Ort der Neuplanung. Wird nach jedem erfolgreichen Schritt aktualisiert;
    daraus erzeugt problem_generator das naechste PDDL-Problem."""

    def __init__(self, mission: Mission, current_ort: str, wegschaetzer=None):
        """wegschaetzer(von_xy, nach_xy) -> Weglaenge [m] oder None (z. B. ueber den Nav2-Planer). Ohne ihn - oder
        wenn er nichts liefert - gilt die Luftlinie. Die Luftlinie unterschaetzt Wege um Waende und durch Tueren
        stark und konnte deshalb eine falsche Reihenfolge der Ziele liefern."""
        self.mission = mission
        self.wegschaetzer = wegschaetzer
        self._weg_cache: Dict[frozenset, int] = {}
        self.orte = dict(mission.orte)
        self.verbindungen = set(mission.verbindungen)
        self.current_ort = current_ort
        self.fakten: Set[Tuple[str, str]] = {("besucht", current_ort)}
        # Gegenstand: Ort, an dem er liegt (None solange er gehalten wird); abgelegt: Orte, an denen er abgelegt wurde
        self.gegenstand_ort: Optional[str] = mission.gegenstand[1] if mission.gegenstand else None
        self.haelt: bool = False
        # gerichtete Kanten NUR vom dynamischen Ort weg (siehe aktualisiere_position)
        self.dyn_ziele: Set[str] = set()

    @staticmethod
    def naechster_ort(orte: Dict[str, Tuple[float, float]], pos) -> str:
        return min(orte, key=lambda n: math.hypot(orte[n][0] - pos[0], orte[n][1] - pos[1]))

    def strecke_cm(self, a: str, b: str) -> int:
        """Wegkosten in cm (ganzzahlig, Fast Downward verlangt ganzzahlige Kosten); je Ortspaar einmal geschaetzt."""
        schluessel = frozenset((a, b, self.orte[a], self.orte[b]))
        if schluessel not in self._weg_cache:
            (ax, ay), (bx, by) = self.orte[a], self.orte[b]
            meter = self.wegschaetzer((ax, ay), (bx, by)) if self.wegschaetzer else None
            if meter is None:
                meter = math.hypot(ax - bx, ay - by)
            self._weg_cache[schluessel] = max(1, int(round(100.0 * meter)))
        return self._weg_cache[schluessel]

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
        elif name == "greife":
            self.gegenstand_ort, self.haelt = None, True
        elif name == "ablegen":
            self.gegenstand_ort, self.haelt = args[1], False
            self.fakten.add(("abgelegt", args[1]))

    def offene_ziele(self):
        def erfuellt(z):
            art, ort = z
            if art == "am_ort":
                return self.current_ort == ort
            return z in self.fakten
        return [z for z in self.mission.ziele if not erfuellt(z)]
