"""Tests des Layer-1-Kerns (ohne ROS2): Mission, Problem, Planer (Fast Downward), Executor."""
import threading

import pytest

from roboter_task_planning.deliberative.i_planner import PlanningFailed
from roboter_task_planning.deliberative.pddl_planner import (
    FastDownwardPlanner, find_fast_downward, parse_plan)
from roboter_task_planning.deliberative.problem_generator import generate_problem, read_domain
from roboter_task_planning.executive.action_dispatcher import (
    ABGEBROCHEN, ERFOLG, FEHLGESCHLAGEN, MissionExecutor)
from roboter_task_planning.executive.i_action_bridge import BridgeResult, IActionBridge
from roboter_task_planning.executive.simple_bridges import WarteBridge
from roboter_task_planning.executive.world_knowledge import Mission, WorldKnowledge

MISSION = {
    "orte": {"start": {"x": 0, "y": 0}, "a": {"x": 3, "y": 0}, "b": {"x": 3, "y": 2}, "c": {"x": 10, "y": 0}},
    "verbindungen": [["start", "a"], ["a", "b"], ["b", "start"], ["a", "c"]],
    "start_ort": "start",
    "ziele": [{"art": "besucht", "ort": "b"}, {"art": "balanciert", "ort": "b"}, {"art": "gewartet", "ort": "b"}],
    "parameter": {"warte_s": 0.05, "max_replans": 2},
}


def fd_available():
    try:
        find_fast_downward()
        return True
    except PlanningFailed:
        return False


needs_fd = pytest.mark.skipif(not fd_available(), reason="Fast Downward nicht installiert")


# ---------------- Mission / Problem ----------------
def test_mission_parsing_und_validierung():
    m = Mission.from_dict(MISSION)
    assert m.orte["a"] == (3.0, 0.0) and len(m.verbindungen) == 4
    for bad in ({"orte": {"a": {"x": 0, "y": 0}}, "ziele": [{"art": "fliegt", "ort": "a"}]},
                {"orte": {"a": {"x": 0, "y": 0}}, "ziele": [{"art": "besucht", "ort": "zzz"}]},
                {"orte": {"a": {"x": 0, "y": 0}}, "ziele": []},
                {"orte": {"Küche": {"x": 0, "y": 0}}, "ziele": [{"art": "besucht", "ort": "Küche"}]}):
        with pytest.raises(ValueError):
            Mission.from_dict(bad)


def test_problem_enthaelt_ziele_und_strecken():
    w = WorldKnowledge(Mission.from_dict(MISSION), "start")
    p = generate_problem(w)
    assert "(at start)" in p and "(besucht start)" in p
    assert "(= (strecke start a) 300)" in p and "(verbunden a start)" in p
    assert "(:goal (and (besucht b) (balanciert b) (gewartet b)))" in p


def test_parse_plan():
    plan = parse_plan("(gehe_zu start a)\n(balanciere a)\n; cost = 400 (general cost)\n")
    assert [str(s) for s in plan.schritte] == ["gehe_zu(start, a)", "balanciere(a)"] and plan.kosten == 400


# ---------------- Planer (Fast Downward) ----------------
@needs_fd
def test_planer_findet_kuerzesten_weg():
    d = dict(MISSION)
    d["verbindungen"] = [["start", "a"], ["a", "b"], ["b", "start"]]
    w = WorldKnowledge(Mission.from_dict(d), "start")
    plan = FastDownwardPlanner().plan(read_domain(), generate_problem(w))
    namen = [str(s) for s in plan.schritte]
    # direkter Weg start->b (361 cm) ist kuerzer als start->a->b (500 cm)
    assert namen[0] == "gehe_zu(start, b)" and set(namen[1:]) == {"balanciere(b)", "warte(b)"}
    assert plan.kosten == 361 + 100 + 100


@needs_fd
def test_planer_unloesbar():
    d = dict(MISSION)
    d["verbindungen"] = [["a", "b"]]      # start ist abgeschnitten
    w = WorldKnowledge(Mission.from_dict(d), "start")
    with pytest.raises(PlanningFailed):
        FastDownwardPlanner().plan(read_domain(), generate_problem(w))


# ---------------- Executor ----------------
class Skript(IActionBridge):
    """Test-Bridge: protokolliert Schritte, scheitert an vorgegebenen Aufrufnummern."""
    aktionen = ("gehe_zu", "balanciere")

    def __init__(self, scheitert=(), vor_schritt=None):
        self.log, self.n, self.scheitert, self.vor_schritt = [], 0, set(scheitert), vor_schritt

    def execute(self, s, mission, cancel, orte=None):
        self.n += 1
        if self.vor_schritt:
            self.vor_schritt(self.n, s)
        self.log.append(str(s))
        return BridgeResult(self.n not in self.scheitert, "ok" if self.n not in self.scheitert else "Hindernis")


def executor(bridge, **kw):
    return MissionExecutor(FastDownwardPlanner(), [bridge, WarteBridge()], log=lambda m: None, **kw)


@needs_fd
def test_executor_erfolg():
    b = Skript()
    r = executor(b).run(Mission.from_dict(MISSION), threading.Event())
    assert r.status == ERFOLG and r.replans == 0
    assert b.log[0] == "gehe_zu(start, b)"      # direkter Weg 360 cm < 500 cm ueber a... start-b ist verbunden
    assert "balanciere(b)" in b.log


@needs_fd
def test_executor_neuplanung_nach_fehlschlag():
    b = Skript(scheitert={1})
    pos = [(1.0, 0.0)]                          # Roboter blieb 1 m vom Start stehen
    r = executor(b, locator=lambda: pos[0]).run(Mission.from_dict(MISSION), threading.Event())
    assert r.status == ERFOLG and r.replans == 1
    assert any("pos_aktuell" in s for s in b.log[1:])    # Neuplanung geht von der aktuellen Position aus


@needs_fd
def test_executor_gibt_nach_max_replans_auf():
    b = Skript(scheitert=set(range(1, 50)))
    r = executor(b, locator=lambda: (0.0, 0.0)).run(Mission.from_dict(MISSION), threading.Event())
    assert r.status == FEHLGESCHLAGEN and r.replans == 3


@needs_fd
def test_executor_abbruch_waehrend_plan():
    abort = threading.Event()
    b = Skript(vor_schritt=lambda n, s: abort.set() if n == 2 else None)
    r = executor(b).run(Mission.from_dict(MISSION), abort)
    assert r.status == ABGEBROCHEN and len(b.log) < 3


@needs_fd
def test_executor_keine_bridge():
    r = MissionExecutor(FastDownwardPlanner(), [Skript()], log=lambda m: None) \
        .run(Mission.from_dict(MISSION), threading.Event())
    assert r.status == FEHLGESCHLAGEN and "warte" in r.meldung


# ---------------- Zielart am_ort (Endposition) ----------------
def test_ziel_am_ort_im_problem():
    d = dict(MISSION)
    d["ziele"] = [{"art": "besucht", "ort": "a"}, {"art": "am_ort", "ort": "start"}]
    w = WorldKnowledge(Mission.from_dict(d), "start")
    assert "(at start)" in generate_problem(w).split(":goal")[1]
    assert w.offene_ziele() == [("besucht", "a"), ("am_ort", "start")] or w.offene_ziele() == [("besucht", "a")]
    w.schritt_erledigt("gehe_zu", ["start", "a"])
    assert ("am_ort", "start") in w.offene_ziele()          # Besuch erledigt, aber nicht mehr am Start
    w.schritt_erledigt("gehe_zu", ["a", "start"])
    assert w.offene_ziele() == []


@needs_fd
def test_executor_rundfahrt_zurueck_zum_start():
    d = dict(MISSION)
    d["ziele"] = [{"art": "besucht", "ort": "a"}, {"art": "besucht", "ort": "b"}, {"art": "am_ort", "ort": "start"}]
    b = Skript()
    r = executor(b).run(Mission.from_dict(d), threading.Event())
    assert r.status == ERFOLG
    assert b.log[-1] == "gehe_zu(" + b.log[-1].split("(")[1].split(",")[0] + ", start)"   # letzte Aktion endet am Start
    assert {"gehe_zu(start, a)", "gehe_zu(a, b)"} <= set(b.log) or {"gehe_zu(start, b)", "gehe_zu(b, a)"} <= set(b.log)



# ---------------- Review-Paket 2 ----------------
class KoordBridge(IActionBridge):
    """Wie die NavigationBridge: schlaegt Zielkoordinaten nach (KeyError wie im Review B2)."""
    aktionen = ("gehe_zu", "balanciere")

    def __init__(self, scheitert=()):
        self.log, self.n, self.scheitert = [], 0, set(scheitert)

    def execute(self, s, mission, cancel, orte=None):
        self.n += 1
        if s.name == "gehe_zu":
            (orte or mission.orte)[s.args[1]]          # muss existieren
        self.log(str(s)) if callable(self.log) else self.log.append(str(s))
        return BridgeResult(self.n not in self.scheitert, "ok")


KETTE = {
    "orte": {"start": {"x": 0, "y": 0}, "a": {"x": 3, "y": 0}, "b": {"x": 6, "y": 0}, "c": {"x": 6, "y": 3}},
    "verbindungen": [["start", "a"], ["a", "b"], ["b", "c"]],          # nur eine Kette
    "start_ort": "start",
    "ziele": [{"art": "besucht", "ort": "c"}],
    "parameter": {"max_replans": 2},
}


@needs_fd
def test_neuplanung_durchquert_pos_aktuell_nicht():
    b = KoordBridge(scheitert={2})                   # gehe_zu(a, b) scheitert
    r = executor(b, locator=lambda: (4.5, 0.0)).run(Mission.from_dict(KETTE), threading.Event())
    assert r.status == ERFOLG, r.meldung
    assert not any(x.endswith(", pos_aktuell)") for x in b.log), b.log     # nie ZUM dynamischen Ort
    assert "gehe_zu(pos_aktuell, b)" in b.log                               # vom naechsten Ort aus weiter


def test_dynamischer_ort_hat_nur_ausgehende_kanten():
    w = WorldKnowledge(Mission.from_dict(KETTE), "a")
    w.aktualisiere_position((4.5, 0.0), gescheitertes_ziel="b")
    p = generate_problem(w)
    assert "(verbunden pos_aktuell b)" in p and "(verbunden b pos_aktuell)" not in p
    assert "(verbunden c pos_aktuell)" not in p and "(verbunden pos_aktuell c)" not in p   # c nicht unter den 2 naechsten


@needs_fd
def test_grossbuchstaben_in_ortsnamen():
    d = {"orte": {"Start": {"x": 0, "y": 0}, "K1": {"x": 3, "y": 0}}, "start_ort": "Start",
         "ziele": [{"art": "besucht", "ort": "K1"}]}
    m = Mission.from_dict(d)
    assert set(m.orte) == {"start", "k1"}
    b = KoordBridge()
    assert executor(b).run(m, threading.Event()).status == ERFOLG


def test_reservierter_und_doppelter_ortsname():
    for orte in ({"pos_aktuell": {"x": 0, "y": 0}}, {"a": {"x": 0, "y": 0}, "A": {"x": 1, "y": 0}}):
        with pytest.raises(ValueError):
            Mission.from_dict({"orte": orte, "ziele": [{"art": "besucht", "ort": list(orte)[0]}]})


@needs_fd
def test_ausnahme_in_bridge_beendet_mission_mit_status():
    class Kaputt(IActionBridge):
        aktionen = ("gehe_zu", "balanciere")

        def execute(self, s, mission, cancel, orte=None):
            raise RuntimeError("Bridge kaputt")
    st = []
    r = MissionExecutor(FastDownwardPlanner(), [Kaputt(), WarteBridge()], log=lambda m: None,
                        status_cb=st.append).run(Mission.from_dict(MISSION), threading.Event())
    assert r.status == FEHLGESCHLAGEN and "interner Fehler" in r.meldung
    assert st and st[-1]["phase"] == "ende"


def test_ohne_startort_und_position_abgelehnt():
    d = dict(MISSION)
    d.pop("start_ort")
    r = MissionExecutor(FastDownwardPlanner(), [Skript(), WarteBridge()], log=lambda m: None,
                        locator=lambda: None).run(Mission.from_dict(d), threading.Event())
    assert r.status == FEHLGESCHLAGEN and "Startort unbekannt" in r.meldung


# ---------------- Welt (Orte aus roboter_welten) ----------------
def test_mission_mit_welt_laedt_orte_der_welt():
    welt = {"orte": MISSION["orte"], "verbindungen": MISSION["verbindungen"]}
    auftrag = {k: v for k, v in MISSION.items() if k not in ("orte", "verbindungen")}
    m = Mission.from_dict(dict(auftrag, welt="testwelt"), welt_lader=lambda n: welt)
    assert m.welt == "testwelt" and m.orte["c"] == (10.0, 0.0) and len(m.verbindungen) == 4
    with pytest.raises(ValueError):          # Orte gehoeren in die Welt, nicht zusaetzlich in die Mission
        Mission.from_dict(dict(MISSION, welt="testwelt"), welt_lader=lambda n: welt)
    with pytest.raises(ValueError):          # weder Welt noch Orte
        Mission.from_dict(auftrag)


def test_alle_missionsdateien_passen_zu_ihrer_welt():
    """Jede mitgelieferte Mission verweist auf eine vorhandene Welt und nur auf deren Orte (Quellbaum)."""
    import glob
    import os
    import yaml
    src = os.path.join(os.path.dirname(__file__), "..", "..")
    def lader(n):
        with open(os.path.join(src, "roboter_welten", "welten", n, "orte.yaml")) as f:
            return yaml.safe_load(f)
    dateien = sorted(glob.glob(os.path.join(src, "roboter_task_planning", "missions", "*.yaml")))
    assert len(dateien) >= 5
    for p in dateien:
        with open(p) as f:
            d = yaml.safe_load(f)
        assert d.get("welt"), p
        Mission.from_dict(d, welt_lader=lader)
        assert os.path.exists(os.path.join(src, "roboter_welten", "welten", d["welt"], "szene.xml")), p


def test_unbekannte_missionsparameter_abgelehnt():
    with pytest.raises(ValueError):
        Mission.from_dict(dict(MISSION, parameter={"warte_s": 1.0, "nav_toleranz_m": 0.3}))


def test_wegkosten_ueber_schaetzer_mit_rueckfall_auf_luftlinie():
    """Mit Wegschaetzer (Nav2) zaehlt die echte Weglaenge; liefert er nichts, die Luftlinie."""
    m = Mission.from_dict(MISSION)
    aufrufe = []
    def schaetzer(von, nach):
        aufrufe.append((von, nach))
        return None if nach == (10.0, 0.0) or von == (10.0, 0.0) else 7.5     # Umweg statt 3 m Luftlinie
    w = WorldKnowledge(m, "start", schaetzer)
    assert w.strecke_cm("start", "a") == 750 and w.strecke_cm("a", "start") == 750
    assert len(aufrufe) == 1                                     # je Ortspaar nur einmal gefragt
    assert w.strecke_cm("a", "c") == 700                         # Schaetzer ohne Ergebnis -> Luftlinie 7 m
    assert "(= (strecke start a) 750)" in generate_problem(w)


def test_umweg_aendert_die_reihenfolge():
    """Luftlinie wuerde a vor b nehmen; ist der Weg zu a in Wahrheit lang (Wand), plant er b zuerst."""
    d = {"orte": {"s": {"x": 0, "y": 0}, "a": {"x": 1, "y": 0}, "b": {"x": 0, "y": 2}},
         "start_ort": "s", "ziele": [{"art": "besucht", "ort": "a"}, {"art": "besucht", "ort": "b"}]}
    m = Mission.from_dict(d)
    lang = {frozenset({(0.0, 0.0), (1.0, 0.0)}): 9.0}
    schaetzer = lambda von, nach: lang.get(frozenset({von, nach}))     # noqa: E731
    if not fd_available():
        pytest.skip("Fast Downward nicht installiert")
    plan_luft = FastDownwardPlanner().plan(read_domain(), generate_problem(WorldKnowledge(m, "s")))
    plan_weg = FastDownwardPlanner().plan(read_domain(), generate_problem(WorldKnowledge(m, "s", schaetzer)))
    assert str(plan_luft.schritte[0]) == "gehe_zu(s, a)" and str(plan_weg.schritte[0]) == "gehe_zu(s, b)"


def test_auto_start_nur_mit_simulation():
    """Review 2 B9: ohne /clock-Sender (echte Hardware) wird auto_start nach der Wartezeit abgelehnt."""
    pytest.importorskip("rclpy")
    import time
    from roboter_task_planning import mission_node as mn

    class _Log:
        def __init__(self): self.fehler, self.info_ = [], []
        def error(self, m): self.fehler.append(m)
        def info(self, m): self.info_.append(m)

    def knoten(clock_sender, gestartet_vor_s):
        n = object.__new__(mn.MissionNode)                 # ohne Node.__init__ (kein rclpy-Kontext noetig)
        n._log, n.starts = _Log(), []
        n.get_logger = lambda: n._log
        n.count_publishers = lambda topic: clock_sender if topic == "/clock" else 0
        n._start_mission = lambda: (n.starts.append(1), (True, "gestartet"))[1]
        n._auto_done, n._auto_t0 = False, time.monotonic() - gestartet_vor_s
        return n

    n = knoten(clock_sender=0, gestartet_vor_s=1.0)
    n._auto_start_once()
    assert not n.starts and not n._log.fehler and not n._auto_done          # wartet noch auf die Simulation
    n = knoten(clock_sender=0, gestartet_vor_s=mn.MissionNode.AUTO_START_WARTE_S + 1)
    n._auto_start_once(); n._auto_start_once()
    assert not n.starts and len(n._log.fehler) == 1                          # abgelehnt, einmal gemeldet
    n = knoten(clock_sender=1, gestartet_vor_s=1.0)
    n._auto_start_once(); n._auto_start_once()
    assert n.starts == [1]                                                   # Simulation da: genau einmal gestartet
