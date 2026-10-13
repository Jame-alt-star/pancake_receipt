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

    def execute(self, s, mission, cancel):
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
