"""Layer 1: Greifen/Ablegen (Mission, Problem, Plan, Neuplanung nach Fehlgriff), ohne ROS."""
import threading

import pytest

from roboter_task_planning.deliberative.pddl_planner import FastDownwardPlanner
from roboter_task_planning.deliberative.problem_generator import generate_problem, read_domain
from roboter_task_planning.executive.action_dispatcher import ERFOLG, MissionExecutor
from roboter_task_planning.executive.i_action_bridge import BridgeResult, IActionBridge
from roboter_task_planning.executive.world_knowledge import Mission, WorldKnowledge
from test_layer1 import needs_fd

GREIF_MISSION = {
    "orte": {"start": {"x": 0, "y": 0}, "lager": {"x": 1, "y": 0}, "platz": {"x": 1, "y": 2.4}},
    "verbindungen": [["start", "lager"], ["lager", "platz"], ["start", "platz"]],
    "start_ort": "start",
    "gegenstand": {"name": "Kiste", "ort": "lager"},
    "ziele": [{"art": "abgelegt", "ort": "platz"}, {"art": "am_ort", "ort": "start"}],
    "parameter": {"max_replans": 2},
}


def test_greif_mission_validierung():
    m = Mission.from_dict(GREIF_MISSION)
    assert m.gegenstand == ("kiste", "lager")
    ohne = {k: v for k, v in GREIF_MISSION.items() if k != "gegenstand"}
    schlecht = [ohne,                                                            # abgelegt ohne Gegenstand
                dict(GREIF_MISSION, gegenstand={"name": "kiste"}),               # ort fehlt
                dict(GREIF_MISSION, gegenstand={"name": "kiste", "ort": "zzz"}),
                dict(GREIF_MISSION, gegenstand={"name": "lager", "ort": "lager"}),     # Name = Ortsname
                dict(GREIF_MISSION, gegenstand={"name": "kis te", "ort": "lager"})]
    for bad in schlecht:
        with pytest.raises(ValueError):
            Mission.from_dict(bad)


def test_greif_problem_zustaende():
    w = WorldKnowledge(Mission.from_dict(GREIF_MISSION), "start")
    p = generate_problem(w)
    assert "kiste - gegenstand" in p and "(liegt_bei kiste lager)" in p and "(haende_frei)" in p
    assert "(abgelegt kiste platz)" in p.split(":goal")[1]
    w.schritt_erledigt("gehe_zu", ["start", "lager"])
    w.schritt_erledigt("greife", ["kiste", "lager"])
    p = generate_problem(w)
    assert "(haelt kiste)" in p and "(haende_frei)" not in p and "liegt_bei" not in p
    w.schritt_erledigt("gehe_zu", ["lager", "platz"])
    w.schritt_erledigt("ablegen", ["kiste", "platz"])
    assert ("abgelegt", "platz") in w.fakten and not w.haelt and w.gegenstand_ort == "platz"
    assert w.offene_ziele() == [("am_ort", "start")]


@needs_fd
def test_greif_plan_reihenfolge():
    w = WorldKnowledge(Mission.from_dict(GREIF_MISSION), "start")
    plan = FastDownwardPlanner().plan(read_domain(), generate_problem(w))
    assert [str(s) for s in plan.schritte] == [
        "gehe_zu(start, lager)", "greife(kiste, lager)", "gehe_zu(lager, platz)",
        "ablegen(kiste, platz)", "gehe_zu(platz, start)"]


class GreifSkript(IActionBridge):
    aktionen = ("gehe_zu", "greife", "ablegen")

    def __init__(self, scheitert=()):
        self.log, self.n, self.scheitert = [], 0, set(scheitert)

    def execute(self, s, mission, cancel, orte=None):
        self.n += 1
        self.log.append(str(s))
        ok = self.n not in self.scheitert
        return BridgeResult(ok, "ok" if ok else "nicht gegriffen")


def executor(bridge, **kw):
    return MissionExecutor(FastDownwardPlanner(), [bridge], log=lambda m: None, **kw)


@needs_fd
def test_greif_executor_erfolg_und_wiederholung():
    b = GreifSkript()
    r = executor(b).run(Mission.from_dict(GREIF_MISSION), threading.Event())
    assert r.status == ERFOLG and r.replans == 0
    # Fehlgriff (2. Aufruf = greife): neu planen, die Kiste liegt noch am Lager, der zweite Versuch gelingt
    b = GreifSkript(scheitert={2})
    r = executor(b, locator=lambda: (1.0, 0.0)).run(Mission.from_dict(GREIF_MISSION), threading.Event())
    assert r.status == ERFOLG and r.replans == 1
    assert b.log.count("ablegen(kiste, platz)") == 1 and sum(s.startswith("greife") for s in b.log) == 2


class NichtWiederholbar(GreifSkript):
    def execute(self, s, mission, cancel, orte=None):
        r = super().execute(s, mission, cancel, orte)
        if s.name == "ablegen":
            return BridgeResult(False, "Kiste liegt falsch", wiederholbar=False)
        return r


@needs_fd
def test_nicht_wiederholbarer_fehlschlag_beendet_die_mission_ohne_neuplanung():
    b = NichtWiederholbar()
    r = executor(b, locator=lambda: (1.0, 2.4)).run(Mission.from_dict(GREIF_MISSION), threading.Event())
    assert r.status == "fehlgeschlagen" or r.status != ERFOLG
    assert r.replans == 0 and "nicht wiederholbar" in r.meldung
