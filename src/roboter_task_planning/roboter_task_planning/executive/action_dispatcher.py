"""Dispatcher (Layer 1, executive): Mission -> Plan -> Ausfuehrung ueber Bridges, mit Neuplanung."""
import threading
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

from roboter_task_planning.deliberative.i_planner import IPlanner, PlanningFailed
from roboter_task_planning.deliberative.problem_generator import generate_problem, read_domain
from roboter_task_planning.executive.i_action_bridge import IActionBridge
from roboter_task_planning.executive.world_knowledge import Mission, WorldKnowledge

ERFOLG, FEHLGESCHLAGEN, ABGEBROCHEN = "erfolg", "fehlgeschlagen", "abgebrochen"


@dataclass
class MissionResult:
    status: str
    meldung: str = ""
    ausgefuehrt: List[str] = field(default_factory=list)
    replans: int = 0


class MissionExecutor:
    def __init__(self, planner: IPlanner, bridges: List[IActionBridge],
                 locator: Optional[Callable[[], Optional[tuple]]] = None,
                 log: Callable[[str], None] = print,
                 status_cb: Optional[Callable[[dict], None]] = None,
                 domain: Optional[str] = None,
                 wegschaetzer: Optional[Callable[[tuple, tuple], Optional[float]]] = None):
        self.planner = planner
        self.by_action: Dict[str, IActionBridge] = {}
        for b in bridges:
            for a in b.aktionen:
                self.by_action[a] = b
        self.locator = locator or (lambda: None)
        self.log = log
        self.status_cb = status_cb or (lambda s: None)
        self.domain = domain
        self.wegschaetzer = wegschaetzer

    def _status(self, **kw):
        self.status_cb(kw)

    def run(self, mission: Mission, abort: threading.Event) -> MissionResult:
        """Plant und fuehrt die Mission aus. Ablauf: planen -> Schritte ausfuehren -> bei einem
        Fehlschlag Position bestimmen, dynamischen Ort einfuegen, neu planen (hoechstens
        max_replans-mal). Abbruch (NotAus/Service) beendet ohne Neuplanung. Jede unerwartete
        Ausnahme beendet die Mission mit Status 'fehlgeschlagen' (Review B3: vorher starb der
        Thread stumm, ohne Endstatus auf /mission/status)."""
        res = MissionResult(FEHLGESCHLAGEN)
        try:
            return self._run(mission, abort, res)
        except Exception as e:      # bewusst breit: der Missions-Thread darf nie stumm sterben
            return self._finish(res, FEHLGESCHLAGEN, f"interner Fehler: {type(e).__name__}: {e}")

    def _run(self, mission: Mission, abort: threading.Event, res: MissionResult) -> MissionResult:
        pos = self.locator()
        start = mission.start_ort
        if start is None:
            if not pos:
                # Review L5: vorher wurde stillschweigend der erste Ort angenommen
                return self._finish(res, FEHLGESCHLAGEN,
                                    "Startort unbekannt: weder start_ort in der Mission noch eine Position")
            start = WorldKnowledge.naechster_ort(mission.orte, pos)
        world = WorldKnowledge(mission, start, self.wegschaetzer)
        domain = self.domain or read_domain()
        max_replans = int(mission.param("max_replans"))
        self.log(f"Mission startet bei '{start}', Ziele: {mission.ziele}")

        while True:
            if abort.is_set():
                return self._finish(res, ABGEBROCHEN, "abgebrochen vor Planung")
            if not world.offene_ziele():
                return self._finish(res, ERFOLG, "alle Ziele erreicht")
            try:
                plan = self.planner.plan(domain, generate_problem(world))
            except PlanningFailed as e:
                return self._finish(res, FEHLGESCHLAGEN, f"Planung fehlgeschlagen: {e}")
            self.log(f"Plan ({plan.kosten}): " + " -> ".join(map(str, plan.schritte)))
            self._status(phase="plan", plan=[str(s) for s in plan.schritte], replans=res.replans)

            fehler = None
            fehl_ziel = None
            for i, s in enumerate(plan.schritte):
                if abort.is_set():
                    return self._finish(res, ABGEBROCHEN, f"abgebrochen vor {s}")
                bridge = self.by_action.get(s.name)
                if bridge is None:
                    return self._finish(res, FEHLGESCHLAGEN, f"keine Bridge fuer Aktion '{s.name}'")
                self._status(phase="schritt", schritt=str(s), index=i, von=len(plan.schritte))
                self.log(f"[{i + 1}/{len(plan.schritte)}] {s}")
                r = bridge.execute(s, mission, abort, orte=dict(world.orte))
                if abort.is_set():
                    return self._finish(res, ABGEBROCHEN, f"abgebrochen waehrend {s}")
                if r.erfolg:
                    world.schritt_erledigt(s.name, s.args)
                    res.ausgefuehrt.append(str(s))
                else:
                    fehler = f"{s} fehlgeschlagen: {r.meldung}"
                    fehl_ziel = s.args[-1] if s.args else None
                    if not r.wiederholbar:
                        return self._finish(res, FEHLGESCHLAGEN, f"{fehler} (nicht wiederholbar)")
                    break

            if fehler is None and not world.offene_ziele():
                continue            # Schleifenkopf meldet Erfolg
            res.replans += 1
            if res.replans > max_replans:
                return self._finish(res, FEHLGESCHLAGEN,
                                    f"{fehler or 'Ziele nach Plan offen'} - max. Neuplanungen erreicht")
            self.log(f"Neuplanung {res.replans}/{max_replans}: {fehler}")
            p = self.locator()
            if p:
                world.aktualisiere_position(p, gescheitertes_ziel=fehl_ziel)
            else:
                self.log("Position unbekannt - Neuplanung vom letzten bekannten Ort "
                         f"'{world.current_ort}' (kann ungenau sein)")

    def _finish(self, res: MissionResult, status: str, meldung: str) -> MissionResult:
        res.status, res.meldung = status, meldung
        self.log(f"Mission {status}: {meldung}")
        self._status(phase="ende", status=status, meldung=meldung, replans=res.replans)
        return res
