"""Fast-Downward-Anbindung (Layer 1)."""
import os
import re
import shutil
import subprocess
import tempfile

from roboter_task_planning.deliberative.i_planner import IPlanner, Plan, PlanSchritt, PlanningFailed

DEFAULT_SEARCH = "astar(lmcut())"
_ACTION = re.compile(r"^\(([a-z0-9_\-]+)((?:\s+[a-z0-9_\-]+)*)\)\s*$")
_COST = re.compile(r";\s*cost\s*=\s*(\d+)")


def find_fast_downward() -> str:
    cands = [os.environ.get("FAST_DOWNWARD"),
             os.path.expanduser("~/tools/downward/fast-downward.py"),
             shutil.which("fast-downward.py"), shutil.which("fast-downward")]
    for c in cands:
        if c and os.path.exists(c):
            return c
    raise PlanningFailed(
        "Fast Downward nicht gefunden (Umgebungsvariable FAST_DOWNWARD oder ~/tools/downward). "
        "Siehe roboter_task_planning/README.md")


def parse_plan(text: str) -> Plan:
    schritte, kosten = [], 0
    for line in text.splitlines():
        m = _COST.match(line.strip())
        if m:
            kosten = int(m.group(1))
            continue
        m = _ACTION.match(line.strip())
        if m:
            schritte.append(PlanSchritt(m.group(1), m.group(2).split()))
    return Plan(schritte, kosten)


class FastDownwardPlanner(IPlanner):
    def __init__(self, search: str = DEFAULT_SEARCH, timeout_s: float = 30.0, fd_path: str = None):
        self.search = search
        self.timeout_s = timeout_s
        self.fd_path = fd_path

    def plan(self, domain_pddl: str, problem_pddl: str) -> Plan:
        fd = self.fd_path or find_fast_downward()
        with tempfile.TemporaryDirectory(prefix="fd_") as tmp:
            dpath, ppath = os.path.join(tmp, "domain.pddl"), os.path.join(tmp, "problem.pddl")
            with open(dpath, "w") as f:
                f.write(domain_pddl)
            with open(ppath, "w") as f:
                f.write(problem_pddl)
            # fast-downward.py mit python3 starten; ein Wrapper aus PATH (fast-downward) direkt (Review B8)
            aufruf = ["python3", fd] if fd.endswith(".py") else [fd]
            try:
                r = subprocess.run(aufruf + ["--plan-file", os.path.join(tmp, "sas_plan"),
                                    "--sas-file", os.path.join(tmp, "output.sas"),
                                    dpath, ppath, "--search", self.search],
                                   cwd=tmp, capture_output=True, text=True, timeout=self.timeout_s)
            except subprocess.TimeoutExpired:
                raise PlanningFailed(f"Planer-Zeitueberschreitung nach {self.timeout_s} s")
            plan_file = os.path.join(tmp, "sas_plan")
            if r.returncode != 0 or not os.path.exists(plan_file):
                raise PlanningFailed(f"kein Plan (Fast Downward Exit {r.returncode}): "
                                     f"{(r.stdout + r.stderr).strip().splitlines()[-1:]}")
            with open(plan_file) as f:
                return parse_plan(f.read())
