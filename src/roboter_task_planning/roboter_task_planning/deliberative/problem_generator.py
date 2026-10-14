"""Erzeugt aus Weltwissen + Mission ein PDDL-Problem (Layer 1)."""
import os

from roboter_task_planning.executive.world_knowledge import DYNAMISCHER_ORT, WorldKnowledge


def domain_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory("roboter_task_planning"), "pddl", "domain.pddl")
        if os.path.exists(p):
            return p
    except Exception:
        pass
    return os.path.join(os.path.dirname(__file__), "..", "..", "pddl", "domain.pddl")


def read_domain() -> str:
    with open(domain_path(), "r") as f:
        return f.read()


def generate_problem(world: WorldKnowledge) -> str:
    orte = sorted(world.orte)
    lines = ["(define (problem mission)", "  (:domain roboter-mission)",
             f"  (:objects {' '.join(orte)} - ort)", "  (:init", "    (= (total-cost) 0)",
             f"    (at {world.current_ort})"]
    for art, ort in sorted(world.fakten):
        lines.append(f"    ({art} {ort})")
    for v in sorted(tuple(sorted(p)) for p in world.verbindungen):
        a, b = v
        lines.append(f"    (verbunden {a} {b})")
        lines.append(f"    (verbunden {b} {a})")
        c = world.strecke_cm(a, b)
        lines.append(f"    (= (strecke {a} {b}) {c})")
        lines.append(f"    (= (strecke {b} {a}) {c})")
    for ziel in sorted(world.dyn_ziele):             # nur ausgehend vom dynamischen Ort
        lines.append(f"    (verbunden {DYNAMISCHER_ORT} {ziel})")
        lines.append(f"    (= (strecke {DYNAMISCHER_ORT} {ziel}) {world.strecke_cm(DYNAMISCHER_ORT, ziel)})")
    lines.append("  )")
    # am_ort (Endposition) entspricht dem Praedikat (at ort)
    goals = [f"(at {ort})" if art == "am_ort" else f"({art} {ort})" for art, ort in world.mission.ziele]
    lines.append(f"  (:goal (and {' '.join(goals)}))")
    lines.append("  (:metric minimize (total-cost)))")
    return "\n".join(lines) + "\n"
