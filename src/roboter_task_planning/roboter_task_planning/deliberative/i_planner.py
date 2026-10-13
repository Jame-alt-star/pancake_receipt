"""Schnittstelle des Planers (Layer 1, deliberative)."""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List


class PlanningFailed(Exception):
    """Kein Plan gefunden (unloesbar, Planer-Fehler oder Zeitueberschreitung)."""


@dataclass
class PlanSchritt:
    name: str                     # z.B. "gehe_zu"
    args: List[str] = field(default_factory=list)   # z.B. ["start", "a"]

    def __str__(self):
        return f"{self.name}({', '.join(self.args)})"


@dataclass
class Plan:
    schritte: List[PlanSchritt]
    kosten: int = 0


class IPlanner(ABC):
    @abstractmethod
    def plan(self, domain_pddl: str, problem_pddl: str) -> Plan:
        """Liefert einen Plan oder wirft PlanningFailed."""
