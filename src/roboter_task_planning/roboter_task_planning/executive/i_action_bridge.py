"""Schnittstelle zwischen Dispatcher (Layer 1) und ausfuehrender Schicht (Layer 2/3)."""
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass

from roboter_task_planning.deliberative.i_planner import PlanSchritt
from roboter_task_planning.executive.world_knowledge import Mission


@dataclass
class BridgeResult:
    erfolg: bool
    meldung: str = ""


class IActionBridge(ABC):
    """Fuehrt einen Planschritt blockierend aus. Muss `cancel` zeitnah beachten."""

    #: PDDL-Aktionsnamen, die diese Bridge ausfuehrt
    aktionen: tuple = ()

    @abstractmethod
    def execute(self, schritt: PlanSchritt, mission: Mission, cancel: threading.Event) -> BridgeResult:
        ...
