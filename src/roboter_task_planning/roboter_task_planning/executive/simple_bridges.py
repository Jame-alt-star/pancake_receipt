"""ROS-freie Bridges."""
import threading
import time

from roboter_task_planning.deliberative.i_planner import PlanSchritt
from roboter_task_planning.executive.i_action_bridge import BridgeResult, IActionBridge
from roboter_task_planning.executive.world_knowledge import Mission


class WarteBridge(IActionBridge):
    """'warte(ort)': feste Zeit warten, abbrechbar."""
    aktionen = ("warte",)

    def execute(self, schritt: PlanSchritt, mission: Mission, cancel: threading.Event) -> BridgeResult:
        dauer = float(mission.param("warte_s"))
        ende = time.monotonic() + dauer
        while time.monotonic() < ende:
            if cancel.wait(0.05):
                return BridgeResult(False, "abgebrochen")
        return BridgeResult(True, f"{dauer:.1f} s gewartet")
