from roboter_steuerung.states.i_robot_state import IRobotState


class GehenState(IRobotState):
    """Gehen: die Policy setzt den aufbereiteten Fahrbefehl um (Rampe, Kurshaltung, Begrenzung). Wechselt nach
    Balancieren, sobald kein Bewegungswunsch mehr ansteht (geprueft gegen den Wunsch, nicht gegen den Befehl an die
    Policy - sonst hielten die Haltungskorrekturen den Roboter im Zustand Gehen)."""

    def execute(self, node) -> IRobotState:
        self._policy_schritt(node, node.befehl_gehen())
        if node.bewegung_gewuenscht():
            return self
        from roboter_steuerung.states.balancieren_state import BalancierenState
        node.get_logger().info("Zustandswechsel: Gehen -> Balancieren (Kommando ~0).")
        return BalancierenState()
