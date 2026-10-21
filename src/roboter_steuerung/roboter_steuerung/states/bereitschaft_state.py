from roboter_steuerung.states.i_robot_state import IRobotState


class BereitschaftState(IRobotState):
    """Bereitschaft: Roboter steht und wartet auf die bewusste Freigabe (/freigabe), statt nach dem Hochfahren mit
    einem anstehenden Befehl loszulaufen.

    Warum die Policy dabei aktiv bleibt (Fahrbefehl 0, Kurshaltung auf der Drehrate): ohne /joint_commands haelt
    Layer 4 die Ruhepose nur per PD-Regelung, und reine Gelenk-PD merkt nicht, wenn der Schwerpunkt wandert - der
    Roboter fiel sofort um (12.10.2026). Mit Freigabe: Befehl ~0 -> Balancieren, sonst -> Gehen."""

    def __init__(self):
        self._warte_gemeldet = False

    def execute(self, node) -> IRobotState:
        if not node.bewegung_freigegeben:
            if not self._warte_gemeldet:
                node.get_logger().info("Zustand: Bereitschaft - warte auf Freigabe (/freigabe).")
                self._warte_gemeldet = True
            self._policy_schritt(node, node.befehl_stehen(mit_positionshaltung=False))
            return self
        node.get_logger().info("Zustand: Bereitschaft - Freigabe erhalten.")
        if node.bewegung_gewuenscht():
            from roboter_steuerung.states.gehen_state import GehenState
            node.get_logger().info("Zustandswechsel: Bereitschaft -> Gehen.")
            return GehenState()
        from roboter_steuerung.states.balancieren_state import BalancierenState
        node.get_logger().info("Zustandswechsel: Bereitschaft -> Balancieren.")
        return BalancierenState()
