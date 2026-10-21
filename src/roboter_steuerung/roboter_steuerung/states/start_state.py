from roboter_steuerung.states.i_robot_state import IRobotState


class StartState(IRobotState):
    """Start: noch keine gueltigen Sensordaten. Sendet nichts (die Policy kann ohne Messwerte nicht sinnvoll
    rechnen) und wechselt mit den ersten Daten nach Bereitschaft. Das Start-Gate im Knoten schuetzt zusaetzlich
    alle Zustaende vor Nullwerten."""

    def execute(self, node) -> IRobotState:
        if not node.sensordaten_gueltig():
            return self
        from roboter_steuerung.states.bereitschaft_state import BereitschaftState
        node.get_logger().info("Zustandswechsel: Start -> Bereitschaft (erste gueltige Sensordaten).")
        return BereitschaftState()
