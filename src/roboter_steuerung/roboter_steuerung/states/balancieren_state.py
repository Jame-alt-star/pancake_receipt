from roboter_steuerung.states.i_robot_state import IRobotState


class BalancierenState(IRobotState):
    """Balancieren = auf der Stelle stehen und das Gleichgewicht halten. Das Gleichgewicht haelt die Policy selbst
    (sie ist auf Befehl 0 trainiert; auch Unitrees eigene Steuerung bleibt beim Stehen im Lauf-Zustand). Dieser
    Zustand unterscheidet sich von Gehen nur im Befehl: die Rampe klingt weich ab, danach haelt die Positionshaltung
    den Standort (gemessen: < 1 cm Drift in 120 s; seitlicher Stoss 0.6 m/s -> 6 statt 24 cm Endabweichung).

    Warum keine eigene Stand-Regelung: starre PD-Haltung der Ruhepose - sofort oder interpoliert, auch mit hoeheren
    Verstaerkungen - kippte jedes Mal, obwohl die Pose statisch stabil ist; Gelenk-PD ist blind fuer Schwerpunkt und
    Schwung (12.10.2026). Wechselt nach Gehen, sobald ein Bewegungswunsch ansteht."""

    def execute(self, node) -> IRobotState:
        self._policy_schritt(node, node.befehl_stehen(mit_positionshaltung=True))
        if not node.bewegung_gewuenscht():
            return self
        from roboter_steuerung.states.gehen_state import GehenState
        node.get_logger().info("Zustandswechsel: Balancieren -> Gehen (Kommando != 0).")
        return GehenState()
