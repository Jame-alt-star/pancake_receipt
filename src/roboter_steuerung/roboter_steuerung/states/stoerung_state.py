from roboter_steuerung.states.i_robot_state import IRobotState


class StoerungState(IRobotState):
    """Stoerung: die Steuerung kann nicht mehr sinnvoll regeln - Sensordaten bleiben aus, die Policy liefert ungueltige
    Werte (NaN/inf) oder der Roboter ist gestuerzt. Dann sendet sie KEINE Zielwinkel mehr: mit alten Messwerten oder
    im Liegen weiterzuregeln ist gefaehrlicher als aufzuhoeren. Layer 4 erkennt das ausbleibende Kommando und geht in
    den Daempfungsmodus (Simulation: mujoco_node kommando_timeout_s; Hardware: Unitree-Firmware).

    Kein automatisches Zurueck: erst der Dienst /steuerung/quittieren (Sensordaten wieder da, aufrecht, kein NotAus)
    fuehrt nach Bereitschaft, mit neuer Freigabe-Pflicht. Nicht Teil des urspruenglichen SysML-Automaten (Ergaenzung
    21.10.2026)."""

    def __init__(self, grund: str):
        self.grund = grund

    def execute(self, node) -> IRobotState:
        return self
