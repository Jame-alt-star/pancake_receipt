import numpy as np

from roboter_steuerung.states.i_robot_state import IRobotState


class StartState(IRobotState):
    """
    Zustand "Start": initialer Zustand direkt nach dem Hochfahren dieses
    Knotens (SysML: RoboterZustandsautomat, Zustand "Start", siehe
    RoboterZustaende.sysml). Es sind noch keine gueltigen Sensordaten
    von mujoco_node.py angekommen - erkennbar daran, dass sowohl die
    Gelenkwinkel als auch die Quaternion noch auf ihrem Initialwert
    (ueberall 0.0) stehen (eine echte Quaternion ist nie komplett 0,
    siehe last_quat-Default in mujoco_bridge.py).

    Sendet bewusst NICHTS an die Bridge, solange das der Fall ist -
    entspricht dem fruehen "Start-Gate" in
    steuereinheit_node._control_loop() (das bleibt dort als zentraler
    Schutz fuer ALLE Zustaende bestehen, siehe Kommentar dort), macht
    diesen Wartezustand aber jetzt auch im Zustandsautomaten selbst
    sichtbar (self.state zeigt waehrend der Wartezeit korrekt auf
    StartState), statt nur ein stiller fruehzeitiger return zu sein.

    Wechselt nach BereitschaftState, sobald die erste gueltige Messung
    da ist.
    """

    def execute(self, node) -> IRobotState:
        quat, _ = node.bridge.read_imu()
        joint_pos, _ = node.bridge.read_joint_states()
        if np.all(joint_pos == 0.0) and np.all(quat == 0.0):
            # Noch keine gueltigen Daten - im Start-Zustand bleiben.
            return self

        # Lazy Import, siehe Begruendung in gehen_state.py (Ringimport
        # vermeiden).
        from roboter_steuerung.states.bereitschaft_state import BereitschaftState
        node.get_logger().info("Zustandswechsel: Start -> Bereitschaft (erste gueltige Sensordaten).")
        return BereitschaftState()
