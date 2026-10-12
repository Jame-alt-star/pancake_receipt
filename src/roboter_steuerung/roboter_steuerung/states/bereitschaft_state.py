import numpy as np

from roboter_steuerung.states.i_robot_state import IRobotState
from std_msgs.msg import Bool


class BereitschaftState(IRobotState):
    """
    Zustand "Bereitschaft" (SysML: RoboterZustandsautomat, siehe
    RoboterZustaende.sysml): Uebergang zwischen StartState und der
    aktiven Bewegungsregelung (Gehen/Balancieren) - der Roboter wartet
    hier auf eine bewusste Freigabe (node.bewegung_freigegeben, siehe
    steuereinheit_node._cmd_vel_callback()), bevor er in Gehen/
    Balancieren wechselt (siehe FIX 1 unten).

    FIX 1 (12.10.2026, Nutzerwunsch - Realismus): eine fruehere Version
    hat sofort automatisch anhand von node.cmd_target entschieden, OHNE
    auf einen bewussten Befehl zu warten - das fuehrte dazu, dass der
    Roboter direkt nach dem Hochfahren losgelaufen ist (node.cmd_target
    startet mit cmd_init aus der YAML, haeufig != 0). Nicht realistisch:
    ein Roboter sollte nach dem Hochfahren in Bereitschaft stehen
    bleiben, bis ein Bediener/eine uebergeordnete Instanz explizit ein
    Kommando sendet.

    FIX 2 (12.10.2026, direkt im Anschluss, Testfund): die ERSTE Version
    von FIX 1 hat waehrend des Wartens GAR KEINE Gelenk-Zielwerte
    gesendet (send_joint_commands() nicht aufgerufen) - in der Annahme,
    MuJoCo wuerde dadurch einfach die aktuelle (Ruhe-)Pose halten. Im
    Test: der Roboter ist SOFORT nach dem Laden umgefallen. Ursache
    gefunden in mujoco_node.py: ohne eine Nachricht auf /joint_commands
    regelt mujoco_node.py weiterhin per PD auf target_dof_pos =
    default_angles (dem YAML-Konfigurationswert) - das ist exakt
    "Variante 2" aus balancieren_state.py (reine Gelenk-Positions-PD-
    Regelung ohne die RL-Policy), bei der schon FRUEHER in diesem Projekt
    festgestellt wurde, dass sie umkippt - die Pose selbst ist zwar
    geometrisch stabil, aber eine reine PD-Haltung merkt nicht, wenn der
    Roboter kippt (hat keinen Rueckkopplungskreis auf den Schwerpunkt).

    DIESE VERSION (FIX 2): haelt waehrend des Wartens auf Freigabe die
    RL-Policy aktiv, mit cmd hart auf [0,0,0] - exakt dieselbe,
    nachweislich stabile Kombination wie BalancierenState/NotAusState
    ("Variante 4"). Der Roboter steht/trabt also schon WAEHREND der
    Bereitschaft stabil, nicht erst ab Gehen/Balancieren.
    """

    CMD_NULL_SCHWELLE = 0.05

    def __init__(self):
        # Nur fuer die einmalige Log-Meldung unten - verhindert, dass
        # "warte auf Freigabe" bei 50 Hz jeden Takt neu ins Log
        # geschrieben wird, solange keine Freigabe vorliegt.
        self._warte_gemeldet = False

    def execute(self, node) -> IRobotState:
        if not node.bewegung_freigegeben:
            if not self._warte_gemeldet:
                node.get_logger().info(
                    "Zustand: Bereitschaft - warte auf erstes /cmd_vel-Kommando (Freigabe)."
                )
                self._warte_gemeldet = True

            # FIX 2 (siehe Docstring): Policy aktiv halten, cmd hart auf
            # [0,0,0] - NICHT einfach nichts senden, das fuehrt sonst zu
            # reiner PD-Haltung in mujoco_node.py und damit zum Umfallen.
            node.pub_balance_mode.publish(Bool(data=False))
            quat, omega = node.bridge.read_imu()
            joint_pos, joint_vel = node.bridge.read_joint_states()
            cmd_fuer_policy = np.zeros(3, dtype=np.float32)
            target_positions = node.controller.compute_joint_targets(
                joint_positions=joint_pos,
                joint_velocities=joint_vel,
                base_quat=quat,
                base_ang_vel=omega,
                cmd=cmd_fuer_policy,
            )
            node.bridge.send_joint_commands(target_positions)
            return self

        node.get_logger().info("Zustand: Bereitschaft - Freigabe erhalten.")

        # Lazy Import, siehe Begruendung in gehen_state.py (Ringimport
        # vermeiden).
        from roboter_steuerung.states.gehen_state import GehenState
        from roboter_steuerung.states.balancieren_state import BalancierenState

        if max(abs(float(node.cmd_target[0])), abs(float(node.cmd_target[1])), abs(float(node.cmd_target[2]))) < self.CMD_NULL_SCHWELLE:
            node.get_logger().info("Zustandswechsel: Bereitschaft -> Balancieren.")
            return BalancierenState()
        node.get_logger().info("Zustandswechsel: Bereitschaft -> Gehen.")
        return GehenState()
