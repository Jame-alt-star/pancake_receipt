import numpy as np

from roboter_steuerung.states.i_robot_state import IRobotState
from std_msgs.msg import Bool


class BalancierenState(IRobotState):
    """
    Zustand "Balancieren": der Roboter soll auf der Stelle stehen
    bleiben/das Gleichgewicht halten, OHNE sich fortzubewegen.

    VERLAUF (11.-12.10.2026, siehe Chat-Historie fuer Details): bisher
    drei Varianten ausprobiert, alle verworfen:
    1. Lauf-Policy mit erzwungenem Null-Kommando - driftete seitlich weg,
       fiel aber NIE um.
    2. Starre Ruhepose per PD-Regelung, SOFORT angesprungen - kippte um.
    3. Sanft interpolierte Trajektorie zur Ruhepose + staerkere Stand-
       Gains (kps_stand/kds_stand) - kippte TROTZDEM um (nach vorne).

    Per Rechnung (nicht Simulation) wurde geprueft, ob default_angles
    ueberhaupt eine geometrisch stabile Standpose ist (Schwerpunkt vs.
    konvexe Huelle der Fuss-Eckpunkte, mit expliziten Masse-/Schwerpunkt-
    Werten aus g1_12dof.xml): JA, klar stabil (~7,4cm Puffer nach vorne
    UND hinten). Die Pose ist also NICHT die Ursache.

    SCHLUSSFOLGERUNG: reine Gelenk-Positions-PD-Regelung (Varianten 2+3)
    ist blind fuer Schwerpunkt/Schwung - sie regelt nur "Gelenkwinkel
    soll=ist", nicht "Schwerpunkt ueber der Standflaeche halten". Beim
    Wechsel aus dem Gehen ist vermutlich noch Vorwaerts-Schwung im
    System, den reine PD-Haltung nicht ausgleichen kann.

    Zusaetzliche Recherche zum Stand der Technik (12.10.2026): Unitrees
    eigene Deployment-FSM (unitree_rl_lab) wechselt beim Stoppen NICHT in
    einen separaten statischen Halte-Zustand - der "Velocity"-RL-Zustand
    bleibt aktiv, auch bei Kommando=0. Das passt zur ueblichen Trainings-
    konvention (zero_cmd_prob, z.B. in legged_gym/unitree_rl_gym): die
    Policy wird explizit so trainiert, dass sie bei Kommando=0 stehen
    bleibt - das ist kein Edge-Case, sondern Teil ihres Trainingsbereichs.

    DIESE VERSION (Variante 4, 12.10.2026) setzt genau das um: statt PD-
    Haltung/Interpolation (Varianten 2+3) bleibt die RL-Policy aktiv, mit
    cmd auf ~0 erzwungen (= Variante 1, die als einzige bisher NIE
    umgefallen ist). Normale (nicht Stand-)PD-Gains, weil die Policy
    selbst die Stabilisierung uebernimmt - die Gains muessen nur noch die
    von der Policy vorgegebenen Zielwinkel verfolgen, wie beim Gehen auch.

    ERGAENZUNG (12.10.2026, Heading-/Positions-Haltung): Variante 1/4
    zeigte zwei Arten von Drift (Drehung UND reine seitliche Verschiebung
    ohne Drehung, siehe steuereinheit_node.py fuer die Einordnung). Statt
    cmd hier selbst zu konstruieren, wird jetzt EINFACH node.cmd
    durchgereicht (genau wie in gehen_state.py) - node.cmd enthaelt zu
    diesem Zeitpunkt bereits die in steuereinheit_node.py berechneten
    Korrekturen (_apply_heading_hold()/_apply_position_hold()), die vx/vy/
    yaw_rate nur so weit von 0 wegbewegen, wie es braucht, um Ausrichtung/
    Position zu halten. RISIKO (siehe Docstring von
    _apply_position_hold()): das ist nicht mehr exakt dieselbe Eingabe wie
    bei der urspruenglichen, nachweislich stabilen Variante 1 (dort war
    cmd hart [0,0,0]) - deshalb als zu beobachtender Testpunkt markiert,
    nicht als automatisch gesicherte Verbesserung.

    PBalanceController (Ankle-Strategy) bleibt als eigenstaendige
    Komponente im Code erhalten (siehe control/p_balance_controller.py) -
    falls spaeter eine aktive Stoerungsausgleichsregelung fuer echte
    externe Stoesse gebraucht wird, waere das der naechste Ausbauschritt
    (zusaetzlich zur Policy, nicht als Ersatz).

    Wechselt zurueck nach GehenState, sobald ein von Null verschiedenes
    /cmd_vel-Kommando ankommt.
    """

    CMD_NULL_SCHWELLE = 0.05

    def execute(self, node) -> IRobotState:
        # Policy laeuft normal weiter -> normale (Lauf-)Gains, keine
        # Stand-Gains mehr noetig (siehe Docstring).
        node.pub_balance_mode.publish(Bool(data=False))

        quat, omega = node.bridge.read_imu()
        joint_pos, joint_vel = node.bridge.read_joint_states()

        # ZURUECKGEBAUT (12.10.2026): node.cmd direkt durchreichen (inkl.
        # Positions-Haltung auf vx/vy) wurde getestet, fuehrte aber zum
        # Umfallen - vx/vy daher wieder hart auf 0, nur die Drehrate
        # (node.cmd[2], Heading-Hold) bleibt durchgereicht, da die beim
        # Testen NICHT zum Umfallen gefuehrt hat. Siehe Docstring oben.
        # vx/vy: nur die Positionshaltungs-Korrektur (0, solange diese aus ist);
        # die Rampen-Restwerte vom Gehen bleiben bewusst draussen (Variante 4).
        cmd_fuer_policy = np.array(
            [float(node.pos_hold_corr[0]), float(node.pos_hold_corr[1]), float(node.cmd[2])],
            dtype=np.float32,
        )
        target_positions = node.controller.compute_joint_targets(
            joint_positions=joint_pos,
            joint_velocities=joint_vel,
            base_quat=quat,
            base_ang_vel=omega,
            cmd=cmd_fuer_policy,
        )
        node.bridge.send_joint_commands(target_positions)

        # NEU (12.10.2026): Pruefung bewusst gegen node.cmd_target, nicht
        # node.cmd - siehe Begruendung in gehen_state.py/
        # steuereinheit_node.py. Sonst koennten die Haltungs-Korrekturen
        # faelschlich einen Wechsel zurueck nach Gehen ausloesen.
        if max(abs(float(node.cmd_target[0])), abs(float(node.cmd_target[1])), abs(float(node.cmd_target[2]))) >= self.CMD_NULL_SCHWELLE:
            # Lazy Import, siehe Begruendung in gehen_state.py.
            from roboter_steuerung.states.gehen_state import GehenState
            node.get_logger().info("Zustandswechsel: Balancieren -> Gehen (Kommando != 0).")
            return GehenState()
        return self
