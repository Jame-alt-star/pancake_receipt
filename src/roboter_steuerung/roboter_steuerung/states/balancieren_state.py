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

    STAND 14.10.2026 (Eingabe an die Policy): vx/vy = abklingende Rampe (weiches Abbremsen nach
    dem Gehen) + Positionshaltungs-Korrektur (node.pos_hold_corr, greift erst wenn die Rampe
    abgeklungen ist, nur mit position_hold: true); yaw_rate = node.cmd[2] (Rampe + Kurshaltung).
    Gemessen (29-DOF): mit Kurshaltung < 1 cm Drift in 120 s, Beine praktisch still (0.0004 rad/s);
    die Positionshaltung hilft gegen Stoesse (0.6 m/s seitlich: 6 cm statt 24 cm Endabweichung).

    PBalanceController (Ankle-Strategy) bleibt als eigenstaendige
    Komponente im Code erhalten (siehe control/p_balance_controller.py) -
    falls spaeter eine aktive Stoerungsausgleichsregelung fuer echte
    externe Stoesse gebraucht wird, waere das der naechste Ausbauschritt
    (zusaetzlich zur Policy, nicht als Ersatz).

    Wechselt nach GehenState, sobald ein von Null verschiedenes /cmd_vel-Kommando ansteht.
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
        # GEAENDERT (14.10.2026, Review L1/U2): vx/vy = abklingende Rampe (weiches Abbremsen
        # nach dem Gehen, wie in _ramp_cmd dokumentiert) + Positionshaltungs-Korrektur (die erst
        # greift, wenn die Rampe abgeklungen ist). Vorher wurde vx/vy beim Wechsel aus Gehen
        # hart auf 0 gesetzt; die Policy vertrug das, die Doku beschrieb aber das Abbremsen.
        cmd_fuer_policy = np.array(
            [float(node.cmd_ramped[0] + node.pos_hold_corr[0]),
             float(node.cmd_ramped[1] + node.pos_hold_corr[1]),
             float(node.cmd[2])],
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
