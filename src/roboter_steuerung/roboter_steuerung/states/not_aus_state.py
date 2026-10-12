import numpy as np

from roboter_steuerung.states.i_robot_state import IRobotState
from std_msgs.msg import Bool


class NotAusState(IRobotState):
    """
    Zustand "NotAus" (SysML: RoboterZustandsautomat, siehe
    RoboterZustaende.sysml). Ausgeloest/aufgehoben zentral in
    steuereinheit_node._control_loop() ueber das /not_aus-Topic (nicht
    ueber den normalen execute()-Rueckgabewert wie bei Gehen/
    Balancieren/Bereitschaft), WEIL Not-Aus jeden anderen Zustand
    unterbrechen kann/muss - nicht nur einen bestimmten.

    RECHERCHE ZUR UEBLICHEN PRAXIS (12.10.2026, siehe Chat-Historie): es
    gibt kein einzelnes, ROS(2)-weit standardisiertes Not-Aus-Topic/
    Service (anders als z.B. /cmd_vel). Auf echter Hardware wird Not-Aus
    ueblicherweise HARDWARE-seitig (Sicherheitsrelais/-schaltkreis)
    verdrahtet und kappt die Motorversorgung UNABHAENGIG von der
    Software. Da hier keine solche physische Verdrahtung existiert
    (MuJoCo-Simulation), wird Not-Aus hier rein softwareseitig ausgeloest
    - siehe steuereinheit_node.py fuer das /not_aus-Topic (latched/
    TRANSIENT_LOCAL).

    VERLAUF (12.10.2026, siehe Chat-Historie) - DREI Iterationen,
    empirisch durchgetestet:
    1. Hartes Einfrieren der letzten ctrl-Werte, SOFORT bei Ausloesung.
       Im Test beim Gehen ausgeloest: Roboter fiel nach vorne.
    2. Policy dauerhaft aktiv mit cmd hart auf [0,0,0] (wie
       BalancierenState/"Variante 4"). Im Test: faellt NICHT mehr um -
       aber die Policy "trabt" dabei unbegrenzt weiter auf der Stelle
       (dieselbe Links-Rechts-Trittbewegung wie beim normalen Stehen).
    3. Zweiphasiger Versuch: wie 2., aber nach einer festen Abbrems-
       Zeit (erst 1,5s, dann auf 6,0s erhoeht) zu hartem Einfrieren
       gewechselt, in der Annahme, der Restschwung sei danach abgebaut.
       Im Test: faellt bei BEIDEN Zeitwerten (1,5s UND 6,0s) beim
       Einfrieren um.

    SCHLUSSFOLGERUNG aus Iteration 3 (wichtig): die Trittbewegung bei
    cmd=0 ist KEIN abklingender Restschwung, der nach einer Wartezeit
    verschwindet - es ist eine fortlaufende, zyklische Bewegung
    (Trainingsartefakt der Policy: die Gangphase laeuft auch bei
    Kommando=0 immer weiter, siehe gleiche Beobachtung beim normalen
    Stehen). Es gibt dadurch GAR KEINEN sicheren Moment zum Einfrieren -
    egal wie lange man wartet, es ist (fast) immer Gewicht einseitig
    verteilt/ein Fuss gerade im Abheben. Hartes Einfrieren ist mit
    dieser Policy daher grundsaetzlich nicht sicher moeglich, nicht nur
    eine Frage der richtigen Wartezeit.

    FIX (DIESE VERSION, nach Iteration 3): Phase 2 (hartes Einfrieren)
    ERSATZLOS GESTRICHEN. NotAusState bleibt jetzt dauerhaft in der
    nachweislich stabilen Variante aus Iteration 2 (Policy aktiv, cmd
    hart auf [0,0,0], keine Heading-Hold-Korrektur fuer maximale
    Vorhersagbarkeit). KONSEQUENZ: der Roboter steht bei NotAus nicht
    woertlich bewegungslos still, sondern trabt/trittet weiter leicht
    auf der Stelle, solange NotAus aktiv ist - das eigentliche Ziel
    (nicht umfallen/beschaedigt werden, siehe Nutzervorgabe) ist damit
    aber erfuellt, "woertlich einfrieren" war nur ein Mittel dazu und
    erwies sich als nicht sicher erreichbar.

    WICHTIGER KOMPROMISS (bewusst in Kauf genommen): das ist kein
    "Motoren sofort aus" mehr im wortwoertlichen Sinn, sondern ein aktiv
    geregeltes Halten der Balance - bei einem dynamisch balancierenden
    Zweibeiner ohne passive Standstabilitaet ist ein echtes Sofort-Aus
    ohnehin gleichbedeutend mit einem Sturz.

    OFFENER PUNKT (bewusst nicht umgesetzt): nach Entwarnung (/not_aus
    wieder False) wechselt der Automat automatisch zurueck nach
    BereitschaftState - es gibt KEINEN bewussten Freigabe-/Reset-
    Schritt. Siehe auch BereitschaftState.
    """

    def execute(self, node) -> IRobotState:
        # Policy bleibt dauerhaft aktiv, cmd hart auf [0,0,0] - siehe
        # Docstring/FIX oben. Kein Zeit-/Phasenwechsel mehr, nachdem
        # sich das in zwei Testlaeufen (1,5s und 6,0s) als nicht sicher
        # herausgestellt hat.
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

        # Kein Zustandswechsel ueber den Rueckgabewert - der Ausstieg aus
        # NotAusState passiert ausschliesslich zentral in
        # steuereinheit_node._control_loop(), siehe dort.
        return self
