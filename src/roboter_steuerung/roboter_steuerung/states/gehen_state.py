from roboter_steuerung.states.i_robot_state import IRobotState


class GehenState(IRobotState):
    """
    Zustand "Gehen": der Roboter soll sich aktiv gemaess dem per
    /cmd_vel gesetzten Kommando (node.cmd, siehe steuereinheit_node.py)
    fortbewegen. Nutzt dafuer direkt den vorhandenen RL-Locomotion-
    Controller (node.controller) - das ist bisher der einzige Regler,
    den es gibt, unabhaengig vom Zustand (siehe BalancierenState fuer
    denselben Fallback mit erzwungenem Null-Kommando).

    Wechselt nach BalancierenState, sobald das Kommando (naeherungsweise)
    null ist - der Nutzer/eine uebergeordnete Instanz (Joystick, Nav2,
    "ros2 topic pub") hat dann signalisiert, dass sich der Roboter nicht
    mehr fortbewegen soll.
    """

    # Schwellwert, unterhalb dessen ein Kommando als "praktisch null"
    # gilt (statt exakt 0.0 zu verlangen - ein Joystick/Nav2 liefert
    # praktisch nie exakte Nullen). Bewusst klein gewaehlt, noch nicht
    # gegen echte Nutzung getestet - ggf. anzupassen.
    CMD_NULL_SCHWELLE = 0.05

    def execute(self, node) -> IRobotState:
        # NEU (12.10.2026): jeden Takt False melden, damit mujoco_node.py
        # beim Gehen die normalen (fuer die Policy abgestimmten) Gains
        # verwendet - siehe balancieren_state.py fuer das Gegenstueck.

        quat, omega = node.bridge.read_imu()
        joint_pos, joint_vel = node.bridge.read_joint_states()

        target_positions = node.controller.compute_joint_targets(
            joint_positions=joint_pos,
            joint_velocities=joint_vel,
            base_quat=quat,
            base_ang_vel=omega,
            cmd=node.cmd
        )
        node.bridge.send_joint_commands(target_positions)

        # NEU (12.10.2026): Pruefung bewusst gegen node.cmd_target (das
        # ANGEFORDERTE Kommando), nicht gegen node.cmd (der tatsaechlich
        # an die Policy gegebene, u.a. um die Heading-Hold-Korrektur
        # ergaenzte Wert) - siehe Begruendung im Docstring von
        # steuereinheit_node.py. Sonst koennte eine kleine
        # Kurshaltungs-Korrektur auf cmd[2] faelschlich wie ein
        # Bewegungswunsch aussehen.
        if max(abs(float(node.cmd_target[0])), abs(float(node.cmd_target[1])), abs(float(node.cmd_target[2]))) < self.CMD_NULL_SCHWELLE:
            # Lazy Import (erst hier, nicht am Dateianfang), um einen
            # Ringimport mit balancieren_state.py zu vermeiden - die
            # Zustandsklasse wird erst gebraucht, wenn tatsaechlich
            # gewechselt wird.
            from roboter_steuerung.states.balancieren_state import BalancierenState
            node.get_logger().info("Zustandswechsel: Gehen -> Balancieren (Kommando ~0).")
            return BalancierenState()
        return self
