from abc import ABC, abstractmethod

import numpy as np


class IRobotState(ABC):
    """Zustand des Roboter-Zustandsautomaten (State Pattern, SysML RoboterZustandsautomat).

    Umgesetzt: Start, Bereitschaft, Gehen, Balancieren, NotAus. AkkuSchwach/Laden fehlen bewusst, solange es keine
    echten Batteriedaten gibt (ein erfundener Akku waere ohne Aussage); festgelegt ist schon: Laden wird extern
    ausgeloest (latched Topic wie /not_aus), AkkuSchwach zwingt wie NotAus zum sicheren Stehen.

    Ein Zustand entscheidet je Regeltakt nur zweierlei: welcher Fahrbefehl an die Policy geht und welcher Zustand
    im naechsten Takt gilt. Sensoren lesen, Policy rechnen und Zielwinkel senden ist fuer alle gleich
    (_policy_schritt). Den Knoten fragt ein Zustand nur ueber dessen Zustands-API ab (befehl_*, bewegung_gewuenscht,
    bewegung_freigegeben).
    """

    @abstractmethod
    def execute(self, node) -> "IRobotState":
        """Einen Regeltakt ausfuehren und den Zustand fuer den naechsten Takt zurueckgeben."""

    @staticmethod
    def _policy_schritt(node, cmd) -> None:
        """Sensoren lesen, Policy mit Fahrbefehl cmd [vx, vy, wz] rechnen, Zielwinkel an Layer 4 senden."""
        quat, omega = node.bridge.read_imu()
        joint_pos, joint_vel = node.bridge.read_joint_states()
        ziele = node.controller.compute_joint_targets(
            joint_positions=joint_pos, joint_velocities=joint_vel, base_quat=quat, base_ang_vel=omega,
            cmd=np.asarray(cmd, dtype=np.float32))
        node.sende_gelenkziele(ziele)
