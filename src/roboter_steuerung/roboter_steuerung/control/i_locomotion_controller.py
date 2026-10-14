from abc import ABC, abstractmethod
import numpy as np


class ILocomotionController(ABC):
    """
    Abstrakte Basisklasse (Layer 3) fuer alle Lauf-/Fortbewegungs-Regler.

    Definiert nur den "Vertrag" (eine Methode: aus Sensordaten + Kommando
    werden Ziel-Gelenkwinkel), keine Umsetzung - nach demselben Muster
    wie IRobotHardwareBridge (siehe hardware/i_hardware_bridge.py).

    Zweck: steuereinheit_node.py kennt nur diese eine Methode, nicht WIE
    die Ziel-Gelenkwinkel berechnet werden. Dadurch kann der eigentliche
    Regler austauschbar bleiben - aktuell RLLocomotionController (neuronale
    Netz-Policy), denkbar waeren z.B. spaeter ein klassischer Gang-Regler
    oder ein anderes trainiertes Netz - ohne den Rest der Steuerungslogik
    anzufassen.
    """

    @abstractmethod
    def compute_joint_targets(
        self,
        joint_positions: np.ndarray,
        joint_velocities: np.ndarray,
        base_quat: np.ndarray,
        base_ang_vel: np.ndarray,
        cmd: np.ndarray
    ) -> np.ndarray:
        """
        Berechnet aus den aktuellen Sensordaten (Gelenkwinkel/-geschwindigkeiten,
        Orientierung/Drehrate der Floating Base) und dem Geschwindigkeits-
        kommando die naechsten Ziel-Gelenkwinkel.
        Rueckgabe: target_dof_pos (ein Winkel pro gesteuertem Gelenk).
        """
        pass
