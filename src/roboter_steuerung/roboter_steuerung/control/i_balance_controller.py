from abc import ABC, abstractmethod
import numpy as np


class IBalanceController(ABC):
    """
    Abstrakte Basisklasse (Layer 3) fuer alle Balance-/Stand-Regler.

    Definiert nur den "Vertrag" (aus Koerper-Orientierung/-Drehrate
    werden Ziel-Gelenkwinkel, die den Roboter stabil auf der Stelle
    stehen lassen sollen - OHNE Fortbewegungskommando, anders als
    ILocomotionController), keine Umsetzung - nach demselben Muster wie
    ILocomotionController/IRobotHardwareBridge.

    Zweck: BalancierenState (siehe roboter_steuerung/states/) kennt nur
    diese eine Methode, nicht WIE die Korrektur berechnet wird. Aktuell
    einzige Implementierung: PBalanceController (siehe dort fuer Details,
    Herleitung und bekannte Einschraenkungen).
    """

    @abstractmethod
    def compute_joint_targets(
        self,
        joint_positions: np.ndarray,
        joint_velocities: np.ndarray,
        base_quat: np.ndarray,
        base_ang_vel: np.ndarray,
        default_pose: np.ndarray
    ) -> np.ndarray:
        """
        Berechnet aus der aktuellen Koerper-Orientierung/-Drehrate eine
        kleine Korrektur um default_pose herum, mit der der Roboter
        stabil stehen bleiben soll. Kein Geschwindigkeitskommando wie bei
        ILocomotionController - dieser Regler soll NUR balancieren, nicht
        sich fortbewegen. default_pose wird als Parameter uebergeben
        (statt hier fest anzunehmen), damit dieser Regler nicht an eine
        bestimmte Policy/Config gekoppelt ist - der Aufrufer (aktuell
        BalancierenState) entscheidet, welche Ruhepose gilt.
        """
        pass
