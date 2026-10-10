from abc import ABC, abstractmethod
import numpy as np


class ILocomotionController(ABC):
    """Schnittstelle für Fortbewegungs-Controller."""

    @abstractmethod
    def compute_joint_targets(
        self,
        joint_positions: np.ndarray,
        joint_velocities: np.ndarray,
        base_quat: np.ndarray,
        base_ang_vel: np.ndarray,
        cmd: np.ndarray,
        sim_time: float = None
    ) -> np.ndarray:
        pass