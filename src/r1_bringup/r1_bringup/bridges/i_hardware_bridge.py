from abc import ABC, abstractmethod
from typing import Tuple
import numpy as np


class IRobotHardwareBridge(ABC):
    """Abstrakte Basisklasse für alle Hardware- und Simulationsschnittstellen."""

    @abstractmethod
    def read_imu(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Liest IMU-Werte aus.
        Rückgabe: (quaternion [w, x, y, z], angular_velocity [wx, wy, wz])
        """
        pass

    @abstractmethod
    def read_joint_states(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Liest Gelenkwerte aus.
        Rückgabe: (joint_positions, joint_velocities)
        """
        pass

    @abstractmethod
    def send_joint_commands(self, target_positions: np.ndarray) -> None:
        """
        Sendet Ziel-Gelenkwinkel an die Hardware oder Simulation.
        """
        pass