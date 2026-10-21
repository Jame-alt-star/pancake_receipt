from abc import ABC, abstractmethod
from typing import Tuple
import numpy as np


class IRobotHardwareBridge(ABC):
    """
    Schnittstelle von Layer 3 zur Hardware bzw. Simulation (Layer 4). Liegt bewusst in Layer 3: die
    Steuerung legt fest, was sie braucht; Layer 4 (Simulation oder spaeter die Unitree-SDK-Anbindung)
    implementiert es. So haengt die Steuerung von keiner Simulation ab.

    Definiert nur den "Vertrag" (welche Methoden es geben muss), keine
    Umsetzung - deshalb ABC (Abstract Base Class): davon kann man kein
    eigenes Objekt erzeugen, nur konkrete Unterklassen wie TopicBridge.

    Zweck: steuereinheit_node.py und der RL-Controller kennen nur diese
    Methoden, nicht WIE/WOHIN sie tatsaechlich Daten lesen/schreiben.
    Dadurch kann die Ausfuehrungsplattform (MuJoCo-Simulation oder
    echte Hardware per CycloneDDS/unitree_sdk2) ausgetauscht werden, ohne
    den Rest der Steuerungslogik anzufassen - das ist der Kernpunkt der
    gesamten Architektur-Entscheidung fuer dieses Interface.
    """

    @abstractmethod
    def read_imu(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Liest die zuletzt bekannten IMU-Werte aus (nicht blockierend -
        gibt einfach den letzten empfangenen Wert zurueck).
        Rueckgabe: (quaternion [w, x, y, z], angular_velocity [wx, wy, wz])
        """
        pass

    @abstractmethod
    def read_joint_states(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Liest die zuletzt bekannten Gelenkwerte aus.
        Rueckgabe: (joint_positions, joint_velocities)
        """
        pass

    @abstractmethod
    def send_joint_commands(self, target_positions: np.ndarray) -> None:
        """
        Sendet Ziel-Gelenkwinkel an die Hardware oder Simulation. Wie das
        konkret passiert (ROS2-Topic, direkter SDK-Aufruf, ...) entscheidet
        die jeweilige Implementierung dieser Klasse, nicht der Aufrufer.
        """
        pass

    @abstractmethod
    def sensor_alter_s(self) -> float:
        """Alter der aeltesten der zuletzt empfangenen Messungen (IMU, Gelenke) in Sekunden; inf, solange noch
        keine kam. Die Steuerung erkennt daran einen Sensorausfall, statt mit alten Werten weiterzuregeln."""
        pass

    def sensor_detail(self) -> str:
        """Optional: Alter je Sensorquelle als Text, fuer die Diagnose einer Stoerung."""
        return ""
