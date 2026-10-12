from abc import ABC, abstractmethod
from typing import Tuple
import numpy as np


class IRobotHardwareBridge(ABC):
    """
    Abstrakte Basisklasse (Layer 4 / HAL) fuer alle Hardware- und
    Simulationsschnittstellen.

    Definiert nur den "Vertrag" (welche Methoden es geben muss), keine
    Umsetzung - deshalb ABC (Abstract Base Class): davon kann man kein
    eigenes Objekt erzeugen, nur konkrete Unterklassen wie MujocoBridge.

    Zweck: steuereinheit_node.py und der RL-Controller kennen nur diese
    Methoden, nicht WIE/WOHIN sie tatsaechlich Daten lesen/schreiben.
    Dadurch kann die Ausfuehrungsplattform (MuJoCo, spaeter Gazebo oder
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
    def read_base_position(self) -> np.ndarray:
        """
        NEU (12.10.2026): liest die zuletzt bekannte Basis-Position
        (Weltkoordinaten) aus, fuer die Positions-Haltung beim Stehen
        (siehe steuereinheit_node.py/_apply_position_hold()).
        Rueckgabe: [x, y, z]

        WICHTIGER VORBEHALT: in der MuJoCo-Simulation (MujocoBridge) ist
        das reiner Ground-Truth aus dem Simulationszustand - auf echter
        Hardware gibt es keine derart einfache, direkte Positionsquelle,
        dafuer braeuchte es echte Odometrie/Lokalisierung (z.B. aus
        Nav2/SLAM). Eine spaetere Hardware-Implementierung dieser
        Schnittstelle muesste das entsprechend ersetzen - hier bewusst
        als einfache Annahme fuer den aktuellen Simulationsstand belassen.
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
