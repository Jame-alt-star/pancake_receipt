from abc import ABC, abstractmethod


class IRobotState(ABC):
    """
    Abstrakte Basisklasse (Layer 3, State Pattern) fuer die Roboter-
    Zustaende "Gehen"/"Balancieren" (siehe RoboterZustandsautomat im
    SysML-Modell - dort als Teil eines groesseren Zustandsautomaten mit
    weiteren, hier nicht relevanten Zustaenden wie AkkuSchwach/Laden/
    NotAus, die auf dieser ROS2-Ebene bisher nicht abgebildet sind).

    Definiert nur den "Vertrag" (ein Zustand fuehrt pro Regelungstakt
    eine execute()-Methode aus und entscheidet dabei selbst, ob in einen
    anderen Zustand gewechselt wird), keine Umsetzung - nach demselben
    Muster wie IRobotHardwareBridge/ILocomotionController.

    Zweck: steuereinheit_node.py kennt nur diese eine Methode und haelt
    eine Referenz auf den jeweils aktuellen Zustand (self.state). Dadurch
    ist klar getrennt, WAS der Roboter gerade tun soll (Gehen vs.
    Balancieren) von WIE das technisch umgesetzt wird (aktuell fuer beide
    Zustaende die RL-Policy, siehe GehenState/BalancierenState - spaeter
    ggf. ein eigener Balance-Controller, siehe control/i_balance_controller.py,
    aktuell noch nicht implementiert).

    "node" (Context, hier immer SteuereinheitNode) wird bei jedem Aufruf
    durchgereicht statt im Zustand selbst gespeichert zu werden, damit ein
    Zustandsobjekt bei Bedarf wiederverwendbar bleibt und keinen eigenen
    Bezug auf einen bestimmten Knoten braucht.
    """

    @abstractmethod
    def execute(self, node) -> "IRobotState":
        """
        Fuehrt die Logik dieses Zustands fuer einen Regelungstakt aus
        (Ziel-Gelenkwinkel berechnen und ueber node.bridge senden) und
        gibt den fuer den NAECHSTEN Takt geltenden Zustand zurueck - das
        kann derselbe Zustand sein (kein Wechsel) oder ein anderer.
        """
        pass
