from abc import ABC, abstractmethod


class IRobotState(ABC):
    """
    Abstrakte Basisklasse (Layer 3, State Pattern) fuer die Roboter-
    Zustaende (siehe RoboterZustandsautomat im SysML-Modell,
    RoboterZustaende.sysml). Stand 12.10.2026: "Start", "Bereitschaft",
    "Gehen", "Balancieren" und "NotAus" sind auf dieser ROS2-Ebene
    implementiert (siehe start_state.py/bereitschaft_state.py/
    gehen_state.py/balancieren_state.py/not_aus_state.py). "AkkuSchwach"
    und "Laden" bleiben bewusst zurueckgestellt.

    ENTSCHEIDUNG (12.10.2026, Nutzerentscheidung, siehe Chat-Historie):
    AkkuSchwach/Laden werden bewusst NICHT mit einem erfundenen
    Platzhalter-Akku-Wert umgesetzt, sondern erst, wenn ein echter
    Roboter mit echten Batteriedaten angeschlossen ist - ein simulierter
    Software-Timer waere reine Fiktion ohne Aussagekraft. Zwei
    Designentscheidungen fuer DANN sind aber bereits festgelegt, damit
    sie nicht neu diskutiert werden muessen:
    1. "Laden" wird AUCH bei echter Hardware explizit extern ausgeloest
       (z.B. ueber ein eigenes Topic, nach demselben latched-Bool-Muster
       wie /not_aus) - NICHT automatisch, sobald AkkuSchwach erreicht
       ist. Begruendung: das Andocken an eine Ladestation ist ein
       eigener physischer Vorgang, den die Software nicht von sich aus
       als abgeschlossen annehmen kann.
    2. "AkkuSchwach" soll den Roboter analog zu NotAusState zum sicheren
       Stehenbleiben zwingen (Policy aktiv, cmd=[0,0,0]), statt normal
       weiterzulaufen, bis "Laden" beginnt - dieselbe Lektion wie bei
       NotAus: Weiterlaufen mit zu wenig Puffer ist riskant.

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
