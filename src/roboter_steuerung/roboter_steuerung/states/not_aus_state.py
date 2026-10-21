from roboter_steuerung.states.i_robot_state import IRobotState


class NotAusState(IRobotState):
    """NotAus: aktiv geregelter Stillstand - Policy laeuft mit Befehl 0, ohne Kurshaltung (maximal vorhersagbar).
    Wird zentral im Knoten erzwungen und aufgehoben (unterbricht jeden Zustand); danach Bereitschaft mit neuer
    Freigabe-Pflicht.

    Warum nicht "Motoren aus" oder Einfrieren der Gelenke: ein dynamisch balancierender Zweibeiner faellt dann um.
    Getestet (12.10.2026): sofortiges Einfrieren und Einfrieren nach 1,5 s bzw. 6 s Abbremsen - jedes Mal Sturz.
    Gemessen mit der 29-DOF-Policy: aus voller Fahrt in ~2-3 s Stillstand, Beine danach praktisch still.
    Auf echter Hardware ersetzt das nicht den physischen Not-Halt (siehe docs/architektur.md, Sicherheit)."""

    def execute(self, node) -> IRobotState:
        self._policy_schritt(node, (0.0, 0.0, 0.0))
        return self
