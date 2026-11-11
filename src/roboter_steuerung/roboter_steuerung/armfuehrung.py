"""Armfuehrung (Layer 3, ROS-frei): gefuehrte Arme neben dem Zustandsautomaten.

Warum kein eigener Zustand: NotAus und Freigabe-Entzug ersetzen jeden Zustand (NotAus bzw. Bereitschaft). Waere das
Fuehren ein Zustand, fielen die Arme dabei sofort auf die Policy zurueck, die Kiste fiele, und die Policy saehe die
Arme wieder unmaskiert - genau der gemessene Sturzfall (tests/simulation/test_greifen_machbarkeit.py). Die
Armfuehrung laeuft deshalb unabhaengig vom Zustand mit; jeder Zustand, der die Policy rechnet, wendet sie an
(IRobotState._policy_schritt). Nur Stoerung sendet gar nichts (Layer 4 daempft).

Modi: AUS (Policy fuehrt die Arme), FUEHREN (Ziele vom Greif-Dienst), HALTEN (letzte Ziele bleiben),
AUSBLENDEN (Arme weich zurueck an die Policy, danach AUS).
  - Bleiben die Kommandos beim Fuehren laenger als timeout_s aus (Greif-Dienst abgestuerzt), wird gehalten.
  - Zurueck an die Policy nur auf ausdrueckliches AUSBLENDEN.
  - Armziele aendern sich hoechstens um max_rate (rad/s) je Gelenk (Sprunglimit der Steuereinheit: 30 rad/s).
Solange nicht AUS: Policy blind fuer die Arme, Armziele ersetzen die Policy, Taille geneigt (greifen/manipulation.py).
"""
import numpy as np

AUS, FUEHREN, HALTEN, AUSBLENDEN = "aus", "fuehren", "halten", "ausblenden"


class Armfuehrung:

    def __init__(self, namen, timeout_s=0.5, max_rate=2.0, einblenden_s=0.5, ausblenden_s=1.5):
        self.namen = list(namen)                 # erwartete 14 Armgelenke in Reihenfolge
        self.timeout_s, self.max_rate = timeout_s, max_rate
        self.einblenden_s, self.ausblenden_s = einblenden_s, ausblenden_s
        self.modus = AUS
        self.ziele = None                        # Soll vom Greif-Dienst
        self.ausgabe = None                      # ratenbegrenzte Armziele (zuletzt ausgegeben)
        self.taille_anteil = 0.0
        self.gewicht = 0.0                       # 1 = Arme voll gefuehrt
        self._letzte_nachricht = None

    @property
    def aktiv(self) -> bool:
        return self.modus != AUS

    def empfange(self, modus: int, namen, ziele, taille_anteil: float, jetzt: float):
        """Kommando uebernehmen (modus 1 FUEHREN, 2 HALTEN, 3 AUSBLENDEN). Rueckgabe: Fehlertext oder None."""
        if modus == 1:
            if list(namen) != self.namen:
                return f"Armkommando mit falschen Gelenknamen ignoriert (erwartet {self.namen})"
            z = np.asarray(ziele, dtype=np.float32)
            if z.shape != (len(self.namen),) or not np.all(np.isfinite(z)):
                return "Armkommando mit falscher Laenge oder ungueltigen Werten ignoriert"
            self.ziele = z
            self.taille_anteil = float(np.clip(taille_anteil, 0.0, 1.0))
            self.modus = FUEHREN
            self._letzte_nachricht = jetzt
        elif modus == 2:
            if self.modus != AUS:
                self.modus = HALTEN
        elif modus == 3:
            if self.modus != AUS:
                self.modus = AUSBLENDEN
        else:
            return f"Armkommando mit unbekanntem Modus {modus} ignoriert"
        return None

    def halten(self):
        """Von aussen erzwingen (NotAus, Freigabe-Entzug): letzte Ziele halten, nichts loslassen."""
        if self.modus in (FUEHREN, AUSBLENDEN):
            self.modus = HALTEN

    def takt(self, policy_arme: np.ndarray, gemessene_arme: np.ndarray, jetzt: float, dt: float):
        """Einen Regeltakt rechnen. Rueckgabe (arme_ziele, gewicht, taille_anteil) oder None bei AUS.
        policy_arme: Armziele der Policy in diesem Takt (Ziel beim Ausblenden), gemessene_arme: Startwert."""
        if self.modus == AUS:
            self.ausgabe, self.gewicht = None, 0.0
            return None
        if self.modus == FUEHREN and self._letzte_nachricht is not None \
                and jetzt - self._letzte_nachricht > self.timeout_s:
            self.modus = HALTEN
        if self.ausgabe is None:
            self.ausgabe = np.asarray(gemessene_arme, dtype=np.float32).copy()
        if self.modus == AUSBLENDEN:
            self.gewicht = max(0.0, self.gewicht - dt / self.ausblenden_s)
            self.taille_anteil = max(0.0, self.taille_anteil - dt / self.ausblenden_s)
            if self.gewicht <= 0.0:
                self.modus, self.ausgabe, self.ziele = AUS, None, None
                self.taille_anteil = 0.0
                return None
            return self.ausgabe.copy(), self.gewicht, self.taille_anteil
        self.gewicht = min(1.0, self.gewicht + dt / self.einblenden_s)
        if self.modus == FUEHREN and self.ziele is not None:
            schritt = self.max_rate * dt
            self.ausgabe = (self.ausgabe + np.clip(self.ziele - self.ausgabe, -schritt, schritt)).astype(np.float32)
        return self.ausgabe.copy(), self.gewicht, self.taille_anteil
