"""Anfahrt vom Navigationsziel an die Greif- bzw. Ablageposition und zurueck (ROS-frei).

Nav2 bringt den Roboter nur auf etwa 0,85 m an den Tisch (Roboterradius + Inflation, xy-Toleranz 0,25 m) und mit
beliebiger Ausrichtung. Den Rest faehrt der Greif-Dienst ueber einen Vorrang-Eingang der Steuerung selbst.

Die Lauf-Policy hat Totzonen (gemessen 11.11.2026 im Regelkreis, 4 s je Befehl):
  - Drehen auf der Stelle: ohne Last 0,8 rad/s -> 80 Grad, mit Kiste nur 9 Grad; 1,0 rad/s -> 110 Grad mit Kiste.
  - Seitwaerts mit Kiste: 0,35 m/s -> 0,9-1,2 m seitlich, aber zugleich 0,7 m nach vorn. Ohne Last sauber.
  - Gehen: erst ab ~0,3 m/s (roboter_task_planning/README.md).
Deshalb kein P-Regler mit kleinen Werten und keine Seitwaertsschritte, sondern "drehen - gerade fahren - drehen" mit
festen Befehlen (wie ein Differentialantrieb), danach erneut pruefen. Liegt das Ziel hinter dem Roboter, faehrt er
rueckwaerts statt sich umzudrehen (Zuruecktreten vom Tisch). Die Arme gleichen den Rest aus (IK fuer die
tatsaechliche Lage der Kiste).
"""
import numpy as np


def standpose(ziel_pos, ziel_yaw: float, abstand: float):
    """Becken-Sollpose (x, y, yaw), damit die Kistenmitte `abstand` m vor dem Becken liegt (Blick zur Kiste)."""
    x = float(ziel_pos[0]) - abstand * np.cos(ziel_yaw)
    y = float(ziel_pos[1]) - abstand * np.sin(ziel_yaw)
    return x, y, float(ziel_yaw)


def winkel(a: float) -> float:
    return float((a + np.pi) % (2 * np.pi) - np.pi)


class Anfahrt:
    """Jeder Aufruf von befehl(ist, t) liefert (vx, vy, wz, fertig). ist = (x, y, yaw) der Basis im selben Frame wie
    soll = (x, y, yaw). tol_pos (vor/zurueck), tol_quer (seitlich), tol_yaw: Zieltoleranz; fertig erst, wenn der
    Roboter beruhigen_s still innerhalb steht (die Policy rollt nach dem Anhalten aus).

    Fern (> nah): Linienfolge. Mit linie > 0 faehrt der Roboter auf eine gedachte Linie, die in der Sollpose endet
      und in Sollrichtung zeigt (Zielpunkt linie m voraus auf der Linie, wz proportional zum Peilfehler, wie ein
      Fahrzeug beim Andocken). So kommt er schon ausgerichtet an und muss am Tisch kaum noch drehen - Drehen auf der
      Stelle und kleine Schritte sind die Schwaechen der Policy, besonders mit Kiste. Auf der Stelle (mit Seitschritt)
      dreht er nur bei Peilfehlern ueber bogen_max_winkel. linie = 0: direkt auf den Zielpunkt (Ersatz fuer Nav2).
    Nah (< nah, mit Hysterese): erst ausrichten, dann Schrittpulse. Gemessen 11.11.2026 im Regelkreis mit Rampe:
      - feste 0,35 m/s schiessen nach dem Stopp ~10 cm ueber (einmal bis an die Tischkante),
      - zum Zielpunkt drehen pendelt endlos (wenige cm seitlich = grosse Peilung),
      - unter 0,2 m/s (vor) bzw. 0,25 m/s (seitlich) lehnt sich der Roboter nur und federt zurueck,
      - der kleinste Puls, der einen Schritt ausloest, bewegt ihn 9-12 cm (vx 0,3 fuer 0,5 s, vy 0,3 fuer 0,8 s).
    Also: Puls, beruhigen, messen, naechster Puls. Bewegt ein Puls nichts (z. B. mit Last), wird er laenger.
    Die Toleranzen muessen deshalb mindestens einen halben Schritt betragen; den Rest gleichen die Arme aus."""

    def __init__(self, soll, tol_pos=0.05, tol_yaw=np.radians(5.0), v=0.35, w=1.0, beruhigen_s=1.5,
                 tol_peilung=np.radians(10.0), tol_quer=None, nah=0.30, v_puls=0.3, puls_s=(0.5, 0.8),
                 puls_max_s=2.0, quer_beim_drehen=0.0, v_bogen=0.3, bogen_max_winkel=np.radians(35.0), k_lenk=2.0,
                 linie=0.5, langsam_ab=0.6):
        self.soll = soll
        self.tol_pos, self.tol_yaw, self.v, self.w = tol_pos, tol_yaw, v, w
        self.tol_quer = tol_pos if tol_quer is None else tol_quer
        self.tol_peilung = tol_peilung
        self.beruhigen_s = beruhigen_s
        self.nah, self.v_puls, self.puls_max_s = nah, v_puls, puls_max_s
        self.quer_beim_drehen = quer_beim_drehen   # Seitschritt beim Drehen (aus dem Stand dreht die Policy sonst nicht)
        # Gehen und dabei lenken (wz proportional zum Peilfehler) statt auf der Stelle zu drehen: mit Kiste rutschte sie
        # beim Drehen mit Seitschritt 7,7 cm in den Haenden und fiel beim Richtungswechsel; im Bogen 0,0 cm
        # (gemessen 11.11.2026). Ein fester Bogen (0,3 m/s, 1 rad/s = 0,3 m Radius) kreiste um nahe Ziele.
        self.v_bogen, self.bogen_max_winkel, self.k_lenk = v_bogen, bogen_max_winkel, k_lenk
        self.linie, self.langsam_ab = linie, langsam_ab
        self.puls_dauer = list(puls_s)       # je Achse (vor, seitlich), passt sich an
        self.pulse = 0                       # Anzahl Pulse (Diagnose)
        self._nahmodus = False
        self._still_seit = None
        self._puls = None                    # (achse, richtung, t_ende, startpose)
        self._auswerten = None               # beendeter Puls, wird nach dem Beruhigen ausgewertet
        self._drehen = False
        self._ruhe_ab = None                 # Zeit des letzten Befehls im Nahbereich

    def fehler(self, ist):
        """(Laengs-, Quer-Fehler im Roboter-Frame [m], Winkelfehler zur Sollausrichtung [rad])."""
        x, y, yaw = ist
        ex_w, ey_w = self.soll[0] - x, self.soll[1] - y
        c, s = np.cos(yaw), np.sin(yaw)
        return c * ex_w + s * ey_w, -s * ex_w + c * ey_w, winkel(self.soll[2] - yaw)

    def _im_ziel(self, ex, ey, eyaw):
        return abs(ex) <= self.tol_pos and abs(ey) <= self.tol_quer and abs(eyaw) <= self.tol_yaw

    def _zielpunkt(self, ist):
        """Punkt, auf den der Roboter zuhaelt: linie m voraus auf der Anfahrlinie (nicht ueber die Sollpose hinaus),
        bei linie = 0 die Sollpose selbst."""
        if self.linie <= 0.0:
            return self.soll[0], self.soll[1]
        c, s = np.cos(self.soll[2]), np.sin(self.soll[2])
        laengs = c * (ist[0] - self.soll[0]) + s * (ist[1] - self.soll[1])    # < 0: vor der Sollpose
        s_ziel = min(laengs + self.linie, 0.0)
        return self.soll[0] + c * s_ziel, self.soll[1] + s * s_ziel

    def _dreh(self, richtung):
        return [0.0, float(np.sign(richtung) * self.quer_beim_drehen), float(np.sign(richtung) * self.w)]

    def _werte_puls_aus(self, ist):
        achse, richtung, _, start = self._auswerten
        self._auswerten = None
        c, s = np.cos(start[2]), np.sin(start[2])
        dx, dy = ist[0] - start[0], ist[1] - start[1]
        weg = (c * dx + s * dy) if achse == 0 else (-s * dx + c * dy)
        if richtung * weg < 0.03:            # kein Schritt ausgeloest (Totzone, Last): naechstes Mal laenger
            self.puls_dauer[achse] = min(self.puls_max_s, 1.5 * self.puls_dauer[achse])
        elif abs(weg) > 0.15:
            self.puls_dauer[achse] = max(0.3, 0.75 * self.puls_dauer[achse])

    def _nah(self, ist, t, ex, ey, eyaw):
        """Nahbereich: (cmd, fertig)."""
        if self._puls is not None:
            achse, richtung, ende, _ = self._puls
            if t < ende:
                cmd = [0.0, 0.0, 0.0]
                cmd[achse] = richtung * self.v_puls
                return cmd, False
            self._auswerten, self._puls, self._ruhe_ab = self._puls, None, t
        if self._drehen:
            if abs(eyaw) > 0.5 * self.tol_yaw:
                return self._dreh(eyaw), False
            self._drehen, self._ruhe_ab = False, t
        if self._ruhe_ab is not None and t - self._ruhe_ab < self.beruhigen_s:
            return [0.0, 0.0, 0.0], False
        if self._auswerten is not None:
            self._werte_puls_aus(ist)
        if self._im_ziel(ex, ey, eyaw):
            return [0.0, 0.0, 0.0], True
        if abs(eyaw) > self.tol_yaw:
            self._drehen = True
            return self._dreh(eyaw), False
        achse = 0 if abs(ex) > self.tol_pos else 1
        richtung = float(np.sign(ex if achse == 0 else ey))
        self._puls = (achse, richtung, t + self.puls_dauer[achse], tuple(ist))
        self.pulse += 1
        cmd = [0.0, 0.0, 0.0]
        cmd[achse] = richtung * self.v_puls
        return cmd, False

    def befehl(self, ist, t: float):
        ex, ey, eyaw = self.fehler(ist)
        abstand = float(np.hypot(ex, ey))
        if not self._nahmodus and abstand < self.nah:
            self._nahmodus, self._ruhe_ab = True, t          # erst ausrollen lassen, dann messen
        elif self._nahmodus and abstand > 1.6 * self.nah and self._puls is None:
            self._nahmodus, self._drehen = False, False
        if self._nahmodus:
            cmd, fertig = self._nah(ist, t, ex, ey, eyaw)
            return cmd[0], cmd[1], cmd[2], fertig
        cmd = [0.0, 0.0, 0.0]
        if abstand > self.tol_pos:
            ziel = self._zielpunkt(ist)
            zx, zy = ziel[0] - ist[0], ziel[1] - ist[1]
            c, s = np.cos(ist[2]), np.sin(ist[2])
            peilung = float(np.arctan2(-s * zx + c * zy, c * zx + s * zy))     # zum Zielpunkt, im Roboter-Frame
            rueckwaerts = abs(peilung) > np.pi / 2 and abstand < 1.0
            fehler_peilung = winkel(peilung - np.pi) if rueckwaerts else peilung
            if rueckwaerts:
                if abs(fehler_peilung) > self.tol_peilung:
                    cmd = self._dreh(fehler_peilung)
                else:
                    cmd[0] = -self.v
            elif abs(fehler_peilung) > self.bogen_max_winkel:
                cmd = self._dreh(fehler_peilung)
            else:
                # schnell nur fast geradeaus und weit weg; mit 0,5 m/s und voller Drehrate (Radius 0,5 m) trug ein
                # Bogen den Roboter neben den Tisch (gemessen 11.11.2026)
                gerade = abs(fehler_peilung) <= self.tol_peilung and abstand > self.langsam_ab
                v = self.v if gerade else self.v_bogen
                cmd = [v, 0.0, float(np.clip(self.k_lenk * fehler_peilung, -self.w, self.w))]
        elif abs(eyaw) > self.tol_yaw:
            cmd = self._dreh(eyaw)
        if any(cmd):
            self._still_seit = None
            return cmd[0], cmd[1], cmd[2], False
        if self._still_seit is None:
            self._still_seit = t
        return 0.0, 0.0, 0.0, t - self._still_seit >= self.beruhigen_s


class Rueckzug:
    """Vom Tisch zuruecktreten: gerade rueckwaerts um `weg` m (entlang der Ausrichtung beim Start), dann stehen.
    Nicht zur Startpose zurueck: rueckwaerts mit Kiste ist instabil - mit -0,5 m/s ueber 0,7 m lief der Roboter
    nicht mehr an und rannte 12 m rueckwaerts weiter (gemessen 11.11.2026). Den Rest faehrt Nav2 (dreht auf der
    Stelle und geht vorwaerts). befehl(ist, t) -> (vx, vy, wz, fertig)."""

    def __init__(self, weg=0.35, v=0.3, auslauf=0.08, beruhigen_s=1.5, timeout_s=8.0):
        self.weg, self.v, self.auslauf, self.beruhigen_s, self.timeout_s = weg, v, auslauf, beruhigen_s, timeout_s
        self._start = None
        self._t0 = None
        self._stopp = None

    def zurueckgelegt(self, ist) -> float:
        if self._start is None:
            return 0.0
        x0, y0, yaw0 = self._start
        return -(np.cos(yaw0) * (ist[0] - x0) + np.sin(yaw0) * (ist[1] - y0))

    def befehl(self, ist, t: float):
        if self._start is None:
            self._start, self._t0 = tuple(ist), t
        if self._stopp is None and (self.zurueckgelegt(ist) >= self.weg - self.auslauf or t - self._t0 > self.timeout_s):
            self._stopp = t
        if self._stopp is None:
            return -self.v, 0.0, 0.0, False
        return 0.0, 0.0, 0.0, t - self._stopp >= self.beruhigen_s
