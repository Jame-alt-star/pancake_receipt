"""Greif-Auftraege als Ablaufsteuerung (ROS-frei): Greifen und Ablegen mit Anfahrt und Zuruecktreten.

Der ROS-Knoten (greif_dienst_node.py) und der Regelkreistest (tests/simulation/greif_helpers.py) rufen je Takt
`takt(eingang)` auf und setzen die Ausgabe um; so pruefen die Tests dieselbe Logik, die im System laeuft.

Ablauf Greifen:  warten (Objekt gesehen?) -> anfahren (Standpose vor dem Objekt) -> ausfuehren (Greifablauf) ->
                 pruefen (angehoben?) -> zuruecktreten (gerade 0,35 m rueckwaerts, dann faehrt Nav2 weiter).
                 Nicht angehoben: Haende oeffnen und zurueck (loesen), dann zuruecktreten, Ergebnis wiederholbar.
Ablauf Ablegen:  warten (Ablagestelle gesehen?) -> anfahren -> ausfuehren (Ablegeablauf) -> Arme ausblenden ->
                 zuruecktreten -> pruefen (Objekt an der Ablagestelle?).
Lagen kommen relativ zum Becken (wie von einer Kamera) und werden mit der aktuellen Odometrie in den Odometrie-Frame
umgerechnet; Drift der Odometrie geht so nur ueber die kurze Zeit zwischen Messung und Bewegung ein.
"""
from dataclasses import dataclass, field

import numpy as np

from roboter_steuerung.greifen import ablauf as greifablauf
from roboter_steuerung.greifen.anfahrt import Anfahrt, Rueckzug, standpose

FUEHREN, HALTEN, AUSBLENDEN = 1, 2, 3          # Armkommando-Modi (roboter_interfaces/Armkommando)


@dataclass
class Greifparameter:
    abstand: float = 0.26            # Objektmitte vor dem Becken [m]. IK exakt fuer 0,20-0,30 m und +-7 cm seitlich
                                     # (gemessen 11.11.2026); ein Schritt der Policy ist ~10 cm, die Anfahrt trifft
                                     # also nur auf +-5 cm, und beim Greifen setzt sich der Roboter um +-3 cm
    abstand_min_ablage: float = 0.22   # naeher stossen die Oberschenkel an die Tischkante
    abstand_max_ablage: float = 0.30
    halbbreite: float = 0.10         # halbe Breite des Objekts zwischen den Haenden [m]
    taille_pitch: float = -0.15      # volle Taillenneigung (wie steuerung.yaml manipulieren_taille_pitch)
    tol_dock: float = 0.05           # Anfahrt an den Tisch: Toleranz vor/zurueck [m] (Rest gleicht die IK aus)
    tol_dock_quer: float = 0.07      # ... seitlich [m]
    tol_dock_yaw: float = np.radians(10.0)   # die IK richtet die Haende nach der Objektlage aus
    rueck_weg: float = 0.35          # Zuruecktreten: gerade rueckwaerts [m], danach uebernimmt Nav2
    v_rueck: float = 0.3             # ... mit [m/s] (rueckwaerts mit Kiste: 0,5 m/s lief davon)
    quer_beim_drehen: float = 0.3    # beim Drehen zugleich seitlich in Drehrichtung [m/s]: aus dem Stand drehte der
                                     # Roboter rechts herum nicht an (mit Kiste -5 Grad in 4 s, ohne 2 Grad in 12 s),
                                     # mit Seitschritt -59 Grad in 3 s (gemessen 11.11.2026)
    v_bogen: float = 0.3             # Gehen beim Lenken und im letzten Stueck vor dem Tisch [m/s]
    linie: float = 0.5               # Anfahrt als Linienfolge: Zielpunkt so weit voraus auf der Linie [m]
    nah: float = 0.30                # ab hier Schrittpulse statt Gehen: aus dem Bogen rollt er 10 cm und mehr aus,
                                     # mit 0,15 lief er mit Kiste bis an die Tischkante und blieb haengen
    v_puls: float = 0.3              # Schrittpulse im Nahbereich der Anfahrt [m/s] (darunter kein Schritt)
    v: float = 0.35                  # Gehbefehl ohne Last [m/s]
    v_last: float = 0.5              # mit Last [m/s]: die Policy reagiert mit Kiste traeger (0,35 m/s -> 8 cm in 3 s)
    w: float = 1.0                   # Drehbefehl [rad/s] (mit Kiste dreht sie erst ab 1,0 zuverlaessig)
    anfahrt_timeout_s: float = 45.0
    sicht_timeout_s: float = 3.0     # so lange auf die erste Objektpose warten
    min_hub: float = 0.04            # gegriffen, wenn das Objekt mindestens so weit gehoben ist [m]
    ablage_tol: float = 0.06         # abgelegt, wenn das Objekt so nah an der Ablagestelle liegt [m]
    last_kg: float = 0.5             # Masse des Objekts fuer den Schwerkraft-Vorhalt der Arme [kg]
    arm_kp: float = 40.0             # kp der Armgelenke (policy_29dof.yaml), 0 = kein Vorhalt
    hand_kp: float = greifablauf.HAND_KP
    hand_kd: float = 0.05


@dataclass
class Eingang:
    t: float                         # Zeit [s] (monoton)
    pos: np.ndarray                  # Basis-Position (Odometrie-Frame)
    quat: np.ndarray                 # Basis-Orientierung (w, x, y, z)
    gelenke: np.ndarray              # 29 Koerpergelenke (MUJOCO_JOINT_ORDER)
    objekt: tuple = None             # (Position relativ zum Becken (3,), Orientierung relativ (w,x,y,z)) oder None
    ablage: tuple = None             # dasselbe fuer die Ablagestelle


@dataclass
class Ausgabe:
    cmd: tuple = None                # (vx, vy, wz) fuer /cmd_vel/greifen oder None (nichts senden)
    arm_modus: int = 0               # 0 = nichts senden, sonst FUEHREN/HALTEN/AUSBLENDEN
    arme: np.ndarray = None
    taille: float = 0.0
    haende: np.ndarray = None        # 14 Fingerziele oder None
    phase: str = ""
    fortschritt: float = 0.0
    fertig: bool = False
    erfolg: bool = False
    meldung: str = ""
    wiederholbar: bool = False


def _yaw(q) -> float:
    w, x, y, z = q
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def _drehmatrix(q) -> np.ndarray:
    w, x, y, z = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def in_odom(e: Eingang, rel):
    """Relative Objektpose (Position im Becken-Frame, Orientierung relativ als Quaternion w,x,y,z) ->
    (Position, Gier) im Odometrie-Frame. Volle Drehung: schon 5 Grad Beckenneigung verschieben ein Objekt 0,4 m vor
    dem Becken sonst um 3,5 cm in der Hoehe - mehr als die Greiftoleranz."""
    p_rel, q_rel = rel
    r = _drehmatrix(e.quat)
    p = np.asarray(e.pos, dtype=float) + r @ np.asarray(p_rel, dtype=float)
    r_obj = r @ _drehmatrix(q_rel)
    return p, float(np.arctan2(r_obj[1, 0], r_obj[0, 0]))


class Greifauftrag:
    """modus 'greifen' oder 'ablegen'. kin: Armkinematik (Modell mit Haenden), ruhe: Policy-Ruhepose (29).
    start_arme/start_haende/start_taille: Zustand der Haende zu Beginn (beim Ablegen: die Tragepose)."""

    def __init__(self, modus, kin, ruhe, par: Greifparameter, start_arme=None, start_haende=None, start_taille=0.0):
        assert modus in ("greifen", "ablegen")
        self.modus, self.kin, self.ruhe, self.par = modus, kin, np.asarray(ruhe, dtype=np.float32), par
        self.start_arme = None if start_arme is None else np.asarray(start_arme, dtype=np.float32)
        self.start_haende = (greifablauf.HAND_OFFEN.copy() if start_haende is None
                             else np.asarray(start_haende, dtype=np.float32))
        self.start_taille = start_taille
        self.phase = "warten"
        self._t0 = None
        self._t_phase = None
        self._anfahrt = None
        self._ablauf = None
        self._z0 = None                      # relative Hoehe des Objekts vor dem Greifen
        self.arme = self.start_arme          # zuletzt kommandierte Armziele (gehalten nach dem Greifen)
        self.haende = self.start_haende.copy()
        self.taille = start_taille
        self._ergebnis = None                # (erfolg, meldung, wiederholbar)
        self._nachgeplant = set()            # Phasen, vor denen schon nachgeplant wurde
        self._versatz = None                 # Ablegen: Handflaechenmitte relativ zur Kiste (Welt)
        self._korrektur = np.zeros(3)        # Greifen: summierter Fehler der Handflaechen (Welt)
        self._t_ausfuehren = None
        self._dauer_gesamt = 0.0

    # -------------------------------------------------------------- Hilfen
    def _wechsel(self, phase, t):
        self.phase, self._t_phase = phase, t

    def _basis(self, e):
        return float(e.pos[0]), float(e.pos[1]), _yaw(e.quat)

    def _arm_ausgabe(self, a: Ausgabe, modus=FUEHREN):
        a.arm_modus, a.arme, a.taille, a.haende = modus, self.arme, self.taille, self.haende
        return a

    def _beenden(self, a: Ausgabe):
        erfolg, meldung, wdh = self._ergebnis
        a.fertig, a.erfolg, a.meldung, a.wiederholbar = True, erfolg, meldung, wdh
        return a

    # -------------------------------------------------------------- Takt
    def takt(self, e: Eingang) -> Ausgabe:
        if self._t0 is None:
            self._t0, self._t_phase = e.t, e.t
        a = Ausgabe(phase=self.phase)
        ziel = e.objekt if self.modus == "greifen" else e.ablage
        haelt = self.arme is not None

        if self.phase == "warten":
            if ziel is not None:
                p, yaw = in_odom(e, ziel)
                abstand = self.par.abstand
                if self.modus == "ablegen" and e.objekt is not None:
                    # So hinstellen, dass die Kiste in der aktuellen Handpose schon ueber der Sollstelle steht und
                    # nur abgesenkt wird. Sie rutscht beim Tragen in den Haenden (gemessen 6 cm nach hinten); mit
                    # dem Nennabstand mussten die Arme 33 cm weit greifen, die Kiste blieb an der Tischkante haengen.
                    vor = float(e.objekt[0][0])          # Kistenmitte vor dem Becken [m]
                    abstand = float(np.clip(vor + 0.02, self.par.abstand_min_ablage, self.par.abstand_max_ablage))
                self._anfahrt = Anfahrt(standpose(p, yaw, abstand), tol_pos=self.par.tol_dock,
                                        tol_yaw=self.par.tol_dock_yaw, v=self.par.v_last if haelt else self.par.v,
                                        w=self.par.w, tol_quer=self.par.tol_dock_quer, v_puls=self.par.v_puls,
                                        quer_beim_drehen=self.par.quer_beim_drehen, v_bogen=self.par.v_bogen,
                                        linie=self.par.linie, nah=self.par.nah)
                if self.modus == "greifen":
                    self._z0 = float(p[2])
                self._wechsel("anfahren", e.t)
            elif e.t - self._t0 > self.par.sicht_timeout_s:
                self._ergebnis = (False, f"{'Objekt' if self.modus == 'greifen' else 'Ablagestelle'} nicht gesehen",
                                  self.modus == "greifen")
                self._wechsel("fertig", e.t)
                return self._beenden(a)
            return self._arm_ausgabe(a, HALTEN) if haelt else a

        if self.phase == "anfahren":
            vx, vy, wz, ok = self._anfahrt.befehl(self._basis(e), e.t)
            a.cmd = (vx, vy, wz)
            if ok:
                if ziel is None:
                    self._ergebnis = (False, "Ziel nach der Anfahrt nicht mehr gesehen", True)
                    self._starte_rueckzug(e)
                else:
                    self._plane(e, ziel)
                    self._t_ausfuehren, self._dauer_gesamt = e.t, self._ablauf.dauer
                    self._wechsel("ausfuehren", e.t)
                a.cmd = (0.0, 0.0, 0.0)
            elif e.t - self._t_phase > self.par.anfahrt_timeout_s:
                self._ergebnis = (False, "Anfahrt nicht erreicht (Zeit)", True)
                self._starte_rueckzug(e)
            return self._arm_ausgabe(a) if haelt else a

        if self.phase == "ausfuehren":
            arme, haende, taille, name, fertig = self._ablauf.sollwerte(e.t - self._t_phase)
            if (name in greifablauf.NACHPLANEN[self.modus] and name not in self._nachgeplant
                    and ziel is not None and (self.modus == "greifen" or e.objekt is not None)):
                # Rest ab dieser Phase aus der aktuellen Lage neu planen (Greifen: der Roboter setzt sich nach der
                # Anfahrt noch; Ablegen: Rest-Fehler der Kiste ueber der Sollstelle ausgleichen)
                self._nachgeplant.add(name)
                self._plane(e, ziel, ab_phase=name)
                self._t_phase = e.t
                arme, haende, taille, name, fertig = self._ablauf.sollwerte(0.0)
            self.arme, self.haende, self.taille = arme, haende, taille
            a.phase = name
            a.fortschritt = min(1.0, (e.t - self._t_ausfuehren) / max(self._dauer_gesamt, 1e-6))
            if fertig:
                self._nach_ablauf(e, ziel)
            return self._arm_ausgabe(a)

        if self.phase == "loesen":            # Fehlgriff: Haende oeffnen und Arme zurueck in die Ruhepose
            arme, haende, taille, name, fertig = self._ablauf.sollwerte(e.t - self._t_phase)
            self.arme, self.haende, self.taille = arme, haende, taille
            a.phase = "loesen"
            if fertig:
                self._ausblenden(e)
            return self._arm_ausgabe(a)

        if self.phase == "ausblenden":
            self._arm_ausgabe(a, AUSBLENDEN)
            if e.t - self._t_phase > 2.0:
                self.arme = None
                self._starte_rueckzug(e)
            return a

        if self.phase == "zuruecktreten":
            vx, vy, wz, ok = self._rueck.befehl(self._basis(e), e.t)
            a.cmd = (vx, vy, wz)
            if ok or e.t - self._t_phase > self.par.anfahrt_timeout_s:
                a.cmd = (0.0, 0.0, 0.0)
                if self.modus == "ablegen" and self._ergebnis is None:
                    self._ergebnis = self._pruefe_ablage(e)
                if not ok:
                    erfolg, meldung, wdh = self._ergebnis
                    self._ergebnis = (erfolg, meldung + " (Zuruecktreten nicht ganz erreicht)", wdh)
                self._wechsel("fertig", e.t)
                self._beenden(a)
            return self._arm_ausgabe(a) if self.arme is not None else a

        return self._beenden(a)

    # -------------------------------------------------------------- Teilschritte
    def _kin_setzen(self, e):
        self.kin.setze(e.pos, e.quat, e.gelenke)
        return self.kin

    def _plane(self, e, ziel, ab_phase=None):
        kin = self._kin_setzen(e)
        p, yaw = in_odom(e, ziel)
        start_arme = self.arme if self.arme is not None else kin.arme()
        ruhe_arme = self.ruhe[15:29]
        taille_basis = float(self.ruhe[14])
        if self.modus == "greifen":
            if ab_phase is not None:
                # Geschlossener Kreis: wo stehen die Handflaechen wirklich (Durchhang, Lage der Basis)? Soll ist die
                # Kistenmitte (seitlich und laengs) in der Hoehe der gerade beendeten Phase. Der Fehler wird von den
                # naechsten Zielen abgezogen und summiert - der Griff sitzt dann auf ~1 cm ueber dem Schwerpunkt
                # (2 cm daneben kippte die Kiste beim Anheben um 13 Grad).
                mitte = 0.5 * (kin.handflaeche("left")[0] + kin.handflaeche("right")[0])
                hoehe = greifablauf.UEBER if ab_phase == "vorgreifen" else greifablauf.GREIF_DZ
                soll = p + np.array([np.cos(yaw) * greifablauf.GREIF_DX, np.sin(yaw) * greifablauf.GREIF_DX, hoehe])
                self._korrektur = self._korrektur + (mitte - soll)
            p = p - self._korrektur
            self._ablauf = greifablauf.plane_greifen(kin, start_arme, self.haende, p, yaw, self.par.halbbreite,
                                                     ruhe_arme, taille_basis, self.par.taille_pitch, self.taille,
                                                     ab_phase=ab_phase, last_kg=self.par.last_kg,
                                                     arm_kp=self.par.arm_kp)
        else:
            if ab_phase is None:
                # Wo sitzt die Kiste gerade in den Haenden? (Handflaechenmitte relativ zur Kistenmitte)
                self._versatz = None
                if e.objekt is not None:
                    mitte = 0.5 * (kin.handflaeche("left")[0] + kin.handflaeche("right")[0])
                    self._versatz = mitte - in_odom(e, e.objekt)[0]
            elif self._versatz is not None:
                # Kiste sollte jetzt UEBER_ABLAGE ueber der Sollstelle stehen: Rest-Fehler waagrecht abziehen
                fehler = in_odom(e, e.objekt)[0] - (p + np.array([0.0, 0.0, greifablauf.UEBER_ABLAGE]))
                self._versatz = self._versatz - np.array([fehler[0], fehler[1], 0.0])
            self._ablauf = greifablauf.plane_ablegen(kin, start_arme, self.haende, p, yaw, self.par.halbbreite,
                                                     ruhe_arme, taille_basis, self.par.taille_pitch, self.taille,
                                                     last_kg=self.par.last_kg, arm_kp=self.par.arm_kp,
                                                     griff_versatz=self._versatz, ab_phase=ab_phase)

    def _nach_ablauf(self, e, ziel):
        if self.modus == "greifen":
            hub = (float(in_odom(e, ziel)[0][2]) - self._z0) if ziel is not None else 0.0
            if hub >= self.par.min_hub:
                self._ergebnis = (True, f"gegriffen, um {hub * 100:.0f} cm angehoben", False)
                self._starte_rueckzug(e)         # Arme bleiben in der Tragepose (gefuehrt)
            else:
                self._ergebnis = (False, f"nicht gegriffen (Hub {hub * 100:.1f} cm)", True)
                self._plane_loesen(e)
        else:
            self._ausblenden(e)

    def _plane_loesen(self, e):
        """Nach einem Fehlgriff: Finger oeffnen, Haende seitlich weg, hoch, vor die Brust, Ruhepose (wie beim Ablegen,
        aber an der Stelle, an der die Haende gerade sind)."""
        kin = self._kin_setzen(e)
        p, yaw = in_odom(e, e.objekt) if e.objekt is not None else (None, None)
        if p is None:
            ab = greifablauf.Ablauf(self.arme.copy(), self.haende.copy(), start_taille=self.taille)
            ab.phasen.append(greifablauf.Phase("oeffnen", 1.0, self.arme.copy(), greifablauf.HAND_OFFEN.copy(),
                                               self.taille))
            ab.phasen.append(greifablauf.Phase("zurueck", 3.0, self.ruhe[15:29].copy(),
                                               greifablauf.HAND_OFFEN.copy(), 0.0))
        else:
            # Haende bleiben, wo sie sind (Kiste steht auf dem Tisch): Versatz gemessen, ohne Eindruecken
            mitte = 0.5 * (kin.handflaeche("left")[0] + kin.handflaeche("right")[0])
            versatz = mitte - p + np.array([0.0, 0.0, greifablauf.EINDRUECKEN])
            ab = greifablauf.plane_ablegen(kin, self.arme, self.haende, p, yaw, self.par.halbbreite, self.ruhe[15:29],
                                           float(self.ruhe[14]), self.par.taille_pitch, self.taille,
                                           last_kg=0.0, arm_kp=self.par.arm_kp,    # Fehlgriff: keine Last
                                           griff_versatz=versatz, ab_phase="loesen")  # Haende sind schon dort
        self._ablauf = ab
        self._wechsel("loesen", e.t)

    def _ausblenden(self, e):
        self._wechsel("ausblenden", e.t)

    def _starte_rueckzug(self, e):
        self._rueck = Rueckzug(weg=self.par.rueck_weg, v=self.par.v_rueck)
        self._wechsel("zuruecktreten", e.t)

    def _pruefe_ablage(self, e):
        if e.objekt is None or e.ablage is None:
            return (False, "Objekt oder Ablagestelle nach dem Ablegen nicht gesehen", False)
        d = in_odom(e, e.objekt)[0] - in_odom(e, e.ablage)[0]
        if float(np.hypot(d[0], d[1])) <= self.par.ablage_tol and abs(float(d[2])) <= 0.03:
            return (True, f"abgelegt ({np.hypot(d[0], d[1]) * 100:.0f} cm neben der Sollstelle)", False)
        return (False, f"Objekt nicht an der Ablagestelle (Abstand {np.hypot(d[0], d[1]) * 100:.0f} cm, "
                       f"Hoehe {d[2] * 100:+.0f} cm)", False)
