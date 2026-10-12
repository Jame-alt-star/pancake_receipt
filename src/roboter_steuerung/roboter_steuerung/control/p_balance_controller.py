import numpy as np

from roboter_steuerung.control.i_balance_controller import IBalanceController


class PBalanceController(IBalanceController):
    """
    Erste, einfache Implementierung von IBalanceController ("P" fuer
    Proportional-artige Rueckfuehrung, analog zum Namensschema von
    RLLocomotionController/"rl_"). Entstanden direkt aus einem konkreten
    Problem (11.10.2026): weder die Lauf-Policy mit Null-Kommando noch
    eine starre Ruhepose (nur PD-Regelung, kein aktiver Regler) konnten
    den Roboter stabil stehend halten - die Policy driftete seitlich weg,
    die starre Pose kippte nach vorne. Beides zeigt: default_angles ist
    nur ein Referenzpunkt, keine fuer sich genommen stabile Standpose -
    es braucht eine aktive Korrektur.

    REGELANSATZ ("Ankle Strategy"): bei kleinen Stoerungen reicht es laut
    Literatur zur bipedalen Standregelung, ueber die Knoechelgelenke eine
    Korrektur proportional zu einer kombinierten Kippwinkel-/Kippraten-
    Groesse einzuspeisen (aehnlich einem Capture-Point-Ansatz, nicht nur
    reines P auf den Winkel allein - macht die Reaktion weniger
    "nervoes"/ueberschiessend als reines P):

        P = theta + sqrt(l/g) * theta_dot
        korrektur = -kp * P

    theta = Vorwaerts-/Rueckwaerts-Kippwinkel (Pitch), theta_dot = dessen
    Rate, l = Naeherung fuer die "Pendellaenge" (Knoechel bis Schwerpunkt
    im Stehen), g = Erdbeschleunigung. Quelle/Herleitung: Carpentier et
    al., "Push recovery"-Literaturueberblick in Frontiers in Robotics and
    AI (2015), frontiersin.org/articles/10.3389/frobt.2015.00021 - dort
    fuer DREHMOMENT hergeleitet; hier wird dieselbe Stabilitaetsgroesse P
    stattdessen in eine kleine GELENKWINKEL-Korrektur uebersetzt, weil
    unsere Architektur durchgaengig Positions-Ziele (nicht Drehmomente)
    zwischen Layer 3 und der PD-Regelung in mujoco_node.py austauscht.
    Die Korrektur wird auf Knoechel- UND Hueft-Pitch-Gelenke angewendet
    (Huefte mit kleinerem Anteil) - exakt dieselben vier Gelenke wie beim
    alten, bereits getesteten R1-Ausgleichsregler (siehe Projektstand,
    Abschnitt "roboter_ws"), nur mit der P-Groesse statt reinem
    Kippwinkel-Proportionalregler.

    BEKANNTE EINSCHRAENKUNGEN (identisch zum Befund beim alten R1-Regler,
    nicht geloest, nur uebernommen):
    - Behandelt NUR Vorwaerts-/Rueckwaerts-Kippen (Pitch), KEIN seitliches
      Kippen (Roll) - genau das war beim Testen der juengste beobachtete
      Fehlerfall (Policy-Variante driftete seitlich), dieser Regler hilft
      dagegen NICHT. Waere eine naheliegende naechste Erweiterung.
    - Die Korrektur wird hart begrenzt (MAX_KORREKTUR), um keine
      unsinnigen Gelenkwinkel zu erzeugen. Beim alten R1-Regler fuehrte
      genau diese Saettigung bei groesseren Kippwinkeln zu einem
      exponentiellen Kollaps statt einer sauberen Regelung - das ist
      hier nicht geloest, nur die Form der Rueckfuehrung ist anders.
      Fuer groessere Stoerungen braeuchte es einen Schritt (Capture Point/
      Stepping-Strategie), der hier NICHT implementiert ist.
    - kp und die Naeherung fuer l (Pendellaenge) sind NICHT empirisch
      getunt/gemessen, sondern erste, plausible Schaetzwerte - vor
      Verlass darauf im echten Testlauf beobachten und anpassen (siehe
      kps/kds in g1_policy.yaml fuer dasselbe Caveat bei anderen Werten
      in diesem Projekt).
    - Vorzeichen von pitch/pitch_rate relativ zu MuJoCos Quaternion-/
      Drehraten-Konvention sind aus derselben Methode wie
      RLLocomotionController.get_gravity_orientation() abgeleitet, aber
      fuer DIESEN konkreten Anwendungsfall (Korrekturrichtung) noch nicht
      im echten Testlauf verifiziert - falls der Roboter mit dieser
      Korrektur STAERKER statt schwaecher kippt, ist das Vorzeichen von
      KORREKTUR_VORZEICHEN unten umzudrehen.
    """

    # Pendellaenge-Naeherung: Hoehe Knoechel -> Schwerpunkt im Stehen.
    # Grobe Schaetzung (nicht am Modell gemessen), siehe Einschraenkungen
    # oben.
    PENDEL_LAENGE_M = 0.55
    G = 9.81

    # Verstaerkung der P-Rueckfuehrung auf die Knoechel-Pitch-Gelenke.
    # GEAENDERT (11.10.2026, nach echtem Testlauf): urspruenglicher
    # Schaetzwert war 1.5 - der Roboter hat dabei nicht einfach stabil
    # gestanden, sondern ist ueber die aufrechte Position hinausgeschossen
    # und nach HINTEN gekippt (vorher, ganz ohne Korrektur, kippte er nach
    # VORNE) - klares Zeichen fuer Ueberschwingen durch zu hohe
    # Verstaerkung, nicht fuer ein falsches Vorzeichen. Deutlich reduziert
    # als naechster Tuning-Schritt - weiterhin nicht endgueltig getunt.
    KP_KNOECHEL = 0.4
    # Die Huefte bekommt nur einen Teil derselben Korrektur (typische
    # Aufteilung bei Ankle+Hip-Strategien: Knoechel traegt bei kleinen
    # Stoerungen den Hauptanteil, Huefte unterstuetzt nur).
    HUEFTE_ANTEIL = 0.5

    # Harte Begrenzung der Korrektur in rad, um keine unsinnigen
    # Gelenkwinkel zu erzeugen (siehe Einschraenkungen oben zur
    # Saettigungs-Problematik). Zusammen mit KP_KNOECHEL reduziert, um
    # das beobachtete Ueberschwingen zu entschaerfen.
    MAX_KORREKTUR = 0.15

    # Gelenkindizes in der 12-DOF-Reihenfolge aus g1_policy.yaml
    # (default_angles: [hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch,
    # ankle_roll] x2, links dann rechts - siehe rl_locomotion_controller.py).
    IDX_HUEFTE_PITCH_LINKS = 0
    IDX_KNOECHEL_PITCH_LINKS = 4
    IDX_HUEFTE_PITCH_RECHTS = 6
    IDX_KNOECHEL_PITCH_RECHTS = 10

    def __init__(self):
        self._stability_time_const = np.sqrt(self.PENDEL_LAENGE_M / self.G)

    @staticmethod
    def _get_gravity_orientation(quaternion: np.ndarray) -> np.ndarray:
        # Identische Methode wie RLLocomotionController.get_gravity_orientation()
        # (bewusst hier noch einmal eigenstaendig implementiert statt
        # importiert, damit dieser Regler nicht von der RL-Controller-
        # Klasse abhaengt - beide Regler sollen unabhaengig austauschbar
        # bleiben, siehe IBalanceController/ILocomotionController).
        qw, qx, qy, qz = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        gravity_orientation = np.zeros(3, dtype=np.float32)
        gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
        gravity_orientation[1] = -2 * (qz * qy + qw * qx)
        gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)
        return gravity_orientation

    def compute_joint_targets(
        self,
        joint_positions: np.ndarray,
        joint_velocities: np.ndarray,
        base_quat: np.ndarray,
        base_ang_vel: np.ndarray,
        default_pose: np.ndarray
    ) -> np.ndarray:
        gravity_orientation = self._get_gravity_orientation(base_quat)

        # Pitch-Kippwinkel aus der projizierten Schwerkraftrichtung im
        # Koerper-KS (siehe Docstring oben zum Vorzeichen-Caveat).
        pitch = float(np.arctan2(gravity_orientation[0], gravity_orientation[2]))
        # Pitch-Rate: Annahme, dass base_ang_vel[1] die Drehrate um die
        # Pitch-Achse ist (gleiche Konvention wie imu.angular_velocity in
        # mujoco_node.py, dort direkt aus qvel[3:6] uebernommen) - ebenfalls
        # nicht separat verifiziert.
        pitch_rate = float(base_ang_vel[1])

        stability_value = pitch + self._stability_time_const * pitch_rate
        korrektur = -self.KP_KNOECHEL * stability_value
        korrektur = float(np.clip(korrektur, -self.MAX_KORREKTUR, self.MAX_KORREKTUR))

        target_positions = default_pose.copy()
        target_positions[self.IDX_KNOECHEL_PITCH_LINKS] += korrektur
        target_positions[self.IDX_KNOECHEL_PITCH_RECHTS] += korrektur
        target_positions[self.IDX_HUEFTE_PITCH_LINKS] += korrektur * self.HUEFTE_ANTEIL
        target_positions[self.IDX_HUEFTE_PITCH_RECHTS] += korrektur * self.HUEFTE_ANTEIL

        return target_positions
