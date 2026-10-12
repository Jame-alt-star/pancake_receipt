import numpy as np
import torch
from roboter_steuerung.control.i_locomotion_controller import ILocomotionController


class RLLocomotionController(ILocomotionController):
    """
    Kapselt das neuronale Netz (motion.pt, TorchScript) und die komplette
    mathematische Aufbereitung des 47-dimensionalen Observation-Vektors,
    mit dem dieses Netz trainiert wurde. Reine Reglerlogik (Layer 3) -
    kennt weder ROS2 noch MuJoCo, bekommt alle Sensordaten als einfache
    numpy-Arrays uebergeben (siehe steuereinheit_node.py) und gibt nur
    Ziel-Gelenkwinkel zurueck.
    """

    def __init__(self, config: dict):
        self.config = config
        
        # Policy laden. eval() schaltet Trainings-spezifisches Verhalten
        # ab (z.B. Dropout) - hier wird nur noch Inferenz gefahren, nie
        # mehr trainiert.
        self.policy = torch.jit.load(config["policy_path"])
        self.policy.eval()

        # Skalierungen & Parameter direkt aus g1_policy.yaml - muessen
        # exakt zu den Werten passen, mit denen die Policy trainiert
        # wurde (siehe Kommentare dort), sonst "verstehen" sich Training
        # und Deployment nicht mehr.
        self.kps = np.array(config["kps"], dtype=np.float32)
        self.kds = np.array(config["kds"], dtype=np.float32)
        self.default_angles = np.array(config["default_angles"], dtype=np.float32)
        
        self.ang_vel_scale = float(config["ang_vel_scale"])
        self.dof_pos_scale = float(config["dof_pos_scale"])
        self.dof_vel_scale = float(config["dof_vel_scale"])
        self.action_scale = float(config["action_scale"])
        self.cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)
        
        self.num_actions = int(config["num_actions"])
        self.num_obs = int(config["num_obs"])
        self.simulation_dt = float(config["simulation_dt"])
        self.control_decimation = int(config["control_decimation"])

        # Puffer, die zwischen zwei Aufrufen von compute_joint_targets()
        # erhalten bleiben muessen:
        # - self.action: die letzte Netz-Ausgabe. Wird selbst wieder Teil
        #   des naechsten Observation-Vektors (siehe unten) - die Policy
        #   wurde so trainiert, dass sie ihre eigene letzte Aktion "kennt",
        #   das macht die Bewegung glatter/konsistenter.
        # - self.target_dof_pos: Startwert = Ruhepose, bevor die erste
        #   echte Inferenz gelaufen ist.
        # - self.counter/self.period: siehe Phasenberechnung unten -
        #   period=0.8s ist die angenommene Dauer eines Schrittzyklus,
        #   mit der die Policy trainiert wurde (nicht frei waehlbar).
        self.action = np.zeros(self.num_actions, dtype=np.float32)
        self.target_dof_pos = self.default_angles.copy()
        self.obs = np.zeros(self.num_obs, dtype=np.float32)
        
        self.counter = 0
        self.period = 0.8

    @staticmethod
    def get_gravity_orientation(quaternion: np.ndarray) -> np.ndarray:
        """
        Rechnet die globale Schwerkraftrichtung in das lokale Koerper-
        Koordinatensystem um (Projektion ueber die Orientierungs-
        Quaternion). Ergebnis ist kein Winkel, sondern ein 3D-Vektor -
        "wo unten ist" aus Sicht des Roboters -, das ist fuer die Policy
        aussagekraeftiger als z.B. Euler-Winkel. Stammt (wie pd_control
        in mujoco_node.py) aus unitree_rl_gym's deploy_mujoco.py.
        """
        qw = quaternion[0]
        qx = quaternion[1]
        qy = quaternion[2]
        qz = quaternion[3]

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
        cmd: np.ndarray
    ) -> np.ndarray:
        """
        Baut aus den aktuellen Sensordaten den 47-dimensionalen
        Observation-Vektor auf (exakt das Format, mit dem motion.pt
        trainiert wurde) und laesst die Policy daraus die naechsten
        Ziel-Gelenkwinkel berechnen.
        """
        # Eingaben skalieren - nicht optional, sondern Teil des trainierten
        # Formats (siehe *_scale-Werte in g1_policy.yaml). qj/dqj sind
        # Gelenkposition/-geschwindigkeit relativ zur Ruhepose, nicht
        # die absoluten MuJoCo-Rohwerte.
        qj = (joint_positions - self.default_angles) * self.dof_pos_scale
        dqj = joint_velocities * self.dof_vel_scale
        gravity_orientation = self.get_gravity_orientation(base_quat)
        omega = base_ang_vel * self.ang_vel_scale

        # Takt-Phasensignal: wo im (angenommenen) Schrittzyklus von 0.8s
        # befinden wir uns gerade. sin/cos statt eines einzelnen
        # Phasenwerts, damit der Uebergang von Ende zu Anfang des Zyklus
        # keinen Sprung im Eingabewert erzeugt (0.99 -> 0.0 waere sonst
        # ein harter Sprung; sin/cos sind an dieser Stelle stetig) - das
        # gibt der Policy ein Zeitgefuehl fuer Schwung-/Standphase der
        # Beine, ohne eine explizite Zustandsmaschine zu brauchen.
        count = self.counter * self.simulation_dt
        phase = (count % self.period) / self.period
        sin_phase = np.sin(2 * np.pi * phase)
        cos_phase = np.cos(2 * np.pi * phase)

        # Observation-Vektor (47 Werte) befuellen, in exakt der Reihenfolge,
        # mit der trainiert wurde: 3 Drehrate + 3 Schwerkraftrichtung +
        # 3 Geschwindigkeitskommando + 12 Gelenkpositionen + 12 Gelenk-
        # geschwindigkeiten + 12 letzte Aktion + 2 Phase (sin/cos) = 47.
        # self.num_actions wird hier verwendet statt hartkodierter 12 -
        # passt sich automatisch an, falls num_actions in der YAML mal
        # geaendert wird.
        self.obs[:3] = omega
        self.obs[3:6] = gravity_orientation
        self.obs[6:9] = cmd * self.cmd_scale
        self.obs[9 : 9 + self.num_actions] = qj
        self.obs[9 + self.num_actions : 9 + 2 * self.num_actions] = dqj
        self.obs[9 + 2 * self.num_actions : 9 + 3 * self.num_actions] = self.action
        self.obs[9 + 3 * self.num_actions : 9 + 3 * self.num_actions + 2] = np.array([sin_phase, cos_phase])

        # Inferenz via TorchScript: unsqueeze(0) fuegt eine Batch-
        # Dimension hinzu (das Netz erwartet [Batch, 47], wir haben aber
        # nur einen einzelnen Zustand -> Batch-Groesse 1). no_grad(), weil
        # hier nur vorhergesagt und nicht trainiert wird - spart Speicher/
        # Rechenzeit, da keine Gradienten mitgefuehrt werden muessen.
        # squeeze() entfernt die Batch-Dimension wieder aus dem Ergebnis.
        obs_tensor = torch.from_numpy(self.obs).float().unsqueeze(0)
        with torch.no_grad():
            self.action = self.policy(obs_tensor).detach().numpy().squeeze()

        # Die rohe Netz-Ausgabe (self.action) ist keine Winkel-Angabe,
        # sondern eine kleine Korrektur um die Ruhepose - action_scale
        # begrenzt, wie stark die Policy von der Ruhepose abweichen darf.
        self.target_dof_pos = self.action * self.action_scale + self.default_angles
        # counter um control_decimation erhoehen (nicht um 1), weil diese
        # Funktion nur alle control_decimation Physik-Schritte aufgerufen
        # wird (50 Hz Regeltakt bei 500 Hz Physik) - die Phase muss trotzdem
        # in echter Simulationszeit weiterlaufen.
        self.counter += self.control_decimation

        return self.target_dof_pos.copy()
