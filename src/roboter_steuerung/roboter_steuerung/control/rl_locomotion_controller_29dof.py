from collections import deque

import numpy as np
import onnxruntime as ort

from roboter_steuerung.control.i_locomotion_controller import ILocomotionController


# Gelenkreihenfolge wie MuJoCo/g1_29dof.xml sie deklariert (<motor>-
# Reihenfolge, identisch zur Unitree-SDK-LowCmd-Reihenfolge). Das ist die
# Reihenfolge, in der joint_positions/joint_velocities hier ankommen
# (mujoco_bridge.py liest /joint_states direkt aus self.data.qpos[7:]) und
# in der compute_joint_targets() seine Rueckgabe liefern MUSS (mujoco_node.py
# wendet target_dof_pos 1:1 auf self.data.ctrl[:] an, selbe Reihenfolge).
MUJOCO_JOINT_ORDER = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

# Gelenkreihenfolge, in der die ONNX-Policy selbst rechnet (Isaac-Lab-
# interne Reihenfolge - NICHT identisch zu MUJOCO_JOINT_ORDER!). Hergeleitet
# (nicht geraten) aus unitree_rl_lab's deploy.yaml: joint_ids_map gibt fuer
# jedes Policy-Gelenk i die Position in MUJOCO_JOINT_ORDER an
# (joint_ids_map[i] = Position), Richtung ueber die numpy-Fancy-Indexing-
# Semantik in export_deploy_cfg.py bestaetigt (stiffness[joint_ids_map] =
# asset_data_isaac_order) und gegen ALLE 29 Werte aus deploy.yaml's
# default_joint_pos gegengeprueft (29/29 Treffer), nicht nur stichprobenhaft.
# Siehe statusupdate_g1_projekt.md, Abschnitt "DOF-Migration" fuer die
# vollstaendige Herleitung.
POLICY_JOINT_ORDER = [
    "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint",
    "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
    "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint",
    "left_knee_joint", "right_knee_joint",
    "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
    "left_ankle_pitch_joint", "right_ankle_pitch_joint",
    "left_shoulder_roll_joint", "right_shoulder_roll_joint",
    "left_ankle_roll_joint", "right_ankle_roll_joint",
    "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
    "left_elbow_joint", "right_elbow_joint",
    "left_wrist_roll_joint", "right_wrist_roll_joint",
    "left_wrist_pitch_joint", "right_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_wrist_yaw_joint",
]

# joint_ids_map aus unitree_rl_lab's deploy.yaml, UNVERAENDERT uebernommen
# (nicht selbst neu berechnet - direkte Quelle, siehe oben).
# JOINT_IDS_MAP[i] = Position von POLICY_JOINT_ORDER[i] in
# MUJOCO_JOINT_ORDER. Zwei Verwendungen in compute_joint_targets():
#   - Gather  (MuJoCo -> Policy): policy_arr = mujoco_arr[JOINT_IDS_MAP]
#   - Scatter (Policy -> MuJoCo): mujoco_arr[JOINT_IDS_MAP] = policy_arr
JOINT_IDS_MAP = np.array(
    [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16, 23, 5, 11,
     17, 24, 18, 25, 19, 26, 20, 27, 21, 28],
    dtype=np.int64,
)


class RLLocomotionController29Dof(ILocomotionController):
    """
    ONNX-Variante von RLLocomotionController (siehe rl_locomotion_controller.py)
    fuer Unitree's vortrainierte 29-DOF-Ganzkoerper-Policy aus unitree_rl_lab
    (Isaac-Lab-basiert), statt der 12-DOF-nur-Beine-TorchScript-Policy aus
    unitree_rl_gym. Gleiche Rolle/Schnittstelle (ILocomotionController,
    Layer 3) - austauschbar mit RLLocomotionController, Auswahl ueber
    controller_type in der jeweiligen g1_policy*.yaml (siehe
    steuereinheit_node.py). Der 12-DOF-Pfad bleibt unveraendert bestehen -
    diese Klasse ist eine Ergaenzung, kein Ersatz, solange sie nicht per
    Config ausgewaehlt wird.

    Zwei grundlegende Unterschiede zur 12-DOF-Variante:

    1. Die Policy wurde mit Isaac Lab trainiert, das Gelenke in einer
       ANDEREN Reihenfolge erwartet als MuJoCo/die SDK (siehe
       POLICY_JOINT_ORDER/MUJOCO_JOINT_ORDER oben) - jede Ein-/Ausgabe muss
       deshalb explizit umsortiert werden (JOINT_IDS_MAP).

    2. Isaac Lab baut den Beobachtungsvektor nicht aus einem einzelnen
       Zeitschritt auf, sondern haelt pro Beobachtungs-Term einen eigenen
       Verlauf der letzten history_length Zeitschritte (aelteste zuerst) -
       siehe _history/TERM_ORDER unten, hergeleitet aus
       ObservationTermCfg::add()/get() in unitree_rl_lab
       (deploy/include/isaaclab/manager/manager_term_cfg.h): ein
       std::deque mit push_back()/pop_front() bei Ueberlauf, also FIFO mit
       aeltestem Eintrag vorne - get() konkateniert in genau dieser
       (Iterations-)Reihenfolge.
    """

    # Reihenfolge und Rolle der Beobachtungs-Terme wie in deploy.yaml
    # deklariert (Schluesselreihenfolge dort) - bestimmt die
    # Konkatenationsreihenfolge im Beobachtungsvektor. Zusammen mit
    # history_length=5 ergibt das 6 Terme * 5 Zeitschritte (3+3+3+29+29+29
    # = 96 Werte/Zeitschritt * 5 = 480, siehe num_obs in der YAML).
    TERM_ORDER = (
        "base_ang_vel", "projected_gravity", "velocity_commands",
        "joint_pos_rel", "joint_vel_rel", "last_action",
    )

    def __init__(self, config: dict):
        self.config = config

        # ONNX-Laufzeit statt TorchScript. CPUExecutionProvider reicht fuer
        # Inferenz bei 50 Hz voellig aus (keine GPU im Simulations-Setup
        # vorausgesetzt) - vermeidet zusaetzliche CUDA-Abhaengigkeiten.
        # Single-Thread + kein Spin-Waiting (gemessen 13.10.2026): mit den
        # onnxruntime-Defaults (alle Kerne, aktives Warten) belegte der
        # Steuerknoten ~270% CPU und nahm mujoco_node die Rechenzeit weg
        # (Simulation lief nur mit ~0.45x Echtzeit). Das Netz ist klein
        # genug fuer einen Thread bei 50 Hz.
        so = ort.SessionOptions()
        so.intra_op_num_threads = 1
        so.inter_op_num_threads = 1
        so.add_session_config_entry("session.intra_op.allow_spinning", "0")
        so.add_session_config_entry("session.inter_op.allow_spinning", "0")
        self.session = ort.InferenceSession(
            config["policy_path"], sess_options=so, providers=["CPUExecutionProvider"]
        )
        self._input_name = self.session.get_inputs()[0].name

        # Alle Arrays unten sind in MuJoCo-Gelenkreihenfolge (siehe
        # MUJOCO_JOINT_ORDER) - genau wie kps/kds/default_angles in
        # g1_policy_29dof.yaml bereits dort hinterlegt.
        self.kps = np.array(config["kps"], dtype=np.float32)
        self.kds = np.array(config["kds"], dtype=np.float32)
        self.default_angles = np.array(config["default_angles"], dtype=np.float32)

        self.ang_vel_scale = float(config["ang_vel_scale"])
        self.dof_pos_scale = float(config["dof_pos_scale"])
        self.dof_vel_scale = float(config["dof_vel_scale"])
        self.action_scale = float(config["action_scale"])

        self.num_actions = int(config["num_actions"])
        self.history_length = int(config.get("history_length", 5))

        # Letzte Netz-Ausgabe (Policy-Reihenfolge!) - wird selbst wieder
        # Teil des naechsten Beobachtungsvektors (last_action-Term), genau
        # wie bei der 12-DOF-Variante (dort self.action).
        self.last_action = np.zeros(self.num_actions, dtype=np.float32)
        # Ziel-Gelenkwinkel in MuJoCo-Reihenfolge - das ist, was
        # compute_joint_targets() zurueckgibt. Startwert = Ruhepose, bevor
        # die erste echte Inferenz gelaufen ist (gleiches Muster wie bei
        # RLLocomotionController.target_dof_pos).
        self.target_dof_pos = self.default_angles.copy()

        # Pro Term ein FIFO-Verlauf der letzten history_length Zeitschritte
        # (siehe Klassendoc Punkt 2). Wird beim ersten Aufruf von
        # compute_joint_targets() mit history_length Kopien des dann
        # aktuellen Werts vorbefuellt (siehe reset()-Analogie in
        # manager_term_cfg.h) statt mit Nullen - vermeidet einen
        # kuenstlichen Sprung aus Nullen in der allerersten Beobachtung.
        self._history = {name: deque(maxlen=self.history_length) for name in self.TERM_ORDER}
        self._initialized = False

    @staticmethod
    def get_gravity_orientation(quaternion: np.ndarray) -> np.ndarray:
        # Identisch zu RLLocomotionController.get_gravity_orientation() -
        # gleiche Herkunft (unitree_rl_gym deploy_mujoco.py), unabhaengig
        # von Gelenkanzahl/-reihenfolge (arbeitet nur auf der Basis-
        # Orientierung).
        qw, qx, qy, qz = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        g = np.zeros(3, dtype=np.float32)
        g[0] = 2 * (-qz * qx + qw * qy)
        g[1] = -2 * (qz * qy + qw * qx)
        g[2] = 1 - 2 * (qw * qw + qz * qz)
        return g

    def _build_obs(self) -> np.ndarray:
        # Aussen nach TERM_ORDER, innen je Term die History aelteste-zuerst
        # (deque-Iterationsreihenfolge entspricht das bereits von selbst:
        # append() haengt ans Ende an, maxlen=history_length verwirft beim
        # naechsten append() automatisch den aeltesten/vordersten Eintrag -
        # siehe Klassendoc Punkt 2).
        parts = []
        for name in self.TERM_ORDER:
            for frame in self._history[name]:
                parts.append(frame)
        return np.concatenate(parts).astype(np.float32)

    def compute_joint_targets(
        self,
        joint_positions: np.ndarray,
        joint_velocities: np.ndarray,
        base_quat: np.ndarray,
        base_ang_vel: np.ndarray,
        cmd: np.ndarray,
    ) -> np.ndarray:
        """
        Baut aus den aktuellen Sensordaten den 480-dimensionalen
        Beobachtungsvektor auf (exakt das Format, mit dem policy_29dof.onnx
        trainiert wurde - siehe Klassendoc) und laesst die Policy daraus die
        naechsten Ziel-Gelenkwinkel berechnen.
        """
        # 1. Position/Geschwindigkeit relativ zur Ruhepose skalieren (noch
        # in MuJoCo-Reihenfolge), dann per Gather (JOINT_IDS_MAP) in
        # Policy-Reihenfolge umsortieren - alles Weitere rechnet in
        # Policy-Reihenfolge, exakt wie beim Training.
        qj_mujoco = (joint_positions - self.default_angles) * self.dof_pos_scale
        dqj_mujoco = joint_velocities * self.dof_vel_scale
        qj_policy = qj_mujoco[JOINT_IDS_MAP]
        dqj_policy = dqj_mujoco[JOINT_IDS_MAP]

        gravity = self.get_gravity_orientation(base_quat)
        omega = base_ang_vel * self.ang_vel_scale

        # velocity_commands hat scale=1.0 in deploy.yaml - cmd kommt bereits
        # in "physikalischen" Einheiten (m/s, m/s, rad/s) von
        # steuereinheit_node.py (node.cmd), keine weitere Skalierung noetig
        # (anders als cmd_scale bei der 12-DOF-Variante).
        term_values = {
            "base_ang_vel": omega,
            "projected_gravity": gravity,
            "velocity_commands": np.asarray(cmd, dtype=np.float32),
            "joint_pos_rel": qj_policy,
            "joint_vel_rel": dqj_policy,
            "last_action": self.last_action,
        }

        if not self._initialized:
            # Erstaufruf: jede History mit history_length Kopien des
            # aktuellen Werts vorbefuellen (siehe __init__-Kommentar).
            for name, value in term_values.items():
                for _ in range(self.history_length):
                    self._history[name].append(value.astype(np.float32).copy())
            self._initialized = True
        else:
            for name, value in term_values.items():
                self._history[name].append(value.astype(np.float32).copy())

        obs = self._build_obs()

        # 2. Inferenz (ONNX statt TorchScript). Policy erwartet [Batch, 480] -
        # Batch-Groesse 1, wie bei der 12-DOF-Variante (dort unsqueeze(0)).
        outputs = self.session.run(None, {self._input_name: obs[np.newaxis, :]})
        action_policy = np.asarray(outputs[0], dtype=np.float32).squeeze()
        self.last_action = action_policy.copy()

        # 3. Policy-Ausgabe ist wieder in Policy-Gelenkreihenfolge - per
        # Scatter (JOINT_IDS_MAP) zurueck nach MuJoCo-Reihenfolge
        # umsortieren, BEVOR die Ruhepose (default_angles, MuJoCo-
        # Reihenfolge!) aufaddiert wird - wie bei der 12-DOF-Variante ist
        # die rohe Netz-Ausgabe keine absolute Winkel-Angabe, sondern eine
        # kleine Korrektur um die Ruhepose (action_scale begrenzt die
        # maximale Abweichung).
        action_mujoco = np.zeros(self.num_actions, dtype=np.float32)
        action_mujoco[JOINT_IDS_MAP] = action_policy

        self.target_dof_pos = action_mujoco * self.action_scale + self.default_angles
        return self.target_dof_pos.copy()
