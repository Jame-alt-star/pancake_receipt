import os
import numpy as np
import torch
from ament_index_python.packages import get_package_share_directory
from roboter_steuerung.control.i_locomotion_controller import ILocomotionController


class RLLocomotionController(ILocomotionController):
    """
    Kapselt das neuronale Netz (motion.pt) und die mathematische
    Aufbereitung des 47-dimensionalen Observation-Vektors.
    """

    def __init__(self, config: dict):
        self.config = config

        # 1. Pfad zur Policy absichern
        policy_path = config["policy_path"]
        if not os.path.exists(policy_path):
            pkg_share = get_package_share_directory("r1_bringup")
            policy_path = os.path.join(pkg_share, policy_path)
            if not os.path.exists(policy_path):
                policy_path = os.path.expanduser(f"~/humanoid_robot/src/r1_bringup/{config['policy_path']}")

        # 2. PyTorch Policy laden
        self.policy = torch.jit.load(policy_path)
        self.policy.eval()

        # 3. Regelungsgains & Gelenk-Winkel
        self.kps = np.array(config["kps"], dtype=np.float32)
        self.kds = np.array(config["kds"], dtype=np.float32)
        self.default_angles = np.array(config["default_angles"], dtype=np.float32)

        # 4. Skalierungsfaktoren
        self.ang_vel_scale = float(config["ang_vel_scale"])
        self.dof_pos_scale = float(config["dof_pos_scale"])
        self.dof_vel_scale = float(config["dof_vel_scale"])
        self.action_scale = float(config["action_scale"])
        self.cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)

        # 5. Dimensionen & Taktung
        self.num_actions = int(config["num_actions"])
        self.num_obs = int(config["num_obs"])
        self.simulation_dt = float(config["simulation_dt"])
        self.control_decimation = int(config["control_decimation"])

        # 6. Puffer-Arrays
        self.action = np.zeros(self.num_actions, dtype=np.float32)
        self.target_dof_pos = self.default_angles.copy()
        self.obs = np.zeros(self.num_obs, dtype=np.float32)

        self.counter = 0
        self.period = 0.8

    @staticmethod
    def get_gravity_orientation(quaternion: np.ndarray) -> np.ndarray:
        """Projiziert die Orientierung in den Schwerkraftvektor (aus deploy_mujoco.py Zeile 10-23)."""
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
        cmd: np.ndarray,
        sim_time: float = None
    ) -> np.ndarray:
        """
        Baut den 47-dimensionalen Observation-Vektor auf und berechnet über die Policy
        die Zielwinkel für die Gelenke.
        """
        qj = (joint_positions - self.default_angles) * self.dof_pos_scale
        dqj = joint_velocities * self.dof_vel_scale
        gravity_orientation = self.get_gravity_orientation(base_quat)
        omega = base_ang_vel * self.ang_vel_scale

        # Exakte Physikzeit für die Phasenberechnung nutzen
        if sim_time is not None and sim_time > 0.0:
            count = sim_time
        else:
            count = self.counter * self.simulation_dt

        phase = (count % self.period) / self.period
        sin_phase = np.sin(2 * np.pi * phase)
        cos_phase = np.cos(2 * np.pi * phase)

        # Observation-Vektor befüllen
        self.obs[:3] = omega
        self.obs[3:6] = gravity_orientation
        self.obs[6:9] = cmd * self.cmd_scale
        self.obs[9 : 9 + self.num_actions] = qj
        self.obs[9 + self.num_actions : 9 + 2 * self.num_actions] = dqj
        self.obs[9 + 2 * self.num_actions : 9 + 3 * self.num_actions] = self.action
        self.obs[9 + 3 * self.num_actions : 9 + 3 * self.num_actions + 2] = np.array([sin_phase, cos_phase])

        # Inferenz via TorchScript
        obs_tensor = torch.from_numpy(self.obs).float().unsqueeze(0)
        with torch.no_grad():
            self.action = self.policy(obs_tensor).detach().numpy().squeeze()

        self.target_dof_pos = self.action * self.action_scale + self.default_angles
        self.counter += self.control_decimation

        return self.target_dof_pos.copy()