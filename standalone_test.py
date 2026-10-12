import os
import time
import numpy as np
import torch
import yaml
import mujoco
import mujoco.viewer

WS = os.path.expanduser("~/humanoid_robot/src/r1_bringup")

with open(os.path.join(WS, "config/g1_policy.yaml")) as f:
    cfg = yaml.load(f, Loader=yaml.FullLoader)

kps = np.array(cfg["kps"], dtype=np.float32)
kds = np.array(cfg["kds"], dtype=np.float32)
default_angles = np.array(cfg["default_angles"], dtype=np.float32)
sim_dt = float(cfg["simulation_dt"])
control_decimation = int(cfg["control_decimation"])
ang_vel_scale = float(cfg["ang_vel_scale"])
dof_pos_scale = float(cfg["dof_pos_scale"])
dof_vel_scale = float(cfg["dof_vel_scale"])
action_scale = float(cfg["action_scale"])
cmd_scale = np.array(cfg["cmd_scale"], dtype=np.float32)
num_actions = int(cfg["num_actions"])
num_obs = int(cfg["num_obs"])
cmd = np.array(cfg.get("cmd_init", [0.5, 0, 0]), dtype=np.float32)

policy = torch.jit.load(os.path.join(WS, cfg["policy_path"]))
policy.eval()

model = mujoco.MjModel.from_xml_path(os.path.join(WS, cfg["xml_path"]))
data = mujoco.MjData(model)
model.opt.timestep = sim_dt
data.qpos[7:] = default_angles
mujoco.mj_forward(model, data)


def get_gravity_orientation(q):
    qw, qx, qy, qz = q
    g = np.zeros(3, dtype=np.float32)
    g[0] = 2 * (-qz * qx + qw * qy)
    g[1] = -2 * (qz * qy + qw * qx)
    g[2] = 1 - 2 * (qw * qw + qz * qz)
    return g


target_dof_pos = default_angles.copy()
action = np.zeros(num_actions, dtype=np.float32)
obs = np.zeros(num_obs, dtype=np.float32)
counter = 0
period = 0.8

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
        q = data.qpos[7:]
        dq = data.qvel[6:]
        tau = (target_dof_pos - q) * kps + (0 - dq) * kds
        data.ctrl[:] = tau
        mujoco.mj_step(model, data)
        counter += 1

        if counter % control_decimation == 0:
            qj = (data.qpos[7:] - default_angles) * dof_pos_scale
            dqj = data.qvel[6:] * dof_vel_scale
            quat = data.qpos[3:7]
            omega = data.qvel[3:6] * ang_vel_scale
            gravity_orientation = get_gravity_orientation(quat)

            count = counter * sim_dt
            phase = (count % period) / period
            sin_p, cos_p = np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)

            obs[:3] = omega
            obs[3:6] = gravity_orientation
            obs[6:9] = cmd * cmd_scale
            obs[9:9 + num_actions] = qj
            obs[9 + num_actions:9 + 2 * num_actions] = dqj
            obs[9 + 2 * num_actions:9 + 3 * num_actions] = action
            obs[9 + 3 * num_actions:9 + 3 * num_actions + 2] = [sin_p, cos_p]

            obs_tensor = torch.from_numpy(obs).float().unsqueeze(0)
            with torch.no_grad():
                action = policy(obs_tensor).detach().numpy().squeeze()
            target_dof_pos = action * action_scale + default_angles

        viewer.sync()
        time_until_next = model.opt.timestep - (time.time() - step_start)
        if time_until_next > 0:
            time.sleep(time_until_next)
