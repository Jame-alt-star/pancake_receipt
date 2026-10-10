import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from r1_bringup.bridges.mujoco_bridge import MujocoBridge
from roboter_steuerung.control.rl_locomotion_controller import RLLocomotionController


class SteuereinheitNode(Node):
    def __init__(self):
        super().__init__("steuereinheit_node")

        # 1. Konfiguration laden
        bringup_share = get_package_share_directory("r1_bringup")
        config_path = os.path.join(bringup_share, "config", "g1_policy.yaml")
        with open(config_path, "r") as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

        # 2. Hardware-Bridge und RL-Controller initialisieren
        self.bridge = MujocoBridge(self)
        self.controller = RLLocomotionController(self.config)

        # 3. Startbefehl aus YAML
        self.cmd = np.array(self.config.get("cmd_init", [0.5, 0.0, 0.0]), dtype=np.float32)

        # 4. Regelungsschleife takten (50 Hz / alle 20 ms)
        timer_period = float(self.config["simulation_dt"]) * int(self.config["control_decimation"])
        self.timer = self.create_timer(timer_period, self._control_loop)
        self.get_logger().info("Steuereinheit erfolgreich gestartet (50 Hz Regelkreis aktiv).")

    def _control_loop(self):
        quat, omega = self.bridge.read_imu()
        joint_pos, joint_vel = self.bridge.read_joint_states()

        if np.all(joint_pos == 0.0) and np.all(quat == 0.0):
            return

        target_positions = self.controller.compute_joint_targets(
            joint_positions=joint_pos,
            joint_velocities=joint_vel,
            base_quat=quat,
            base_ang_vel=omega,
            cmd=self.cmd,
            sim_time=self.bridge.get_sim_time()
        )

        self.bridge.send_joint_commands(target_positions)


def main(args=None):
    rclpy.init(args=args)
    node = SteuereinheitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()