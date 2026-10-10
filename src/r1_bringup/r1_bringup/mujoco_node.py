import os
import threading
import time
import yaml
import numpy as np

import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray
from builtin_interfaces.msg import Time
from ament_index_python.packages import get_package_share_directory


def pd_control(target_q, q, kp, target_dq, dq, kd):
    return (target_q - q) * kp + (target_dq - dq) * kd


class MujocoBridgeNode(Node):
    def __init__(self):
        super().__init__("mujoco_bridge_node")

        pkg_share = get_package_share_directory("r1_bringup")
        config_path = os.path.join(pkg_share, "config", "g1_policy.yaml")
        with open(config_path, "r") as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

        xml_rel_path = self.config["xml_path"]
        xml_path = os.path.join(pkg_share, xml_rel_path)
        if not os.path.exists(xml_path):
            xml_path = os.path.expanduser(f"~/humanoid_robot/src/r1_bringup/{xml_rel_path}")

        self.simulation_dt = float(self.config["simulation_dt"])
        self.kps = np.array(self.config["kps"], dtype=np.float32)
        self.kds = np.array(self.config["kds"], dtype=np.float32)
        self.default_angles = np.array(self.config["default_angles"], dtype=np.float32)
        self.target_dof_pos = self.default_angles.copy()

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.simulation_dt

        # Grundstellung einnehmen
        self.data.qpos[7:] = self.default_angles
        mujoco.mj_forward(self.model, self.data)

        # Schnelle Sensor-QoS ohne Zwischenspeicher-Latenz
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub_joint_states = self.create_publisher(JointState, "/joint_states", qos)
        self.pub_imu = self.create_publisher(Imu, "/imu", qos)
        self.sub_joint_commands = self.create_subscription(
            Float32MultiArray,
            "/joint_commands",
            self._joint_commands_callback,
            1
        )

        self.controller_connected = False
        self.is_running = True
        self.sim_thread = threading.Thread(target=self._run_simulation)
        self.sim_thread.daemon = True
        self.sim_thread.start()

    def _joint_commands_callback(self, msg: Float32MultiArray):
        cmd_data = np.array(msg.data, dtype=np.float32)
        if len(cmd_data) == len(self.target_dof_pos):
            self.target_dof_pos = cmd_data
            self.controller_connected = True

    def _run_simulation(self):
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running() and self.is_running:
                step_start = time.time()

                self._publish_states()

                if not self.controller_connected:
                    self.data.qpos[7:] = self.default_angles
                    self.data.qvel[:] = 0.0
                    mujoco.mj_forward(self.model, self.data)
                    viewer.sync()
                    time.sleep(0.01)
                    continue

                q = self.data.qpos[7:]
                dq = self.data.qvel[6:]
                tau = pd_control(
                    self.target_dof_pos, q, self.kps, np.zeros_like(self.kds), dq, self.kds
                )
                self.data.ctrl[:] = tau
                mujoco.mj_step(self.model, self.data)

                viewer.sync()

                time_until_next = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next > 0:
                    time.sleep(time_until_next)

    def _publish_states(self):
        # Exakte Simulationszeit für fehlerfreie Phasenberechnung mitsenden
        sim_sec = int(self.data.time)
        sim_nanosec = int((self.data.time - sim_sec) * 1e9)
        stamp = Time(sec=sim_sec, nanosec=sim_nanosec)

        js = JointState()
        js.header.stamp = stamp
        js.position = self.data.qpos[7:].tolist()
        js.velocity = self.data.qvel[6:].tolist()
        self.pub_joint_states.publish(js)

        imu = Imu()
        imu.header.stamp = stamp
        imu.orientation.w = float(self.data.qpos[3])
        imu.orientation.x = float(self.data.qpos[4])
        imu.orientation.y = float(self.data.qpos[5])
        imu.orientation.z = float(self.data.qpos[6])
        imu.angular_velocity.x = float(self.data.qvel[3])
        imu.angular_velocity.y = float(self.data.qvel[4])
        imu.angular_velocity.z = float(self.data.qvel[5])
        self.pub_imu.publish(imu)

    def destroy_node(self):
        self.is_running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MujocoBridgeNode()
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