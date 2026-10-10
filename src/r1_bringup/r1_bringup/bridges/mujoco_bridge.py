from typing import Tuple
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray

from r1_bringup.bridges.i_hardware_bridge import IRobotHardwareBridge


class MujocoBridge(IRobotHardwareBridge):
    def __init__(self, node: Node):
        self.node = node

        self.last_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.last_omega = np.zeros(3, dtype=np.float32)
        self.last_joint_pos = np.zeros(12, dtype=np.float32)
        self.last_joint_vel = np.zeros(12, dtype=np.float32)
        self.current_sim_time = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_pub = self.node.create_publisher(Float32MultiArray, "/joint_commands", 1)
        self.node.create_subscription(JointState, "/joint_states", self._joint_callback, qos)
        self.node.create_subscription(Imu, "/imu", self._imu_callback, qos)
    ## Das ist der callback der Gelenkzustände, welcher 
    def _joint_callback(self, msg: JointState):
        if len(msg.position) > 0:
            self.last_joint_pos = np.array(msg.position, dtype=np.float32)
            self.last_joint_vel = np.array(msg.velocity, dtype=np.float32)
            self.current_sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _imu_callback(self, msg: Imu):
        self.last_quat = np.array(
            [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
            dtype=np.float32
        )
        self.last_omega = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            dtype=np.float32
        )

    def read_imu(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.last_quat.copy(), self.last_omega.copy()

    def read_joint_states(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.last_joint_pos.copy(), self.last_joint_vel.copy()

    def get_sim_time(self) -> float:
        return self.current_sim_time

    def send_joint_commands(self, target_positions: np.ndarray) -> None:
        msg = Float32MultiArray()
        msg.data = target_positions.tolist()
        self.cmd_pub.publish(msg)