"""Realistische Sensorik der Simulation (Schalter realistic_sensors): Latenz, Rauschen, Gyro-Bias, Gier-Drift.

Die Werte sind ANNAHMEN in plausibler Groesse (typische Encoder/MEMS-IMU/DDS), nicht aus einem G1-Datenblatt
(config/simulation.yaml). Gemessen (synchroner Kreis): die Policy bleibt bis zum 10-Fachen dieser Werte plus
20 ms Latenz stabil.
"""
import math
from collections import deque

import numpy as np

from roboter_simulation.geometrie import quat_mul


class Sensormodell:

    def __init__(self, rng, n_q, n_dq, n_gyro, gyro_bias_std, n_ori, latenz_s, yaw_drift_rad_s):
        self.rng = rng
        self.n_q, self.n_dq, self.n_gyro, self.n_ori = n_q, n_dq, n_gyro, n_ori
        self.gyro_bias = rng.normal(0.0, gyro_bias_std, 3)     # konstant je Lauf (seed)
        self.latenz = latenz_s
        self.yaw_drift = yaw_drift_rad_s
        self._puffer = deque()                                 # (sim_zeit, q, dq, quat, omega)

    def anwenden(self, t, q, dq, quat, omega):
        """Messwerte zur Sim-Zeit t -> was ein echter Sensor jetzt liefern wuerde (latenz_s alt, verrauscht)."""
        self._puffer.append((t, q, dq, quat, omega))
        while len(self._puffer) > 1 and self._puffer[1][0] <= t - self.latenz:
            self._puffer.popleft()
        _, q, dq, quat, omega = self._puffer[0]
        r = self.rng
        q = q + r.normal(0.0, self.n_q, q.shape)
        dq = dq + r.normal(0.0, self.n_dq, dq.shape)
        omega = omega + self.gyro_bias + r.normal(0.0, self.n_gyro, 3)
        if self.yaw_drift:
            # Gier-Drift um die Welt-z-Achse (vorne multipliziert): q_drift * q
            h = 0.5 * self.yaw_drift * t
            quat = quat_mul((math.cos(h), 0.0, 0.0, math.sin(h)), quat)
        if self.n_ori > 0.0:
            v = r.normal(0.0, self.n_ori, 3)
            d = np.concatenate([[1.0], 0.5 * v])
            quat = quat_mul(quat, d / np.linalg.norm(d))
        return q, dq, quat, omega
