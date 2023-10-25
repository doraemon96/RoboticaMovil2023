""" Written by Brian Hou for CSE571: Probabilistic Robotics (Winter 2019)
"""

import math
import numpy as np

from utils import minimized_angle


class ExtendedKalmanFilter:
    def __init__(self, mean, cov, alphas, beta):
        self.alphas = alphas
        self.beta = beta

        self._init_mean = mean
        self._init_cov = cov
        self.reset()

    def reset(self):
        self.mu = self._init_mean
        self.sigma = self._init_cov

    def update(self, env, u, z, marker_id):
        """Update the state estimate after taking an action and receiving a landmark
        observation.

        u: action
        z: landmark observation
        marker_id: landmark ID
        """
        G = env.G(self.mu, u)
        V = env.V(self.mu, u)
        M = env.noise_from_motion(u, self.alphas)
        _mu = env.forward(self.mu, u)
        _sigma = G @ self.sigma @ G.T + V @ M @ V.T
        Q = self.beta
        _x, _y, _theta = _mu.ravel()
        dmx = env.MARKER_X_POS[marker_id] - _x
        dmy = env.MARKER_Y_POS[marker_id] - _y
        q = (dmx)**2 + (dmy)**2
        z_hat = math.atan2(dmy, dmx) - _theta #np.array([math.sqrt(q), math.atan2(dmy, dmx) - _theta]).T
        H = env.H(_mu, marker_id)
        S = H @ _sigma @ H.T + Q
        K = (_sigma @ H.T) @ np.linalg.pinv(S)
        self.mu = _mu + K * (z - z_hat)
        self.sigma = (np.eye(3) - K @ H) @ _sigma
        return self.mu, self.sigma
