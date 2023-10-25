""" Written by Brian Hou for CSE571: Probabilistic Robotics (Winter 2019)
"""

import math
import numpy as np

from utils import minimized_angle


class ParticleFilter:
    def __init__(self, mean, cov, num_particles, alphas, beta):
        self.alphas = alphas
        self.beta = beta

        self._init_mean = mean
        self._init_cov = cov
        self.num_particles = num_particles
        self.reset()

    def reset(self):
        self.particles = np.zeros((self.num_particles, 3))
        for i in range(self.num_particles):
            self.particles[i, :] = np.random.multivariate_normal(
                self._init_mean.ravel(), self._init_cov)
        self.weights = np.ones(self.num_particles) / self.num_particles

    def update(self, env, u, z, marker_id):
        """Update the state estimate after taking an action and receiving a landmark
        observation.

        u: action
        z: landmark observation
        marker_id: landmark ID
        """
        for i, particle in enumerate(self.particles):
            particle = env.forward(particle, u)
            dmx = env.MARKER_X_POS[marker_id] - particle[0]
            dmy = env.MARKER_Y_POS[marker_id] - particle[1]
            z_hat = minimized_angle(math.atan2(dmy, dmx) - particle[2])
            weight = env.likelihood(z - z_hat, self.beta)
        
            self.particles[i,:] = particle[0]
            self.weights[i] = weight
        
        mean, cov = self.mean_and_variance(self.particles)
        return mean, cov

    def resample(self, particles, weights):
        """Sample new particles and weights given current particles and weights. Be sure
        to use the low-variance sampler from class.

        particles: (n x 3) matrix of poses
        weights: (n,) array of weights
        """
        new_particles, new_weights = [], []

        M = self.num_particles
        r = np.random()/M
        c = weights[0]
        i = 0
        for m in range(1,M+1):
            u = r + (m-1) / M
            while u > c:
                i = i + 1
                c = c + weights[i] 
            new_particles.append(particles[i])
            new_weights.append(weights[i])
        
        return new_particles, new_weights

    def mean_and_variance(self, particles):
        """Compute the mean and covariance matrix for a set of equally-weighted
        particles.

        particles: (n x 3) matrix of poses
        """
        mean = particles.mean(axis=0)
        mean[2] = np.arctan2(
            np.cos(particles[:, 2]).sum(),
            np.sin(particles[:, 2]).sum()
        )

        zero_mean = particles - mean
        for i in range(zero_mean.shape[0]):
            zero_mean[i, 2] = minimized_angle(zero_mean[i, 2])
        cov = np.dot(zero_mean.T, zero_mean) / self.num_particles

        return mean.reshape((-1, 1)), cov
