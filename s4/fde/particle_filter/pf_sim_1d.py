# Particle filter simulation class file

# Imports
import numpy as np
import matplotlib.pyplot as plt


class ParticleFilter1D():
    """
    Particle filter class for 1D UAV localization along airport runways
    Inputs: airstrip, window length L, number of particles, process noise, random num generator (if applicable)
    """
    def __init__(self, strip, L, num_particles, sigma_s, random_seed = 17, rng=None):
        self.strip = strip # Runway strip
        self.L = L
        self.N = num_particles
        self.sigma_s = sigma_s              # Process noise 
        self.s_max = strip.shape[1] - L     # Last valid window start 
        self.random_seed = random_seed
        self.rng = rng if rng is not None else np.random.default_rng(self.random_seed)
        self.particles = None
        self.weights = None
        

    def init_uniform(self):
        """
        Initiate uniform particle distribution over valid track positions 
        """
        self.particles = self.rng.uniform(0, self.s_max, self.N)
        self.weights = np.full(self.N, 1.0/self.N) # Equally weighted

    def predict(self, v, dt):
        """
        Predicting the movement of each particle with motion and process noise
        NOTE: constant velocity motion, also clipping to valid track 
        Inputs: velocity, timestep
        """
        self.particles += v*dt + self.rng.normal(0,self.sigma_s,self.N)
        self.particles = np.clip(self.particles, 0, self.s_max)
    








# SIMULATION FUNCTIONS
"""
Functions to be used for the simulation 
NOT fed into the particle filter.
This way, the filter is unaware of actual state
Functions: true_path, capture
"""


def true_path(s0, v, dt, K):
    """
    True along track UAV position at each step, constant velocity
    Input: initial position (in px), velocity (px/s), time step dt (s), 
    number of steps in FULL traject K
    Output: s_true (K,), in px
    """
    return s0 + v*dt * np.arange(K)

def capture(strip, s, L):
    """
    Capturing image view from a UAV position, image at along-track position
    Input: airstrip (x,y) pixel grid, position s (px), Length of window L (px)
    Output: window (strip height, L)
    """
    s = int(round(s))
    return strip[:, s:s + L]



def plot_curr_particles(strip, particles, s_true=None):
    """
    Plot and visualize current distribution of particles 
    Input: rectified strip, particle positions, true position (optional)
    """
    fig, axs = plt.subplots(2, 1, figsize=(14, 6), sharex=True)
    axs[0].imshow(strip, cmap='gray', aspect='auto')
    axs[0].scatter(particles, np.full_like(particles, strip.shape[0] // 2), s=4, c='r')
    axs[0].set_title("Particles on strip")
    axs[1].hist(particles, bins=100, range=(0, strip.shape[1]))
    axs[1].set_xlabel("s (px along track)")
    axs[1].set_ylabel("count")
    if s_true is not None:
        for ax in axs:
            ax.axvline(s_true, c='g', lw=2)
    plt.tight_layout()
    plt.show()