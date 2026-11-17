# Particle filter simulation class file

# Imports
import numpy as np
import matplotlib.pyplot as plt
import cv2


class ParticleFilter1D():
    """
    Particle filter class for 1D UAV localization along airport runways
    Inputs: airstrip, window length L, number of particles, process noise, random num generator (if applicable)
    """
    def __init__(self, strip, L, num_particles, sigma_s, random_seed = 17, rng=None, heading=1):
        self.strip = strip # Runway strip
        self.L = L
        self.N = num_particles
        self.sigma_s = sigma_s              # Process noise
        self.heading = heading              # +1 moving toward larger s, -1 toward smaller s (known from INS)
        W = strip.shape[1]
        # Valid UAV positions so the forward window [s, s+L] or [s-L, s] stays on the strip
        self.s_min, self.s_max = (0, W - L) if heading > 0 else (L, W)
        self.random_seed = random_seed
        self.rng = rng if rng is not None else np.random.default_rng(self.random_seed)
        self.particles = None
        self.weights = None
        

    def init_uniform(self):
        """
        Initiate uniform particle distribution over valid track positions 
        """
        self.particles = self.rng.uniform(self.s_min, self.s_max, self.N)
        self.weights = np.full(self.N, 1.0/self.N) # Equally weighted

    def predict(self, delta_s):
        """
        Predicting the movement of each particle with motion and process noise
        NOTE: constant velocity motion, also clipping to valid track 
        Inputs: change in distance (gather from motion model, IMU, etc...), timestep
        """
        noise = self.rng.normal(0, self.sigma_s, self.N)
        self.particles += delta_s + noise
        self.particles = np.clip(self.particles, self.s_min, self.s_max)


    def update(self, z, sigma_z):
        """
        Reweight particles by how well the captured window matches the particles window
        Score: MSE mean-squared error of pixel differences 
        Likelihood: exp(-MSE / (2 sigma_z^2))
        Inputs: captured window z, likelihood std (gray levels)
        NOTE: to be switched with proper update fcn
        """
        # Steps 1-2: cut each particle's window and score against z
        mse = np.array([np.mean((get_window(self.strip, p, self.L, self.heading) - z)**2)
                        for p in self.particles])
        # Step 3: log-likelihood, shifted so the best particle is 0 (avoids underflow)
        log_lik = -mse / (2 * sigma_z**2)
        log_lik -= log_lik.max()
        # Steps 4-5: Bayes update and normalize
        self.weights *= np.exp(log_lik)
        self.weights /= self.weights.sum()


    def update_st(self, z_J, J_strip, sigma_z):
        """
        Reweight particles by structure tensor similarity between captured and particle windows
        Score: NCC between flattened (3, H, L) structure tensor windows
        Likelihood: exp(-(1 - NCC) / (2 sigma_z^2))
        Inputs: captured window structure tensor z_J (3, H, L), strip structure tensor J_strip (3, H, W),
        likelihood std sigma_z (on the 1 - NCC scale)
        NOTE: per-pixel J (no cell pooling), per-pair NCC (no dataset centering)
        """
        # Steps 1-2: cut each particle's J window and score against z_J with NCC
        z = z_J.ravel() - z_J.mean()
        z /= np.linalg.norm(z) + 1e-12
        ncc = np.empty(self.N)
        for i, p in enumerate(self.particles):
            w = get_window(J_strip, p, self.L, self.heading).ravel()
            w = w - w.mean()
            ncc[i] = z @ w / (np.linalg.norm(w) + 1e-12)
        # Step 3: log-likelihood, shifted so the best particle is 0 (avoids underflow)
        log_lik = -(1 - ncc) / (2 * sigma_z**2)
        log_lik -= log_lik.max()
        # Steps 4-5: Bayes update and normalize
        self.weights *= np.exp(log_lik)
        self.weights /= self.weights.sum()


    def n_eff(self):
        """
        Effective sample size: N if weights are uniform, 1 if one particle holds all weight
        Determines if resample is necessary (N_eff < N/2)
        """
        return 1.0 / np.sum(self.weights**2)

    def resample(self):
        """
        Systematic particle resampling: N evenly spaced points with random offset over cumm weights
        High-weight particles copied, low-weight ones dropped. Weights reset to 1/N 
        """
        points = (self.rng.random() + np.arange(self.N)) / self.N
        cum_w = np.cumsum(self.weights)
        cum_w[-1] = 1.0
        idx = np.searchsorted(cum_w, points)
        self.particles = self.particles[idx]
        self.weights = np.full(self.N, 1.0 / self.N)
    








# SIMULATION FUNCTIONS
"""
Functions to be used for the simulation 
NOT fed into the particle filter (except get window fcn).
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


def get_window(strip, s, L, heading=1):
    """
    Forward-looking window from UAV position s: [s, s+L] if heading +1, [s-L, s] if heading -1
    NOTE: no 180 deg rotation for heading -1 (capture and particle windows are cut the same way)
    Input: airstrip (H, W) or stacked maps (C, H, W), position s (px), window length L (px), heading (+1 / -1)
    Output: window (..., strip height, L)
    """
    s = int(round(s))
    w_0, w_1 = (s, s + L) if heading > 0 else (s - L, s)
    if w_0 < 0 or w_1 > strip.shape[-1]:
        raise ValueError(f"s={s} window [{w_0}, {w_1}] outside strip [0, {strip.shape[-1]}]")
    return strip[..., w_0:w_1]

def capture(strip, s, L, heading=1, sigma_meas=0, rng=None):
    """
    Capturing image view from a UAV position, image at along-track position
    Input: airstrip (x,y) pixel grid, position s (px), Length of window L (px),
    pixel noise std sigma_meas, rng for noise, heading (+1 / -1)
    Output: window (strip height, L)
    """
    window = get_window(strip, s, L, heading).astype(np.float32)
    if sigma_meas > 0:
        window += rng.normal(0, sigma_meas, window.shape)
    return window



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


def st_preprocess(img, sigma=2, max_angle=60):
    """
    Structure tensor maps for the PF update: Gaussian-smoothed (integration scale sigma) outer product
    of the Sobel gradient, normalized by trace, orientation filtered
    Trace normalization: contrast-invariant, keeps orientation + coherence only
    Orientation filter: zeros pixels whose dominant gradient is more than max_angle deg from along-track
    Input: grayscale image (H, W), sigma (px), max_angle (deg)
    Output: (3, H, W) maps Jxx, Jyy, Jxy
    """
    img = img.astype(np.float32)
    gx = cv2.Sobel(img, cv2.CV_32F, 1, 0)
    gy = cv2.Sobel(img, cv2.CV_32F, 0, 1)
    J = np.stack([cv2.GaussianBlur(a, (0, 0), sigma) for a in (gx*gx, gy*gy, gx*gy)])
    J = J / (J[0] + J[1] + 1e-6)
    theta = 0.5 * np.arctan2(2*J[2], J[0] - J[1]) # Dominant gradient angle, 0 = along-track
    return J * (np.abs(theta) <= np.deg2rad(max_angle))

