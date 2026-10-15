# Daniel Choate

# Calibration study: gNAV_agent with intrinsics supplied by the user
# (VT / April / Current) instead of read from COLMAP


import numpy as np
import cv2
from scipy.spatial import cKDTree
from groundNAV_agent import gNAV_agent


class gNAV_calib(gNAV_agent):
	"""
	Same as gNAV_agent, plus K (3x3) and dist (OpenCV distortion coeffs)
	"""
	def __init__(self, colm_fn, images, sat_ref, ind_params, K, dist):
		self.K = np.asarray(K, dtype=float)
		self.dist = np.asarray(dist, dtype=float)
		super().__init__(colm_fn, images, sat_ref, ind_params)


	def get_focal(self):
		"""
		Focal length from supplied K (fx only - unit_vec_c uses one focal)
		"""
		return self.K[0,0]


	def unit_vec_c(self, imnum):
		"""
		Unit vectors in camera frame for ROI pixels, using K and dist
		(replaces base pinhole version: image-center pp, single focal, no distortion)
		"""
		pts_loc = self.im_pts_2d[imnum]['pts'].reshape(-1,1,2).astype(np.float64)
		pts_rgb = self.im_pts_2d[imnum]['rgbc']

		# Pixels -> normalized (x, y) at z = 1, distortion removed
		crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1e-9)
		xy = cv2.undistortPointsIter(pts_loc, self.K, self.dist, None, None, crit).reshape(-1,2)

		pts_vec_c = np.column_stack((xy, np.ones(len(xy))))
		pts_vec_c /= np.linalg.norm(pts_vec_c, axis=1, keepdims=True)
		pts_rgb_gnd = pts_rgb.reshape(-1, 3) / 255

		return pts_vec_c, pts_rgb_gnd


	def ssd_norm(self, imnum, n=5, max_d=1.0):
		"""
		Normalized intensity SSD of projected frame vs satellite over (2n+1)^2 shifts (sat px)
		Footprint: sat pts whose nearest projected pt is within max_d sat px (follows curved ROI edges)
		Each shift: zero-mean / unit-std over footprint for sat and image separately, then MEAN squared diff
		Shift applied to sat pts (-shift) so the KD-tree is built once - same as shifting image pts by +shift
		Output: ssds[shiftx + n, shifty + n]
		"""
		pts = self.im_pts_best_guess[imnum]['pts'][:, :2]
		g_im = self.im_mosaic[imnum]['color_g'][:, 0]
		tree = cKDTree(pts)

		# Sat pts in the footprint bounding box (+ shift margin) - avoids querying the whole sat image
		lo, hi = pts.min(0) - n - 1, pts.max(0) + n + 1
		near = np.all((self.ref_pts[:, :2] >= lo) & (self.ref_pts[:, :2] <= hi), axis=1)
		sat_xy, g_sat = self.ref_pts[near, :2], self.ref_rgb[near, 0]

		ssds = np.zeros((2*n+1, 2*n+1))
		for sx in range(-n, n+1):
			for sy in range(-n, n+1):
				d, idx = tree.query(sat_xy - np.array([sx, sy]), k=1, distance_upper_bound=max_d)
				m = np.isfinite(d)   # inside footprint
				a, b = g_sat[m], g_im[idx[m]]
				a = (a - a.mean()) / a.std()
				b = (b - b.mean()) / b.std()
				ssds[sx + n, sy + n] = np.mean((a - b)**2)

		return ssds
