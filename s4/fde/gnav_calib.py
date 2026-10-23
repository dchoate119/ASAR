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


	def raster_patch(self, imnum, min_n = 4, win = 5, n = 0):
		"""
		Resampling the projected frame into a 2D grid on sat pixels for gradient calculations
		Each sat px = mean gray of the camera pts that land in it (near field averaged down to sat res)
		Far-field mask: sat px whose win x win local mean count < min_n -> NaN (camera coarser than sat there)
		Empty px inside the kept region (gaps between far-field rows of camera pts) filled from 3x3 neighbors - interpolated, not measured
		min_n, win are eyeballed defaults - tune from the cnt plot
		n: extra sat px on each side of the sat crop, room to slide it +-n px under the patch
		Output: patch (2D camera gray, NaN = masked), sat (2D sat gray, patch box + n each side), cnt (camera pts per sat px)
		"""
		pts = self.im_pts_best_guess[imnum]['pts'][:, :2]
		g_im = self.im_mosaic[imnum]['color_g'][:,0]

		# Sat pixel each camera pt lands in (ref_pts are integer sat px -> round), relative to box corner
		px = np.round(pts).astype(int)
		x0, y0 = px.min(0)
		x1, y1 = px.max(0)
		shape = (y1 - y0 + 1, x1 - x0 + 1)
		flat = (px[:, 1] - y0) * shape[1] + (px[:, 0] - x0)

		# Sum and count of camera pts per sat px
		sums = np.bincount(flat, weights=g_im, minlength=shape[0]*shape[1]).reshape(shape)
		cnt = np.bincount(flat, minlength=shape[0]*shape[1]).reshape(shape)

		# Mean per sat px; low local count (sparse outer edge, outside footprint) -> NaN
		patch = np.full(shape, np.nan)
		cnt_avg = cv2.blur(cnt.astype(np.float64), (win, win))
		keep = cnt_avg >= min_n
		has = keep & (cnt > 0)
		patch[has] = sums[has] / cnt[has]

		# Fill empty px (gaps between far-field rows of camera pts) with the mean of pts in their 3x3 neighborhood
		sums_3, cnt_3 = cv2.blur(sums, (3, 3)), cv2.blur(cnt.astype(np.float64), (3, 3))
		gap = keep & (cnt == 0) & (cnt_3 > 0)
		patch[gap] = sums_3[gap] / cnt_3[gap]

		# Satellite is already 2D - just crop the same box, n px bigger each side (ref_pts = sat px - [700, 600])
		sat = self.sat_ref[y0+600-n : y1+601+n, x0+700-n : x1+701+n] / 255
		return patch, sat, cnt


	def grad_score(self, patch, sat):
		"""
		Gradient-orientation agreement between rasterized camera patch and sat crop (same grid, from raster_patch)
		S = magnitude difference sum sum|m_sat - m_im|, weighted mean sum(w cos(2 dtheta)) / sum(w), or orientation balanced
		(weighted mean per sat edge-direction bin, then plain mean over bins) - swap by commenting, w = |grad_sat| |grad_im|
		cos(2 dtheta): +1 parallel edges (either polarity), -1 perpendicular; w ~ 0 unless both have an edge
		NaN in patch spreads to its 3x3 neighbors through Sobel -> px next to holes / outside footprint skipped
		Output: S (difference sum: lower = better, weighted mean / balanced: higher = better), m_im, m_sat (gradient magnitudes, for plotting)
		"""
		gx_i, gy_i = cv2.Sobel(patch, cv2.CV_64F, 1, 0), cv2.Sobel(patch, cv2.CV_64F, 0, 1)
		gx_s, gy_s = cv2.Sobel(sat, cv2.CV_64F, 1, 0), cv2.Sobel(sat, cv2.CV_64F, 0, 1)
		m_im, m_sat = np.hypot(gx_i, gy_i), np.hypot(gx_s, gy_s)

		# Angle between edge directions, weighted by both edge strengths
		d_theta = np.arctan2(gy_s, gx_s) - np.arctan2(gy_i, gx_i)
		w = m_sat * m_im
		ok = np.isfinite(w)
		# S = np.sum(np.abs(m_sat[ok] - m_im[ok]))                       # Magnitude difference sum (lower = better)
		# S = np.sum(w[ok] * np.cos(2 * d_theta[ok])) / np.sum(w[ok])  # Weighted mean (-1 to 1)

		# Orientation balanced: score each sat edge-direction bin (mod 180 deg, 8 bins) separately, then plain mean over bins
		b = (np.mod(np.arctan2(gy_s, gx_s), np.pi) / (np.pi / 8)).astype(int) % 8
		W_b = np.bincount(b[ok], weights=w[ok], minlength=8)
		A_b = np.bincount(b[ok], weights=w[ok] * np.cos(2 * d_theta[ok]), minlength=8)
		use = W_b > 0.02 * W_b.sum()   # skip near-empty bins (pure noise) - 2% cutoff eyeballed
		S = np.mean(A_b[use] / W_b[use])
		return S, m_im, m_sat


	def grad_score_shifts(self, imnum, n = 5):
		"""
		Gradient score over (2n+1)^2 integer shifts of the projected frame (sat px), same layout as ssd_norm
		Output: S[shiftx + n, shifty + n] (higher = better)
		"""
		patch, sat, __ = self.raster_patch(imnum, n=n)
		H, W = patch.shape

		S = np.zeros((2*n+1, 2*n+1))
		for sx in range(-n, n+1):
			for sy in range(-n, n+1):
				# Sat window under the patch, image shifted by +(sx, sy) -> window moves by -(sx, sy) (same sign as ssd_norm)
				S[sx + n, sy + n] = self.grad_score(patch, sat[n-sy : n-sy+H, n-sx : n-sx+W])[0]
		return S