# Daniel Choate
# Utilities for micropatch distribution analysis 
# Better organization for eigenvalue decomposition, covariance calculations, etc
# Micropatch distributions done in gnav 

import numpy as np

class microp_distb:
	"""
	Class for distribution analysis and adjustments 
	Organizing analysis for 
	Input: gnav class 
	"""
	def __init__(self, gnav, n, n_ssd, t = 1/12):
		self.gnav = gnav # gnav agent 
		self.n = n # micropatch grid size (usually 8)
		self.n_ssd = n_ssd # ssd search size (typically 5 for 5x5)
		self.t = t # Threshold constant
		self.set_thresh(t)
		self.im_num = self.gnav.im_num
		self.distb_vecs = self.gnav.distb_vecs
		self.distb_pts = self.gnav.distb_pts
		self.ssds_curr_micro = self.gnav.ssds_curr_micro


	def set_thresh(self, t=1/12):
		"""
		Set threshold for confidence directions
		Input: threshold constant (1/12 for long bar)
		Output: T threshold 
		"""

		L = self.n_ssd * 2
		self.T = t * (L**2)


	def calc_mean_var(self):
		"""
		Determine mean and variance for micropatches
		Output: mpa.full_mp_mean_var contains mean and variance of
			micropatches for each image 
		"""

		# Initialize
		self.full_mp_mean_var = [{} for _ in range(self.im_num)]

		# mean and variance calc
		for imnum in range(self.im_num):
			# All micropatch corrections
			cor_vecs = self.distb_vecs[imnum]
			if cor_vecs.shape[0] > 1:
				# Mean and variance
				x_bar = np.mean(cor_vecs[:,0])
				y_bar = np.mean(cor_vecs[:,1])
				self.full_mp_mean_var[imnum]['mu'] = np.array([x_bar, y_bar])
				cov_matrix = np.cov(cor_vecs.T)
				self.full_mp_mean_var[imnum]['cov'] = cov_matrix
			else: 
				self.full_mp_mean_var[imnum]['mu'] = cor_vecs[0] if cor_vecs.shape[0] == 1 else np.array([0,0])
				self.full_mp_mean_var[imnum]['cov'] = np.zeros((2,2))



	def prob_distb_softmax(self, ssds, n):
		"""
		Generate a probability distribution for ssd values through softmax function
		Inputs: ssds (2nx1 x 2nx1), n (shift max)
		Outputs: probabilities (2n+1 x 2n+1)
		"""

		# Calculate beta 
		ssd_std = np.std(ssds)
		beta = 1.0 / (ssd_std + 1e-8)
		# beta *+ 5
		# beta = 1

		# Probability array 
		s = np.asarray(ssds, dtype=float)
		min_ssd = s.min()
		# Center 
		s_centered = -beta * (s - min_ssd)
		# Shift for numerical stability
		m = s_centered.max()
		exp_shifted = np.exp(s_centered-m)
		probs = exp_shifted / (np.sum(exp_shifted) + 1e-8)

		return probs

	def comp_mean_cov(self, probs, ssds, n):
		"""
		Compute the mean shift with the given probability vectors
		Inputs: probabilities, ssds, n(shiftmax)
		Outputs: mean, covariance of shift
		"""
		probs = probs.T
		# X and Y grid
		xs = np.arange(-n, n+1)
		X, Y = np.meshgrid(xs, xs, indexing='xy')
		# print(X,Y)
		coords = np.stack([X, Y], axis=-1)
		# print(coords)
		# print(probs[...,None])
		mu = (probs[...,None] * coords).sum(axis=(0,1))  # (2,)
		# print("MU:\n", mu)
		# Difference
		diff = coords - mu[None, None, :]           # (11,11,2)
		diff_outer = diff[..., :, None] * diff[..., None, :]  # (11,11,2,2)
		cov = np.sum(probs[..., None, None] * diff_outer, axis=(0,1))  # (2,2)

		return mu, cov

	
	# FULL PATCH SOFTMAX
	def FP_softmax(self):
		""" 
		Calculating full probability distribution using softmax
		Determining mean, covariance, and probabilities 
		Inputs:
		Outputs: mpa.prob_distb_FP with mu, cov, probs
		"""

		# Initialize
		self.prob_distb_FP = [{} for _ in range(self.im_num)]

		for imnum in range(self.im_num):
			# Probability distribution
			ssds = self.ssds_curr_SM[imnum]
			probs = self.prob_distb_softmax(ssds, self.n_ssd)
			# print("Probability MAX:", probs.max())
			# Avg correction vector 
			mu, cov = self.comp_mean_cov(probs, ssds, self.n_ssd)

			self.prob_distb_FP[imnum]['mu'] = mu
			self.prob_distb_FP[imnum]['cov'] = cov
			self.prob_distb_FP[imnum]['probs'] = probs




	def elong_ims(self):
		"""
		Determine elongated images based on threshold
		Looking at FULL PATCH softmax distribution
		Output: mpa.ims_elong variable which contains:
			elongated image indices, perpendicular vector (in conf dir)
		"""

		# Initialize elongated images  
		self.ims_elong = []
		self.num_elong = 0
		for imnum in range(self.im_num):
			cov = self.prob_distb_FP[imnum]['cov']
			eigvals, eigvecs = np.linalg.eigh(cov)
			print('\nEigenvalues:\n', eigvals)
			print('Eigenvectors:\n', eigvecs)
			idx = np.argsort(eigvals)
			# print("Indices", idx)
			lam_small = eigvals[idx[0]]
			lam_large = eigvals[idx[-1]]
			# print("Small and large eigenval:", lam_small, lam_large)

			v_small = eigvecs[:, idx[0]]   # stable direction
			v_large = eigvecs[:, idx[-1]]  # unstable direction
			# print("Stable:\n", v_small)
			# print("Unstable:\n", v_large)

			# Check large eigenvalue
			if lam_large > self.T:
				print(f'\nImage {imnum} is ELONGATED')
				self.ims_elong.append(imnum)
				self.num_elong += 1
				# ANGLE
				angle_r = np.arctan2(v_large[1], v_large[0])
				angle_deg = np.degrees(angle_r)
				print("ANGLE:", angle_deg)
				if angle_deg > 90:
					angle_deg_half = angle_deg - 180
					v_large = -v_large
				elif angle_deg <= -90:
					angle_deg_half = angle_deg + 180
					v_large = -v_large
				else: 
					angle_deg_half = angle_deg
				v_perp = np.array([v_large[1], -v_large[0]])
				v_perp = v_perp / (np.linalg.norm(v_perp) + 1e-20)
				print("V perpendicular", v_perp)
				self.ims_elong.append(v_perp)


			# print(f'Done image {imnum}\n')


# FINISH VERIFICATION STAGES FOR FULL FUNCTION

	def exclusion_dirs(self):
		"""
		Determine exclusion directions based on threshold
		Output: mpa.sm_distb_conf variable which contains:
			mu, cov, eigvals, eigvecs, c_dirs (2,1,0)
		"""

		# Initialize dict 
		self.sm_distb_conf = [{} for _ in range(self.im_num)]

		# Tracking variables
		total_mps = 0
		# 'Junk' patches
		num_mp_cd = 0
		num_mp_cd2 = 0

		# Check each micropatch of each image 
		for imnum in range(self.im_num):
			for mp in range(len(self.gnav.micro_ps[imnum])):
				total_mps += 1
				# Make sure dict exists for each mp
				self.sm_distb_conf[imnum].setdefault(mp, {})

				# print(f'IMAGE: {imnum},\n MP: {mp}')
				# Grab ssd of mp 
				ssds_mp = self.gnav.ssds_curr_micro[imnum][mp]
				# Probability distribution - softmax
				probs = self.prob_distb_softmax(ssds_mp, self.n_ssd)
				# print(f'Probability MAX: {probs.max()}')
				# Avg correction vector
				mean_shift, cov = self.comp_mean_cov(probs, ssds_mp, self.n_ssd)
				# print("Mean shift: ", mean_shift)
				# print("Covariance:\n", cov)

				# Insert mean and covariance
				self.sm_distb_conf[imnum][mp]['mu'] = mean_shift
				self.sm_distb_conf[imnum][mp]['cov'] = cov

				# ------ Determine confidence directions -------
				# Assume 2 to start
				conf_dirs = 2
				self.sm_distb_conf[imnum][mp]['cdirs'] = conf_dirs
				eigvals, eigvecs = np.linalg.eigh(cov)
				# print('\nEigenvalues:\n', eigvals)
				# print('Eigenvectors:\n', eigvecs)
				idx = np.argsort(eigvals)
				# print("Indices", idx)
				lam_small = eigvals[idx[0]]
				lam_large = eigvals[idx[-1]]
				# print("Small and large eigenval:", lam_small, lam_large)
				# print("SMALL:", lam_small)

				v_small = eigvecs[:, idx[0]]   # more stable direction
				v_large = eigvecs[:, idx[-1]]  # less stable direction
				# print("More stable:\n", v_small)
				# print("Less stable:\n", v_large)
				# ALWAYS LISTED SMALL THEN LARGE
				self.sm_distb_conf[imnum][mp]['eigvals'] = np.array([lam_small, lam_large])
				# print('EIGVALS:', self.sm_distb_conf[imnum][mp]['eigvals'][0])

				# Normalize eigenvectors

				self.sm_distb_conf[imnum][mp]['eigvecs'] = np.array([v_small, v_large])

				# --- Check large eigenvalue ---
				if lam_large > self.T:
					num_mp_cd += 1
					self.sm_distb_conf[imnum][mp]['cdirs'] -= 1
					# print(f'Image {imnum}, MP {mp} has 1 less confident direction.')

				if lam_small > self.T:
					num_mp_cd2 += 1
					self.sm_distb_conf[imnum][mp]['cdirs'] -= 1
					# print(f'Image {imnum}, MP {mp} has NO confident direction.')

				# print("GOT HERE") 
		print("Total micropatches:", total_mps)
		print("Micropatches with less than 2 confidence directions:", num_mp_cd)
		print("Micropatches with less than NO confidence directions:", num_mp_cd2)



	def blue_isolation(self):
		""" 
		Isolation of blue (2-directional confidence) points with new mean and cov
			of just these points 
		Input: number of confident directions (0, 1, or 2)
		Output: new variable (mpa.distb_vecs_blue) with mean and covariance of just blue pts
		"""

		self.distb_vecs_blue = [[] for _ in range(self.im_num)]

		#******************************************




