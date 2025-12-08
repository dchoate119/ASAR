# Utilities file for distribution visualizations 

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import cv2
from shapely.geometry import Polygon, box, Point

class microp_distb_plotter:
	""" 
	Different plotting functions for some helpful visualizations
	Also cleaning up notebook slop
	Functions: 
	- Microps w corVecs: plot patch trapezoids with grids of micropatches AND error vectors
	- Microps w corVecs, SOFTMAX: NOT USING FOR NOW
	- Plot distb vecs: ............
	- Plot distb vecs BOTH: ........
	"""
	def __init__(self, gnav, n, n_ssd):
		self.gnav = gnav # Gnav agent we need 
		self.n = n # Microp grid size 
		self.n_ssd = n_ssd # ssd shift size 


	def plot_microps_w_corVecs(self):
		"""
		Use matplotlib to plot the patch trapezoids with the grids of the 
		micropatches AND error correction vectors 
		Input: Just using the gnav
		Output: Subplot with trapezoids and micropatch grids
		"""
		# Number of ims
		num_imgs = self.gnav.im_num
		# print(num_imgs)

		# Create figure and axes
		fig, axes = plt.subplots(num_imgs, 1, figsize=(10,20))

		# Each image
		for i in range(num_imgs):
			ax = axes[i] if num_imgs >1 else axes

			# Poly bound of mosaic points 
			# Corners
			pts_curr = self.gnav.im_pts_best_guess[i]['pts']
			corners = self.gnav.im_pts_2d[i]['corners']
			# Corner indices 
			idxs = [0, -corners[2], -1, corners[2]-1]
			# Grab corner points 
			pts_corners = np.array(pts_curr[idxs])
			# Create polygon
			poly = Polygon(pts_corners[:,:2])
			# Draw base of trapezoid
			x, y = poly.exterior.xy
			ax.fill(x,y, alpha=0.3, color='gray', label='Trapezoid')

			# Draw micropatches from corners
			for j in range(len(self.gnav.micro_ps[i])):
				corners = self.gnav.micro_ps[i][j]['corners']
				if corners is None or len(corners) == 0:
					continue

				# Close polygon by repeating first pt
				corners_closed = np.vstack([corners, corners[0]])
				xs, ys = corners_closed[:, 0], corners_closed[:, 1]
				ax.plot(xs, ys, color='blue', linewidth=0.7)

			# Draw CORRECTION VECS for each microp
			pts_corr = self.gnav.distb_pts[i]
			mps = len(self.gnav.ssds_curr_micro[i])
			# Split into base and end points
			base = pts_corr[:mps]
			end = pts_corr[mps:]

			# Compute vector components
			u = end[:, 0] - base[:, 0]
			v = end[:, 1] - base[:, 1]

			# Plot arrows
			ax.quiver(base[:, 0], base[:, 1], u, v,
				angles='xy', scale_units='xy', scale=1,
				color='red', width=0.004, label='Correction Vectors')

			# mark start and end points
			ax.scatter(base[:, 0], base[:, 1], color='black', s=10, label='Base')
			ax.scatter(end[:, 0], end[:, 1], color='orange', s=10, label='End')


			ax.set_aspect('equal')
			ax.set_title(f"Image {i}")



		# legend
		handles, labels = axes[0].get_legend_handles_labels() if num_imgs > 1 else ax.get_legend_handles_labels()
		fig.legend(handles, labels, loc='upper right')
		plt.tight_layout()
		plt.show()


	def plot_distb_vecs(self, distb_vecs, distb_mean_var):
		"""
		Plotting the x y offset correction vector for each mircopatch per image
		Input: distribution vectors, distribution mean and variance
		Output: 5 plots (or # of images) with x and y correction vector points
		"""

		num_imgs = self.gnav.im_num
		fig, axes = plt.subplots(1, num_imgs, figsize=(20, 4 * num_imgs), squeeze=False)
		axes = axes.flatten()

		for i in range(num_imgs):
			ax = axes[i] if num_imgs > 1 else axes

			# grab correction vectors
			xs = distb_vecs[i][:,0]
			ys = distb_vecs[i][:,1]
			ax.scatter(xs,ys, s=10)

			# Mean and cov
			mean = distb_mean_var[i]['mean']
			cov = distb_mean_var[i]['cov']
			# Draw ellipse
			if np.any(cov) and not np.isnan(cov).any():
				vals, vecs = np.linalg.eigh(cov)
				order = vals.argsort()[::-1]
				vals, vecs = vals[order], vecs[:, order]
				angle = np.degrees(np.arctan2(*vecs[:,0][::-1]))
				width, height = 2 * np.sqrt(vals)  # 1-sigma ellipse

				ell = Ellipse(xy=mean, width=width, height=height, angle=angle,
				              color='red', alpha=0.3, lw=2, label='Covariance (1σ)')
				ax.add_patch(ell)

				# Mark the mean
				ax.scatter(*mean, color='red', s=40, marker='x', label='Mean')

			# Styling
			ax.set_title(f"Image {i}: grid = {self.n}x{self.n}", fontsize=12)
			ax.set_xlim([-6, 6])
			ax.set_ylim([-6, 6])
			ax.set_xlabel("X Offset")
			ax.set_ylabel("Y Offset")
			ax.set_aspect('equal', adjustable='box')
			ax.grid(True, linestyle='--', alpha=0.6)
			# ax.set_title(f"Image {i}")

	def plot_softmax_microp_COMP(self, distb_vecs, distb_mean_var, distb_mean_var_SM):
		"""
		Plot comparison between softmax mean and variance of full patch, ...
		and micropatch distribution
		Input: ... CHECK THESE
		Output 5 plots with distibution of microp pts and ellipse of full patch cov
		"""
		num_imgs = self.gnav.im_num
		fig, axes = plt.subplots(1, num_imgs, figsize=(20, 5), squeeze=False) # 4 * num_imgs
		axes = axes.flatten()

		for i in range(num_imgs):
			ax = axes[i] if num_imgs > 1 else axes


			# MICROPATCHES -----------------
			# Grab correction vectors
			xs = distb_vecs[i][:,0]
			ys = distb_vecs[i][:,1]
			ax.scatter(xs,ys, s=10)

			# Mean and cov
			mean = distb_mean_var[i]['mean']
			cov = distb_mean_var[i]['cov']
			# Draw ellipse
			if np.any(cov) and not np.isnan(cov).any():
				vals, vecs = np.linalg.eigh(cov)
				order = vals.argsort()[::-1]
				vals, vecs = vals[order], vecs[:, order]
				angle = np.degrees(np.arctan2(*vecs[:,0][::-1]))
				width, height = 2 * np.sqrt(vals)  # 1-sigma ellipse

				ell = Ellipse(xy=mean, width=width, height=height, angle=angle,
				              color='blue', alpha=0.3, lw=2, label='Covariance (1σ)')
				ax.add_patch(ell)

				# Mark the mean
				ax.scatter(*mean, color='blue', s=40, marker='x', label='Mean')




			# FULL PATCH FROM SOFTMAX -----------------
			# Mean and cov
			mean = distb_mean_var_SM[i]['mu']
			cov = distb_mean_var_SM[i]['cov']
			# Draw ellipse
			if np.any(cov) and not np.isnan(cov).any():
				vals, vecs = np.linalg.eigh(cov)
				order = vals.argsort()[::-1]
				vals, vecs = vals[order], vecs[:, order]
				angle = np.degrees(np.arctan2(*vecs[:,0][::-1]))
				width, height = 2 * np.sqrt(vals)  # 1-sigma ellipse

				ell = Ellipse(xy=mean, width=width, height=height, angle=angle,
				              color='red', alpha=0.3, lw=2, label='Covariance (1σ)')
				ax.add_patch(ell)

				# Mark the mean
				ax.scatter(*mean, color='red', s=40, marker='x', label='Mean')

			

			# Styling
			ax.set_title(f"Image {i}: grid = {self.n}x{self.n}", fontsize=12)
			ax.set_xlim([-6, 6])
			ax.set_ylim([-6, 6])
			ax.set_xlabel("X Offset")
			ax.set_ylabel("Y Offset")
			ax.set_aspect('equal', adjustable='box')
			ax.grid(True, linestyle='--', alpha=0.6)

		# Legend: blue vs red
		blue_handle = plt.Line2D([0], [0], color='blue', lw=3, label='Micropatch')
		red_handle  = plt.Line2D([0], [0], color='red',  lw=3, label='Full Patch')
		fig.legend(handles=[blue_handle, red_handle], loc='upper right')
		fig.tight_layout(rect=[0, 0, 1, 0.97])
		# axes[-1].legend(handles=[blue_handle, red_handle],
        #         loc='upper right',
        #         frameon=True)

	def ssd_surface_plots(self, ssds, imnum, n):
		"""
		xxxxx
		"""

	def plot_softmax_and_confdir(self, distb_vecs, distb_mean_var_SM, elong_ims):
		"""""
		xxxxxx
		"""
		