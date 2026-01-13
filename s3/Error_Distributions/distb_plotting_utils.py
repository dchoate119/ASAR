# Utilities file for distribution visualizations 

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import cv2
from shapely.geometry import Polygon, box, Point
import open3d as o3d

class microp_distb_plotter:
	""" 
	Different plotting functions for some helpful visualizations
	Also cleaning up notebook slop
	Functions: 
	- Microps w corVecs: plot patch trapezoids with grids of micropatches AND error vectors
	- Plot distb vecs: x y offset correction for each micropatch 
	- Softmax microp COMP: offset correction for each mp and softmax distb
	- SSD surface plots: surface plot for SSD confidence level
	- Softmax and conf dir: confidence directions for single-dir mps
	"""
	def __init__(self, gnav, mpa, n, n_ssd):
		self.gnav = gnav # Gnav agent we need CLASS
		self.mpa = mpa # Micropatch analysis CLASS
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
			pts_corr = self.mpa.distb_pts[i]
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
			mean = distb_mean_var[i]['mu']
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
		Input: distribution vectors, mean and variance distb of micropatches, mean and variance distb of FP
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
			mean = distb_mean_var[i]['mu']
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
		Surface plot visualizations for SSD confidence level 
		Inputs: Image number, ssds, n (shift length)
		Output: SSD surface plot 
		"""

		# X coords:
		x = np.linspace(-n,n, 2*n+1)
		y = np.linspace(-n,n, 2*n+1)
		Y, X = np.meshgrid(x,y)

		idrow, idcol = np.unravel_index(np.argmin(ssds), ssds.shape)
		shiftx_min = idrow - n
		shifty_min = idcol - n
		print(f"BEST SHIFT for image {imnum}:", shiftx_min, shifty_min)
		# print("BEST SSD =", ssds[idrow, idcol])

		# Plot SSD as a surface
		fig = plt.figure()
		ax = fig.add_subplot(111, projection='3d')

		# Surface plot
		surf = ax.plot_surface(X, Y, ssds, cmap='viridis', edgecolor='none')

		# Highlight minimum SSD point
		ax.scatter(shiftx_min, shifty_min, ssds[idrow, idcol], color='red', s=50, label='Min SSD')

		# Set axis limits
		ax.set_xlim([-n, n])
		ax.set_ylim([-n, n])
		ax.set_zlim([np.min(ssds), np.max(ssds)])

		# Set ticks at every 1 unit
		ax.set_xticks(np.arange(-n, n+1, 5))
		ax.set_yticks(np.arange(-n, n+1, 5))
		ax.set_zticks([])

		# Labels and title
		ax.set_xlabel('X Shift (pixels)')
		ax.set_ylabel('Y Shift (pixels)')
		# ax.set_zlabel('SSD Value')
		ax.set_title(f'SSD Surface Plot: Image {imnum}')
		ax.legend()
		ax.view_init(elev=90, azim=-90)
		# Color bar for the surface
		# fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, label='SSD Value')

		plt.show()



	def plot_softmax_and_confdir(self, distb_vecs, distb_mean_var_SM, elong_ims):
		"""""
		Plotting the softmax distribution for FULL IMAGE, confidence directions (when needed), 
		and distribution points for micropatches
		Inputs: distribution points for micropatches, softmax distribution, elongated ims
		Output: 5 plots of confidence directions on SM distb with micropatch distb
		"""

		num_imgs = self.gnav.im_num
		fig, axes = plt.subplots(1, num_imgs, figsize=(20, 4 * num_imgs), squeeze=False)
		axes = axes.flatten()
		imss = 0

		for i in range(num_imgs):
			ax = axes[i] if num_imgs > 1 else axes

			# grab correction vectors
			xs = distb_vecs[i][:,0]
			ys = distb_vecs[i][:,1]
			ax.scatter(xs,ys, s=10)

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

			# Perpendicular DIRECTION 
			# if i in elong_ims:
			if any((not isinstance(e, np.ndarray) and i == e) or (isinstance(e, np.ndarray) and i in e) for e in elong_ims):
				v_perp = elong_ims[imss*2+1]
				ax.quiver(mean[0], mean[1], v_perp[0], v_perp[1],
				      angles='xy', scale_units='xy', scale=.5,
				      color='red', width=0.008, label='Correction Vectors')
				imss += 1
				ax.set_title(f"Image {i}: SINGLE conf-dir", fontsize=12)
			else:
				ax.set_title(f"Image {i}", fontsize=12)	


			# Format
			# ax.set_title(f"Image {i}: grid = {self.n}x{self.n}", fontsize=12)
			ax.set_xlim([-6, 6])
			ax.set_ylim([-6, 6])
			ax.set_xlabel("X Offset")
			ax.set_ylabel("Y Offset")
			ax.set_aspect('equal', adjustable='box')
			ax.grid(True, linestyle='--', alpha=0.6)
			# ax.set_title(f"Image {i}")

		# plt.show()



	def plot_distb_vecs_updated(self):
		"""
		Plot all micropatch distribution points, highlighting 
		confidence for each micropatch
		Inputs: ****
		Outputs: 5 plots with blue, yellow, and red points for each micropatch
		"""

		num_imgs = self.gnav.im_num
		fig, axes = plt.subplots(1, num_imgs, figsize=(20, 4 * num_imgs), squeeze=False)
		axes = axes.flatten()
		imss = 0

		# grab correction vectors
		distb_vecs = self.mpa.distb_vecs
		distb_mean_var = self.mpa.full_mp_mean_var # Distribution of ALL micropatches per image

		for i in range(num_imgs):
			ax = axes[i] if num_imgs > 1 else axes

			xs = distb_vecs[i][:,0]
			ys = distb_vecs[i][:,1]
			for mp in range(len(self.gnav.micro_ps[i])):
				conf_dirs = self.mpa.sm_distb_conf[i][mp]['cdirs']
				if conf_dirs == 2:
					ax.scatter(xs[mp], ys[mp], s=10, color='blue')
					# print("VERY CONFIDENT")
				if conf_dirs == 1:
					ax.scatter(xs[mp], ys[mp], s=10, color='orange')
				if conf_dirs == 0:
					ax.scatter(xs[mp], ys[mp], s=10, color='red')
					# print("NOT AT ALL CONFIDENT")

			# Mean and cov
			mean = distb_mean_var[i]['mu']
			cov = distb_mean_var[i]['cov']
			# Draw ellipse
			if np.any(cov) and not np.isnan(cov).any():
				vals, vecs = np.linalg.eigh(cov)
				order = vals.argsort()[::-1]
				vals, vecs = vals[order], vecs[:, order]
				angle = np.degrees(np.arctan2(*vecs[:,0][::-1]))
				width, height = 2 * np.sqrt(vals)  # 1-sigma ellipse

				ell = Ellipse(xy=mean, width=width, height=height, angle=angle,
				              color='blue', alpha=0.1, lw=2, label='Covariance (1σ)')
				ax.add_patch(ell)

				# Mark the mean
				ax.scatter(*mean, color='blue', s=40, marker='x', label='Mean')



			# Format
			# ax.set_title(f"Image {i}: grid = {self.n}x{self.n}", fontsize=12)
			ax.set_xlim([-6, 6])
			ax.set_ylim([-6, 6])
			ax.set_xlabel("X Offset")
			ax.set_ylabel("Y Offset")
			ax.set_aspect('equal', adjustable='box')
			ax.grid(True, linestyle='--', alpha=0.6)
			# ax.set_title(f"Image {i}")

		# plt.show()



	def plot_distb_vecs_BLUE(self):
		"""
		Plot all micropatch distribution points
		Covariance ONLY using the blue points, as opposed to all points 
		Inputs: ****
		Outputs: 5 plots with blue, yellow, and red points for each micropatch, covariance with just blue points
		"""

		num_imgs = self.gnav.im_num
		fig, axes = plt.subplots(1, num_imgs, figsize=(20, 4 * num_imgs), squeeze=False)
		axes = axes.flatten()
		imss = 0

		# grab correction vectors
		distb_vecs = self.gnav.distb_vecs_blue
		distb_mean_var = self.gnav.distb_mean_var_blue

		for i in range(num_imgs):
			ax = axes[i] if num_imgs > 1 else axes

			xs = distb_vecs[i][:,0]
			ys = distb_vecs[i][:,1]
			ax.scatter(xs, ys, s=10, color='blue')
					
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
				              color='blue', alpha=0.1, lw=2, label='Covariance (1σ)')
				ax.add_patch(ell)

				# Mark the mean
				ax.scatter(*mean, color='blue', s=40, marker='x', label='Mean')



			# Format
			# ax.set_title(f"Image {i}: grid = {self.n}x{self.n}", fontsize=12)
			ax.set_xlim([-6, 6])
			ax.set_ylim([-6, 6])
			ax.set_xlabel("X Offset")
			ax.set_ylabel("Y Offset")
			ax.set_aspect('equal', adjustable='box')
			ax.grid(True, linestyle='--', alpha=0.6)
			# ax.set_title(f"Image {i}")

		# plt.show()



	def plot_distb_vecs_w_singleDIR(self):
		"""
		Plot all micropatch distribution points
		Adding dotted lines to represent uncertainty of single direction patches 
		Inputs: ****
		Outputs: 5 plots with blue, yellow, and red points for each micropatch, covariance, and
				single direction uncertainty lines 
		"""

		num_imgs = self.gnav.im_num
		fig, axes = plt.subplots(1, num_imgs, figsize=(20, 4 * num_imgs), squeeze=False)
		axes = axes.flatten()
		imss = 0

		# # grab correction vectors
		# distb_vecs = self.gnav.distb_vecs_blue
		# distb_mean_var = self.gnav.distb_mean_var_blue

		# for i in range(num_imgs):
		# 	ax = axes[i] if num_imgs > 1 else axes

		# 	xs = distb_vecs[i][:,0]
		# 	ys = distb_vecs[i][:,1]
		# 	ax.scatter(xs, ys, s=10, color='blue')
					
		# 	# Mean and cov
		# 	mean = distb_mean_var[i]['mean']
		# 	cov = distb_mean_var[i]['cov']
		# 	# Draw ellipse
		# 	if np.any(cov) and not np.isnan(cov).any():
		# 		vals, vecs = np.linalg.eigh(cov)
		# 		order = vals.argsort()[::-1]
		# 		vals, vecs = vals[order], vecs[:, order]
		# 		angle = np.degrees(np.arctan2(*vecs[:,0][::-1]))
		# 		width, height = 2 * np.sqrt(vals)  # 1-sigma ellipse

		# 		ell = Ellipse(xy=mean, width=width, height=height, angle=angle,
		# 		              color='blue', alpha=0.1, lw=2, label='Covariance (1σ)')
		# 		ax.add_patch(ell)

		# 		# Mark the mean
		# 		ax.scatter(*mean, color='blue', s=40, marker='x', label='Mean')



		# 	# Format
		# 	# ax.set_title(f"Image {i}: grid = {self.n}x{self.n}", fontsize=12)
		# 	ax.set_xlim([-6, 6])
		# 	ax.set_ylim([-6, 6])
		# 	ax.set_xlabel("X Offset")
		# 	ax.set_ylabel("Y Offset")
		# 	ax.set_aspect('equal', adjustable='box')
		# 	ax.grid(True, linestyle='--', alpha=0.6)
		# 	# ax.set_title(f"Image {i}")

		# # plt.show()


	def plot_microp_distb_singleIM(self, imnum):
		"""
		Plots the distribution of each micropatch for a single image
		Inputs: Image number 
		Outputs: plot with cov ellipse for each microp in an image
		"""

		num_imgs = self.gnav.im_num
		fig = plt.figure()
		ax = fig.add_subplot(111)

		# grab correction vectors
		distb_vecs = self.gnav.distb_vecs[imnum]
		mean_var_microps = self.gnav.sm_distb_microp[imnum]

		for mp in range(len(self.gnav.micro_ps[imnum])):
			# Mean and cov
			mean = distb_vecs[mp]
			cov = mean_var_microps[mp]['cov']
			# Draw ellipse
			if np.any(cov) and not np.isnan(cov).any():
				vals, vecs = np.linalg.eigh(cov)
				order = vals.argsort()[::-1]
				vals, vecs = vals[order], vecs[:, order]
				angle = np.degrees(np.arctan2(*vecs[:,0][::-1]))
				width, height = 2 * np.sqrt(vals)  # 1-sigma ellipse

				ell = Ellipse(xy=mean, width=width, height=height, angle=angle,
				              color='blue', alpha=0.1, lw=2, label='Covariance (1σ)')
				ax.add_patch(ell)

				# Mark the mean
				ax.scatter(*mean, color='blue', s=40, marker='x', label='Mean')

		# Format
		ax.set_title(f"Image {imnum}: micropatches", fontsize=12)
		ax.set_xlim([-6, 6])
		ax.set_ylim([-6, 6])
		ax.set_xlabel("X Offset")
		ax.set_ylabel("Y Offset")
		ax.set_aspect('equal', adjustable='box')
		ax.grid(True, linestyle='--', alpha=0.6)
		# ax.set_title(f"Image {i}")

		plt.show()



	def plot_conf_level_microp(self):
		"""
		Plots the distribution of each micropatch for a single image
		Inputs: Image number 
		Outputs: plot with cov ellipse for each microp in an image
		---------------- IN PROGRESS ----------------------
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

				# Draw covariance for each micropatch 
				for mp in range(len(self.gnav.micro_ps[imnum])):
					# Mean and cov
					mean = distb_vecs[mp]
					cov = mean_var_microps[mp]['cov']
				# 	# Draw ellipse
				# 	if np.any(cov) and not np.isnan(cov).any():
				# 		vals, vecs = np.linalg.eigh(cov)
				# 		order = vals.argsort()[::-1]
				# 		vals, vecs = vals[order], vecs[:, order]
				# 		angle = np.degrees(np.arctan2(*vecs[:,0][::-1]))
				# 		width, height = 2 * np.sqrt(vals)  # 1-sigma ellipse

				# 		ell = Ellipse(xy=mean, width=width, height=height, angle=angle,
				# 		              color='blue', alpha=0.1, lw=2, label='Covariance (1σ)')
				# 		ax.add_patch(ell)

				# 		# Mark the mean
				# 		ax.scatter(*mean, color='blue', s=40, marker='x', label='Mean')




			ax.set_aspect('equal')
			ax.set_title(f"Image {i}")


		# num_imgs = self.gnav.im_num
		# fig = plt.figure()
		# ax = fig.add_subplot(111)

		# # grab correction vectors
		# distb_vecs = self.gnav.distb_vecs[imnum]
		# mean_var_microps = self.gnav.sm_distb_microp[imnum]

		# for mp in range(len(self.gnav.micro_ps[imnum])):
		# 	# Mean and cov
		# 	mean = distb_vecs[mp]
		# 	cov = mean_var_microps[mp]['cov']
		# 	# Draw ellipse
		# 	if np.any(cov) and not np.isnan(cov).any():
		# 		vals, vecs = np.linalg.eigh(cov)
		# 		order = vals.argsort()[::-1]
		# 		vals, vecs = vals[order], vecs[:, order]
		# 		angle = np.degrees(np.arctan2(*vecs[:,0][::-1]))
		# 		width, height = 2 * np.sqrt(vals)  # 1-sigma ellipse

		# 		ell = Ellipse(xy=mean, width=width, height=height, angle=angle,
		# 		              color='blue', alpha=0.1, lw=2, label='Covariance (1σ)')
		# 		ax.add_patch(ell)

		# 		# Mark the mean
		# 		ax.scatter(*mean, color='blue', s=40, marker='x', label='Mean')

		# # Format
		# ax.set_title(f"Image {imnum}: micropatches", fontsize=12)
		# ax.set_xlim([-6, 6])
		# ax.set_ylim([-6, 6])
		# ax.set_xlabel("X Offset")
		# ax.set_ylabel("Y Offset")
		# ax.set_aspect('equal', adjustable='box')
		# ax.grid(True, linestyle='--', alpha=0.6)
		# # ax.set_title(f"Image {i}")

		# plt.show()




	def phys_microp_conf(self, sat_map=True):
		"""
		Physical error plot for micropatches and confidence directions
		Input: gnav class, satellite map 
		Output: open3d plot with micropatches, with covariance ellipses for each micropatch
		Yellow covariance ellipse for uncertainty, blue for confidence
		"""

		vis = o3d.visualization.Visualizer()
		vis.create_window(window_name = "Physical Error Plot")

		# Create origin at axis
		axis_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10)
		vis.add_geometry(axis_origin)

		# # Plot each patch 
		# for i in range(len(self.gnav.images_dict)):
		# 	cloud = o3d.geometry.PointCloud()
		# 	cloud.points = o3d.utility.Vector3dVector(self.gnav.im_pts_best_guess[i]['pts'])
		# 	cloud.colors = o3d.utility.Vector3dVector(self.gnav.im_mosaic[i]['color_g'])
		# 	vis.add_geometry(cloud)

		if sat_map == True:
			cloud = o3d.geometry.PointCloud()
			cloud.points = o3d.utility.Vector3dVector(self.gnav.ref_pts)
			cloud.colors = o3d.utility.Vector3dVector(self.gnav.ref_rgb)
			vis.add_geometry(cloud)

		# Create point cloud for image points
		for i in range(len(self.gnav.images_dict)):
			for j in range(len(self.gnav.micro_ps[i])):
				cloud_micro = o3d.geometry.PointCloud()
				cloud_micro.points = o3d.utility.Vector3dVector(self.gnav.micro_ps[i][j]['pts'])
				cloud_micro.colors = o3d.utility.Vector3dVector(self.gnav.micro_ps[i][j]['color_g'])
				vis.add_geometry(cloud_micro)


		# Plot covariances 
		for imnum in range(len(self.gnav.images_dict)):
			distb_vecs = self.gnav.distb_vecs[imnum]
			mean_var_microps = self.gnav.sm_distb_microp[imnum]

			for mp in range(len(self.gnav.micro_ps[imnum])):
				# Mean and cov
				mean_pts = self.gnav.micro_ps[imnum][mp]['pts']
				mean = mean_pts.mean(axis=0)
				cov = mean_var_microps[mp]['cov']
				# Draw ellipse
				if np.any(cov) and not np.isnan(cov).any():
					vals, vecs = np.linalg.eigh(cov)
					order = vals.argsort()[::-1]
					vals, vecs = vals[order], vecs[:, order]
					radii = np.sqrt(vals)
					angle = np.degrees(np.arctan2(*vecs[:,0][::-1]))
					width, height = 2 * np.sqrt(vals)  # 1-sigma ellipse

					# Create sphere
					sph = o3d.geometry.TriangleMesh.create_sphere(radius=1, resolution=20)
					sph.compute_vertex_normals()
					V = np.asarray(sph.vertices)
					# print(mean)
					if cov.shape == (2,2):
						z_radius = 0.05 * max(radii) # ADJUST thickness

						V[:,0] *= radii[0]
						V[:,1] *= radii[1]
						V[:,2] *= z_radius 

						sph.vertices = o3d.utility.Vector3dVector(V)

						# Build 3x3 tormat
						R = np.eye(3)
						R[:2, :2] = vecs

						sph.rotate(R, center=(0,0,0))

						# Translate to mean (expects mean is length-3; if you have 2D mean, pad with 0)
						mean3 = np.array([mean[0], mean[1], mean[2] if len(mean) > 2 else 0.0])
						sph.translate(mean3)

						# COLOR BASED ON CONFIDENCE
						print(self.gnav.sm_distb_microp_confdir[imnum][mp]['lam'])
						if self.gnav.sm_distb_microp_confdir[imnum][mp]['lam'] != []:
							sph.paint_uniform_color([1.0,0.0,0.0])
						else:
							sph.paint_uniform_color([0.0, 1.0, 0.0])

						vis.add_geometry(sph)


				

				# 	ell = Ellipse(xy=mean, width=width, height=height, angle=angle,
				# 	              color='blue', alpha=0.1, lw=2, label='Covariance (1σ)')
				# 	ax.add_patch(ell)

				# 	# Mark the mean
				# 	ax.scatter(*mean, color='blue', s=40, marker='x', label='Mean')




		# Run and destroy 
		vis.run()
		vis.destroy_window()


