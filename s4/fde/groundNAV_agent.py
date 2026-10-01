# Daniel Choate 

# Vision-aided navigation for aircraft surface movement
# Tying structure from motion relative estimate to absolute coordinates
# Through satelltie reference imagery 


import numpy as np
import cv2
import open3d as o3d
import pycolmap
from pathlib import Path as pth
import re
import os
import math
import matplotlib.pyplot as plt
import copy
from scipy.spatial.transform import Rotation as R
from shapely.geometry import Polygon, box, Point
from scipy.spatial import cKDTree
from matplotlib.path import Path


class gNAV_agent:
	"""
	Agent for positioning autonomous UAVs on airport runways
	Initialization of agent 
	Inputs: Reference image (satellite), SfM solution (COLMAP), selected images
	"""
	def __init__(self, colm_fn, images, sat_ref, ind_params):
		self.colm_loc = colm_fn 			# Location of colmap folder
		self.imss = images 					# Location of specific image folder 
		self.sat_ref_c = cv2.imread(sat_ref) 	# Satellite reference image
		self.sat_ref = cv2.cvtColor(self.sat_ref_c, cv2.COLOR_BGR2GRAY)
		self.read_colmap_data()
		self.image_parsing()
		self.sat_im_init()
		# Initial scene points and RGB data
		self.grab_pts(self.pts3d_c)
		self.grab_poses(self.images_c)
		self.random_seed = 17
		# Initialize best guess and SSDs
		self.im_pts_best_guess = {}
		self.ssds_curr = {}
		self.ssds_curr_micro = {}
		self.im_num = len(self.images_dict)
		self.ind_params = ind_params

	def read_colmap_data(self):
		"""
		Reading in colmap data
		SWITCHING TO PYCOLMAP ***
		"""

		# Pycolmap version
		recon = pycolmap.Reconstruction("../datasets/KBCB_colm")
		self.images_c = recon.images
		self.cameras_c = recon.cameras
		self.pts3d_c = recon.points3D


	def image_parsing(self):
		""" 
		Gets the specific image IDs according to COLMAP file. Useful 
		for grabbing transformations later 
		Input: class
		Output: image IDs
		"""
		self.images_dict = {}
		self.im_pts_2d = {}
		self.im_mosaic = {}

		self.images = []

		im_dir = pth(self.imss) 

		if not im_dir.exists() or not im_dir.is_dir():
			raise ValueError(f"self.imss must be an existing foler path. Got: {self.imss}")
		# grab files
		files = [p for p in im_dir.iterdir() if p.is_file()]

		# sorting
		def frame_key(p: pth): 
			m = re.search(r"\d+", p.stem)
			if m is None:
				raise ValueError(f"No frame number found in filename: {p.name}")
			return int(m.group())

		# For file string
		files = sorted(files, key=frame_key)
		# full path 
		self.images = [str(p) for p in files]

		# print("CHCEKING FILE NAMES", self.images)

		im_ids = np.zeros((len(self.images)), dtype=int)
		# print(self.images_c.items())

		for i, image_path in enumerate(self.images):
			# Read images and create new image variables 
			self.read_image_files(image_path,i)
			# Grab file name on end of path
			filename = image_path.split('/')[-1]
			# print("FILENAME:",filename)
			
			# Look up corresponding ID
			for img_c_id, img_c in self.images_c.items():
				if img_c.name.startswith(filename):
					im_ids[i] = img_c_id
					break
				# else:
				# 	# print("Couldnt find?")

		self.im_ids = im_ids

		# print("DONE image parsing function")
		# print("Example image 0:")
		# print(self.images_dict[0])



	def read_image_files(self, image_path, i):
		"""
		Reads in each image file to be parsed through later
		Inputs: filename, picture ID number
		Output: variable created according to image number
		"""
		# image = cv2.imread(image_path)
		# self.images_dict[i] = image
		if os.path.exists(image_path):
			image = cv2.imread(image_path)
			if image is not None:
				self.images_dict[i] = image

	def sat_im_init(self):
		"""
		Initializing the satellite reference image and creating a cloud and RGB array
		NOTE: The image is already in grayscale. Keeping in RGB format for open3d
		Input: reference image 
		Output: 3xn array of points (z=1), and 3xn array of colors (grayscale)
		"""
		cols, rows = self.sat_ref.shape
		x, y = np.meshgrid(np.arange(rows), np.arange(cols))
		ref_pts = np.stack([x.ravel(), y.ravel(), np.ones_like(x).ravel()], axis=1)

		# print("LOOKING AT COLOR VALS\n")
		# print("This is the sat ref variable:\n", self.sat_ref)
		# print("This is the colored sat ref:\n", self.sat_ref_c)

		gray_vals = self.sat_ref.ravel().astype(np.float32)

		# print("These are the gray vals:\n", gray_vals)

		# ADDING COLOR 
		rgb_vals = self.sat_ref_c.reshape(-1, 3).astype(np.float32)[:,::-1]
		# print("These are the rgb vals unraveled:\n", rgb_vals)
		rgb_vals /= 255


		ref_rgb = np.stack([gray_vals]*3, axis=1)
		ref_rgb /= 255
		# print("This is now ref_rgb:\n", ref_rgb)
		# print("This is now ref_rgb_c:\n", rgb_vals)

		# ADDING A SHIFT TO FULL SAT IMAGE 
		ref_pts -= np.array([700,600,0])

		self.ref_pts = ref_pts
		self.ref_rgb = ref_rgb
		self.ref_rgb_c = rgb_vals

	def grab_pts(self, pts3d):
		"""
		Grabbing raw point cloud and RGB data from scene data
		"""
		# Loop through pts3d dictionary using keys
		raw_pts = [pt.xyz for pt in pts3d.values()]
		raw_rgb = [pt.color for pt in pts3d.values()]

		# Stack into numpy array 
		scene_pts =  np.vstack(raw_pts)
		rgb_data = np.vstack(raw_rgb)
		# Normalize rgb data 
		rgb_data = rgb_data/255 

		self.scene_pts = scene_pts
		self.scene_rgb = rgb_data

	def visualize_local_ims(self):
		"""
		Quick visualizer for local images
		"""
		n_imgs = self.im_num
		ncols = 5
		nrows = math.ceil(n_imgs / ncols)

		# Scale figure size: tweak these if you want bigger/smaller
		fig_w = 4 * ncols
		fig_h = 3 * nrows

		plt.figure(figsize=(fig_w, fig_h))

		for imnum in range(n_imgs):
			ax = plt.subplot(nrows, ncols, imnum + 1)

			im = self.images_dict[imnum]
			im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)

			ax.imshow(im)
			ax.axis("off")

		plt.tight_layout()
		plt.show()

	def grab_poses(self, images_c):
		"""
		Grabs initial image poses for visualizations
		Input: Image data
		Output: Poses
		"""
		poses = []

		for img_id, img in images_c.items():	
			T = img.cam_from_world()
			w2c = T.matrix()
			# convert to 4x4
			w2c = np.vstack((w2c, np.array([0, 0, 0, 1])))
			c2w = np.linalg.inv(w2c)   # camera-to-world
			poses.append(c2w)

		self.poses = np.stack(poses)



	def get_gnd_pts(self):
		"""
		Determining ground plane points
		Done through RANSAC plane estimation
		Outputs: pts_gnd_idx 
		"""
		# Build point cloud
		pcd = o3d.geometry.PointCloud()
		pcd.points = o3d.utility.Vector3dVector(self.scene_pts)  # (N,3)

		# Downsample for plane estimation
		pcd_ds = pcd.voxel_down_sample(voxel_size=0.1)

		# Plane segmentation on downsampled cloud
		plane_model, _ = pcd_ds.segment_plane(
			distance_threshold=0.05,
			ransac_n=3,
			num_iterations=1000
		)

		# Unpack plane: ax + by + cz + d = 0
		a, b, c, d = plane_model
		normal_norm = np.linalg.norm([a, b, c])

		# Distance of ALL original points to plane
		dist = np.abs(self.scene_pts @ np.array([a, b, c]) + d) / normal_norm

		# Ground mask on FULL cloud
		ground_mask = dist < 0.05
		ground_idx_scene = np.where(ground_mask)[0]

		# Sample indices INTO scene_pts
		k = 200
		np.random.seed(self.random_seed)
		if len(ground_idx_scene) > k:
			idx = np.random.choice(ground_idx_scene, k, replace=False)
		else:
			idx = ground_idx_scene

		# self.pts_gnd_idx = idx # UNCOMMENT if using own ground truth data

		return idx
	

	def inv_homog_transform(self, homog):
		""" 
		Inverting a homogeneous transformation matrix
		Inputs: homogeneous transformation matrix (4x4)
		Outputs: inverted 4x4 matrix
		"""
		# Grab rotation matrix
		R = homog[:3,:3]

		# Transpose rotation matrix 
		R_inv = R.T

		# Grab translation matrix 
		t = homog[:-1, -1]
		t = t.reshape((3, 1))
		t_inv = -R_inv @ t

		# Form new transformation matrix 
		bottom = np.array([0.0, 0.0, 0.0, 1.0]).reshape([1, 4])
		homog_inv = np.concatenate([np.concatenate([R_inv, t_inv], 1), bottom], 0)    
		# print("\n Homogeneous new = \n", homog_inv)

		return homog_inv



	def grav_SVD(self, pts_gnd):
		"""
		Getting the gravity vector for a set of points on the ground plane
		Input: Indices for the ground plane pts
		Output: Gravity vector 
		Note: potentially automate ground point process in the future 
		"""

		# Subtract centroid for SVD
		centroid = np.mean(pts_gnd, axis=0)
		centered_points = pts_gnd - centroid

		# Singular value decomposition (SVD)
		U, S, Vt = np.linalg.svd(centered_points)

		grav_vec = Vt[-1,:]

		return grav_vec

	def height_avg(self, pts_gnd, origin):
		"""
		Get the initial height of the origin above the ground plane 
		Input: Indices for the ground plane pts
		Output: Average h_0
		"""

		# Multiple h0's
		h0s = np.zeros((len(pts_gnd)))
		for i in range(len(pts_gnd)):
			h0i = np.dot(self.grav_vec, pts_gnd[i]-origin)
			h0s[i] = h0i

		# Average 
		h_0 = np.mean(h0s)

		return h_0

	def set_ref_frame(self):
		"""
		Defines a reference coordinate frame for the matching process
		Input: ground plane points 
		Output: reference frame transformation matrix
		"""
		pts_gnd_idx = self.pts_gnd_idx
		self.origin_w = np.array([0,0,0])
		self.pts_gnd = self.scene_pts[self.pts_gnd_idx]

		# Find gravity and height
		self.grav_vec = -self.grav_SVD(self.pts_gnd)
		print('Gravity vector \n', self.grav_vec)
		self.h_0 = self.height_avg(self.pts_gnd, self.origin_w)
		print('\nHeight h_0 = ', self.h_0)

		# Get focal length 
		cam_id = list(self.cameras_c.keys())[78] # TODO ***
		self.focal = self.cameras_c[cam_id].params[0]
		# print("Focal length \n", self.focal)


		# Define coordinate frame 
		z_bar = self.grav_vec 
		# TODO ****
		# Choose 2 random points to determine x and y directions for reference frame
		np.random.seed(self.random_seed)
		# print(pts_gnd_idx)
		idxs = np.random.choice(pts_gnd_idx, 2, replace=False)
		# print(f"Indices: {idxs}")
		P1, P2 = self.scene_pts[idxs[0],:], self.scene_pts[idxs[1],:]
		# P1, P2 = self.scene_pts[pts_gnd_idx[0],:], self.scene_pts[pts_gnd_idx[5],:]
		v = P2-P1

		# X Direction as ZcrossV
		x_dir = np.cross(z_bar, v)
		x_bar = x_dir/np.linalg.norm(x_dir)
		# print("\nX unit vector \n", x_bar)
		# Y Direction as ZcrossX
		y_dir = np.cross(z_bar, x_bar)
		y_bar = y_dir/np.linalg.norm(y_dir)
		# print("\nY unit vector \n", y_bar)

		# Rotation matrix 
		rotmat = np.column_stack((x_bar, y_bar, z_bar))
		# print("\nRotation Matrix\n", rotmat)
		# Translation Vector
		trans = P1.reshape([3,1])

		# Form transformation matrix 
		bottom = np.array([0.0, 0.0, 0.0, 1.0]).reshape([1,4])
		tform = np.concatenate([np.concatenate([rotmat, trans],1),bottom],0)
		# print("\nTransformation matrix to ground \n", tform)

		# Translation from ground to desired height 
		x = 0
		y = 0
		z = -1
		yaw = np.deg2rad(0)
		# Translation 2
		trans2 = np.array([x, y, z]).reshape([3,1])
		# Rotation 2
		euler_angles = [0., 0., yaw]
		rotmat2 = R.from_euler('xyz', euler_angles).as_matrix()
		tform2 = np.concatenate([np.concatenate([rotmat2, trans2],1),bottom],0)
		# print("\nTransformation from ground to desired coord frame (added a 220 deg yaw)\n", tform2)

		# Combine 
		tform_ref_frame = tform @ tform2
		self.tform_ref_frame = tform_ref_frame

		return tform_ref_frame


	def unit_vec_tform(self, pts_vec, origin, homog_t):
		"""
		Takes a set of unit vectors and transforms them according to a homogeneous transform
		Input: Unit vectors, transform 
		Output: Origin of new unit vectors, end points of new unit vectors, new unit vectors
		"""
		# Get new origin
		origin_o = np.append(origin,1).reshape(-1,1)
		origin_n = (homog_t @ origin_o)[:-1].flatten()

		# Unit vectors to homogeneous coords 
		pts_homog = np.hstack((pts_vec, np.ones((pts_vec.shape[0], 1)))).T

		# Apply transformation
		pts_trans = (homog_t @ pts_homog)[:-1].T

		# New vectors 
		pts_vec_n = pts_trans - origin_n

		return origin_n, pts_trans, pts_vec_n


	def prune_satellite_image(self):
		"""
		Hand select mask you want for satellite image, 
		Avoiding bushes and non-flat regions
		"""

		sat_im = cv2.cvtColor(self.sat_ref_c, cv2.COLOR_BGR2RGB)

		plt.imshow(sat_im)
		pts = plt.ginput(n=-1, timeout=0)  # click points, press Enter when done
		plt.close()

		self.create_mask(pts)

		return pts

	def create_mask(self, pts):
		"""
		Create mask based on points pruned
		Input: points from pruning
		"""
		self.pts_select_sat = np.array(pts)

		# Create grid
		h, w = self.sat_ref_c.shape[:2]
		x, y = np.meshgrid(np.arange(w), np.arange(h))
		points = np.vstack((x.flatten(), y.flatten())).T

		# Create mask
		path = Path(self.pts_select_sat)
		mask = path.contains_points(points)
		mask = mask.reshape((h, w)).astype(np.uint8)
		self.mask_SAT = mask

		# visualize
		plt.imshow(mask, cmap='gray')
		plt.show()

	def get_initial_params(self):
		"""
		Initial sized parameters for image patches
		"""
		im_size = self.images_dict[0].shape # Assuming all images of the same shape
		y_tot, x_tot = im_size[0], im_size[1]
		print(y_tot, x_tot, 'ytot xtot')
		# Initial crop so we aren't looking at entire image 
		x = 100
		y = 600
		width = x_tot - (2*x)
		height = y_tot - y # - 50 # USING THE BOTTOM
		param = np.array([[x,y,width,height]])
		params_tot = np.tile(param, (10,1))

		return params_tot



	def grab_image_pts_tot(self, mosaic_params):
		"""
		Grab points of an image (that we know are on ground plane)
		Based on specified starting x and y location, width, height
		Looking to automate process in future - currently manually chosen pts
		"""
		self.mosaic_params = mosaic_params
		# Loop through mosaic_params for each image
		for imnum in range(len(self.images_dict)):
			x, y, width, height = mosaic_params[imnum]

			# Create grid of coords
			x_coords = np.arange(x, x+width)
			y_coords = np.arange(y, y+height)
			Px, Py = np.meshgrid(x_coords, y_coords, indexing='xy')
			# print(Px, Py)

			# Stack points for array 
			pts_loc = np.stack((Px, Py), axis=-1) # -1 forms a new axis 
			# print(pts_loc)

			# Extract RGB
			im_gnd = self.images_dict[imnum]
			pts_rgb = im_gnd[y:y+height, x:x+width].astype(int)

			# Store in dict
			corners = np.array([x,y,width,height])
			self.im_pts_2d[imnum] = {'pts': pts_loc}
			self.im_pts_2d[imnum]['rgbc'] = pts_rgb
			self.im_pts_2d[imnum]['corners'] = corners

		# return pts_loc, pts_rgb


	def plot_gnd_pts(self):
		"""
		Plotting boxes on each local image to represent ground sections to be used in mosaic process
		Input: figure, axes
		Output: subplot with proper ground section identification 
		"""
		plt.figure(figsize=(45,10))
		rows = max(1,math.ceil(len(self.images_dict)/5))
		# Loop through each image
		for imnum in range(len(self.images_dict)):
			# if imnum == 0:
			# 	last_ax = plt.subplot(rows,5,imnum+1) # CHANGED 
			plt.subplot(rows,5,imnum+1)
			# Grab parameters 
			x, y, width, height = self.mosaic_params[imnum]
			# Draw rectangle 
			rect = plt.Rectangle((x,y), width, height, linewidth=1, edgecolor='r', facecolor='none')
			# Grab correct image based on number indicator 
			im_gnd_plt = self.images_dict[imnum]
			im_gnd_plt = cv2.cvtColor(im_gnd_plt, cv2.COLOR_BGR2RGB)
			# TESTING SOMETHING
			# im_gnd_plt = cv2.cvtColor(im_gnd_plt, cv2.COLOR_BGR2GRAY)
			# print(im_gnd_plt)

			# Plot 
			plt.imshow(im_gnd_plt)
			# TESTING SOMETHING
			# plt.imshow(im_gnd_plt, cmap='gray')
			plt.gca().add_patch(rect)
			plt.axis("off")

		# fig = plt.gcf() # CHANGED

		# # Show plot 
		# plt.savefig(
		# 	'hash_im_box.png',
		# 	dpi=300,
		# 	bbox_inches=last_ax.get_tightbbox(fig.canvas.get_renderer()).transformed(fig.dpi_scale_trans.inverted())
		# )

		plt.show()


# MASKING FOR GROUND PLANE EXTRACTION

	def patch_mask_refine(self):
		"""
		Using the satellite mask to finalize ground patches

		"""
		# Use first ground truth solution for satellite map transformation
		sat_tform_params = self.ind_params[0]
		sat_tform_inv = self.tform_create(sat_tform_params[2], sat_tform_params[3], 0, 0, 0, sat_tform_params[1])
		sat_tform = self.inv_homog_transform(sat_tform_inv)

		# Apply transformation to satellite image
		scale = sat_tform_params[0]
		sat_pts_adj = self.ref_pts.copy().astype(np.float64)
		__, sat_pts_adj_guess, sat_vec_adj_guess = self.unit_vec_tform(sat_pts_adj, self.origin_w, sat_tform)

		# Apply scale 
		sat_pts_adj_guess[:,:2] /= scale

		# Apply transformation to border points 
		pts_border = np.hstack([self.pts_select_sat, np.ones((self.pts_select_sat.shape[0], 1))]).astype(np.float64)
		pts_border -= np.array([700, 600, 0]) # Same as reference plane shift
		# Apply same transformation
		__, pts_border_transformed, __ = self.unit_vec_tform(pts_border, self.origin_w, sat_tform)
		pts_border_transformed[:, :2] /= scale

		self.pts_border_ref = pts_border_transformed
		self.sat_pts_adj = sat_pts_adj_guess

		# Mask the image 
		self.mask_sat_img()

	def mask_sat_img(self):
		"""
		Applying mask to satellite image 
		"""
		mask_flat = self.mask_SAT.ravel().astype(bool)
		self.sat_cloud_masked = self.sat_pts_adj[mask_flat]
		self.ref_rgb_masked = self.ref_rgb[mask_flat]
		self.ref_rgb_c_masked = self.ref_rgb_c[mask_flat]


	def patch_mask_crop(self, range_num):
		"""
		Crop based on satellite mask and range threshold
		Input: range threshold
		"""
		range_thresh = range_num
		self.im_mosaic_new = copy.deepcopy(self.im_mosaic)
		sat_path = Path(self.pts_border_ref[:,:2])

		for i in range(self.im_num):
			print(f"\nImage: {i}")
			ranges = self.im_pts_2d[i]['r']
			pts_o = self.im_mosaic[i]['pts']
			col_o = self.im_mosaic[i]['color_g']
			print(f"Original shape: {pts_o.shape}")

			# mask using polygon
			mask = sat_path.contains_points(pts_o[:, :2])

			# OPTIONAL***: add range mask 
			mask_range = ranges < range_thresh
			mask = mask & mask_range

			# print(mask)
			# print(mask.shape)
			new_pts = pts_o[mask]
			new_col = col_o[mask]
			print(f"Shape of new points: {new_pts.shape}")
			print(f"Shape of new colors: {new_col.shape}")

			self.im_mosaic_new[i]['pts'] = new_pts
			self.im_mosaic_new[i]['color_g'] = new_col

		self.im_mosaic = self.im_mosaic_new


	def unit_vec_c(self, imnum):
		"""
		Create unit vectors in camera frame coordinates for desired pixels 
		Using pixel location of points.
		"""
		# Get pixel locations and RGB values
		pts_loc = self.im_pts_2d[imnum]['pts']  # Shape (H, W, 2)
		# print(f'PTS location: \n{pts_loc}')
		pts_rgb = self.im_pts_2d[imnum]['rgbc']  # Shape (H, W, 3)
		im_imnum = self.images_dict[imnum]

		shape_im_y, shape_im_x = im_imnum.shape[:2]
		# print(shape_im_y, shape_im_x)
		# print("Y, X")

		# Compute shifted pixel coordinates
		# Px = pts_loc[..., 0] - shape_im_x / 2  # Shape (H, W)
		# Py = -pts_loc[..., 1] + shape_im_y / 2  # Shape (H, W)
		Px = pts_loc[..., 0] - shape_im_x / 2  # Shape (H, W)
		Py = pts_loc[..., 1] - shape_im_y / 2  # Shape (H, W)
		

		# Apply final coordinate transformations
		# Px, Py = -Py, -Px  # Swap and negate as per coordinate system
		# trying something
		Px, Py = Px, Py

		# Compute magnitude of vectors
		mag = np.sqrt(Px**2 + Py**2 + self.focal**2)  # Shape (H, W)
		self.im_pts_2d[imnum]['mag'] = mag

		# Compute unit vectors
		pts_vec_c = np.stack((Px / mag, Py / mag, np.full_like(Px, self.focal) / mag), axis=-1)  # Shape (H, W, 3)

		# Reshape into (N, 3) where N = H * W
		pts_vec_c = pts_vec_c.reshape(-1, 3)
		# pts_vec_c = pts_vec_c.reshape(-1, 3, order='F') # This would flatten by COLUMN first (top to bottom, then L to R)
		pts_rgb_gnd = pts_rgb.reshape(-1, 3) / 255  # Normalize and reshape

		return pts_vec_c, pts_rgb_gnd



	def get_pose_id(self, id,imnum):
		"""
		Get the pose transformation for a specific image id
		Input: Image ID
		Output: transform from camera to world coordinates
		"""
		# Get camera from world transformation 
		T = self.images_c[id].cam_from_world().matrix()
		# Turn into 4x4
		w2c = np.vstack((T, np.array([0,0,0,1])))
		c2w = np.linalg.inv(w2c)


		# qvec = self.images_c[id].qvec
		# tvec = self.images_c[id].tvec[:,None]
		# # print(tvec)

		# t = tvec.reshape([3,1])

		# # Create rotation matrix
		# Rotmat = qvec2rotmat(qvec) # Positive or negative does not matter
		# # print("\n Rotation matrix \n", Rotmat)

		# # Create 4x4 transformation matrix with rotation and translation
		# bottom = np.array([0.0, 0.0, 0.0, 1.0]).reshape([1, 4])
		# w2c = np.concatenate([np.concatenate([Rotmat, t], 1), bottom], 0)
		# c2w = np.linalg.inv(w2c)

		self.im_pts_2d[imnum]['w2c'] = w2c
		self.im_pts_2d[imnum]['c2w'] = c2w

		return w2c, c2w


	def unit_vec_tform(self, pts_vec, origin, homog_t):
		"""
		Takes a set of unit vectors and transforms them according to a homogeneous transform
		Input: Unit vectors, transform 
		Output: Origin of new unit vectors, end points of new unit vectors, new unit vectors
		"""
		# Get new origin
		origin_o = np.append(origin,1).reshape(-1,1)
		origin_n = (homog_t @ origin_o)[:-1].flatten()

		# Unit vectors to homogeneous coords 
		pts_homog = np.hstack((pts_vec, np.ones((pts_vec.shape[0], 1)))).T

		# Apply transformation
		pts_trans = (homog_t @ pts_homog)[:-1].T

		# New vectors 
		pts_vec_n = pts_trans - origin_n

		return origin_n, pts_trans, pts_vec_n



	def pt_range(self, pts_vec, homog_t, origin, imnum):
		"""
		Finding the range of the point which intersects the ground plane 
		Input: Unit vectors, homogeneous transform 
		Output: Range for numbers, new 3D points 
		"""

		# Get translation vector 
		t_cw = homog_t[:-1,-1]
		a = np.dot(t_cw, self.grav_vec)

		# Numerator
		num = self.h_0 - a

		# Denominator
		denom = np.dot(pts_vec, self.grav_vec)

		# Compute range
		r = num/denom
		self.im_pts_2d[imnum]['r'] = r
		self.im_pts_2d[imnum]['origin'] = origin

		# New points
		new_pts = origin + pts_vec*r[:, np.newaxis]

		return r.reshape(-1,1), new_pts


	def conv_to_gray(self, pts_rgb, imnum):
		"""
		Takes RGB values and converts to grayscale
		Uses standard luminance-preserving transformation
		Inputs: RGB values (nx3), image number 
		Outputs: grayscale values (nx3) for open3d
		"""

		# Calculate intensity value
		intensity = 0.299 * pts_rgb[:, 0] + 0.587 * pts_rgb[:, 1] + 0.114 * pts_rgb[:, 2]
		# Create nx3
		gray_colors = np.tile(intensity[:, np.newaxis], (1, 3))  # Repeat intensity across R, G, B channels
		# print(gray_colors)

		return gray_colors


	def tform_create(self,x,y,z,roll,pitch,yaw):
		"""
		Creates a transformation matrix 
		Inputs: translation in x,y,z, rotation in roll, pitch, yaw (DEGREES)
		Output: Transformation matrix (4x4)
		"""
		# Rotation
		roll_r, pitch_r, yaw_r = np.array([roll, pitch, yaw])
		euler_angles = [roll_r, pitch_r, yaw_r]
		rotmat = R.from_euler('xyz', euler_angles).as_matrix()

		# Translation
		trans = np.array([x,y,z]).reshape([3,1])

		# Create 4x4
		bottom = np.array([0.0, 0.0, 0.0, 1.0]).reshape([1,4])
		tform = np.concatenate([np.concatenate([rotmat, trans], 1), bottom], 0)
		# print("\nTransformation matrix \n", tform)

		return tform


	def implement_guess_ind(self, ind_params):
		"""
		Implementing new state estimates for INDIVIDUAL patches
		This way, we can implement an individual truth for each patch
		Inputs: ind_params: (num_ims x 4), contains scale, rot, trans(x,y)
		Outputs: transformed points for best guess 
		"""

		# Create and implement tform for each image
		# for i in range(len(self.images_dict)):
		for i in range(len(ind_params)):
			params = ind_params[i,:]
			# print("Check the params:\n", params)
			tform_guess = self.tform_create(params[2], params[3], 0, 0, 0, params[1])
			scale = params[0]
			# Grab transform points 
			loc_im_pts = self.im_mosaic[i]['pts'].copy() # Need deepcopy?
			# apply scale
			loc_im_pts[:,:2] *= scale
			# apply tform
			__, loc_im_pts_guess, loc_im_vec_guess = self.unit_vec_tform(loc_im_pts, self.origin_w, tform_guess)
			# Update best guess
			self.im_pts_best_guess[i] = {'pts': loc_im_pts_guess}


	# *************************************
	# Micropatch division and error distribution representation
	# Helper functions




	def micropatch_division(self, n):
		"""
		Dividing the current patch 'best guesses' into square nxn micropatches
		To be used for error distribution
		Input: n (side length for micropatches)
		Output: micro_ps which contains:
		1. corners for each micropatch 4x2
		2. pts from the best guess within each micro patch 
		3. color of the points within each patch
		"""
		self.micro_ps = [{} for _ in range(len(self.images_dict))]
		# Loop through each image
		for i in range(self.im_num):
			# Grab current points and corners
			pts_curr = self.im_pts_best_guess[i]['pts']
			corners = self.im_pts_2d[i]['corners']
			# Define corner indices 
			idxs = [0, -corners[2], -1, corners[2]-1]
			# Grab corner points
			pts_corners = np.array(pts_curr[idxs])

			# Create polygon of 'trapezoid' patch
			poly = Polygon(pts_corners[:,:2])

			# Create grid
			minx, miny, maxx, maxy = poly.bounds
			x_coords = np.arange(minx, maxx, n)
			y_coords = np.arange(miny, maxy, n)

			# Squares
			squares = []
			for x in x_coords:
				for y in y_coords:
					sq = box(x, y, x+n, y+n)
					# if poly.intersects(sq): # Using ones on border too
					if poly.contains(sq): # only fully inside !
						squares.append(sq)


			# Add corners of each square into micropatch directory
			for j, sq in enumerate(squares):
				cs = np.array(list(sq.exterior.coords)[:-1])
				# self.micro_ps[i][j] = {'corners': cs, 'pts': [], 'color_g'} DELETE JAWN

				# Find points within each square bounds 
				sq_minx, sq_miny, sq_maxx, sq_maxy = sq.bounds
				# print(sq_minx, sq_miny, sq_maxx, sq_maxy)

				# Mask for points and intensities
				mask = (
					(pts_curr[:,0] >= sq_minx) & (pts_curr[:,0] <= sq_maxx) &
					(pts_curr[:,1] >= sq_miny) & (pts_curr[:,1] <= sq_maxy)
				)

				# Inside points
				inside_pts = pts_curr[mask]
				colors_g = np.array(self.im_mosaic[i]['color_g'])[mask]

				self.micro_ps[i][j] = {
					'corners': cs,
					'pts': inside_pts,
					'color_g': colors_g
				}


		return self.micro_ps


	def get_inside_sat_pts(self, imnum, shiftx, shifty):
		"""
		Getting points inside the satellite image
		Input: image number, shiftx, shifty
		Output: Points inside corners from satellite image 
		"""

		# Get corners 
		corners = self.im_pts_2d[imnum]['corners']
		# Define corner indices 
		idxs = [0, -corners[2], -1, corners[2]-1]
		# print(f"IDXs: {idxs}")
		
		# Grab corner points 
		points = np.array(self.im_pts_best_guess[imnum]['pts'])[idxs]
		# Shift corners of points
		points[:,0] += shiftx
		points[:,1] += shifty
		points2d = points[:,:-1]
		# print(points2d)

		# Define polygon path 
		polygon_path = Path(points2d)
		# Points within polygon 
		mask = polygon_path.contains_points(self.ref_pts[:,:-1])
		inside_pts = self.ref_pts[mask]
		inside_cg = self.ref_rgb[mask]

		return inside_pts, inside_cg


	def grab_inside_sat_micro(self, imnum, pnum, shiftx, shifty):
		"""
		Grabbing the satellite points which correspond to the specific micropatch. 
		Shifted according to SSD shift
		Input: Image number, micropatch number, shift in x and y
		Output: Satellite points within micropatch, colors within micropatch
		"""

		# Use corners from micropatches to create a mask around the satellite image
		corners = self.micro_ps_local[imnum][pnum]['corners'].copy()
		# print("Shift in x: ", shiftx, "Shift in y: ", shifty)
		# print("Corners: \n", corners)
		corners[:,0] += shiftx
		corners[:,1] += shifty
		# print("Shift-adjusted corners: \n", corners)

		# Min and max, x and y
		minx, miny = np.min(corners[:,0]), np.min(corners[:,1])
		maxx, maxy = np.max(corners[:,0]), np.max(corners[:,1])
		# print("Max x: ", maxx, "Max y: ", maxy, "Min x: ", minx, "Min y: ", miny)
		mask = (
		    (self.ref_pts[:,0] >= minx) & (self.ref_pts[:,0] <= maxx) &
		    (self.ref_pts[:,1] >= miny) & (self.ref_pts[:,1] <= maxy)
		)
		# print("Mask: \n", mask)
		# Inside points and colors
		inside_pts = self.ref_pts[mask]
		inside_cg = self.ref_rgb[mask]
		# print("Inside points: \n", inside_pts)
		# print("Inside colors: \n", inside_cg)

		return inside_pts, inside_cg


	def dy_from_ssd_micro(self, n, imnum):
		"""
		Takes SSD values and create vectors from original position to minimum SSD location
		Inputs: n (shiftmax), imnum
		Outputs: yi (correction vectors for each micropatch), points (points of vectors)
		"""

		# Set extension pixel threshold
		extend = 5

		# Create vector from original position to minimum SSD location
		cor_vecs = np.zeros((len(self.ssds_curr_micro[imnum]), 2))
		base_vec = np.zeros((len(self.ssds_curr_micro[imnum]), 2))

		# for each micropatch 
		for mp in range(len(self.ssds_curr_micro[imnum])):
			ssds = self.ssds_curr_micro[imnum][mp]
			idrow, idcol = np.unravel_index(np.argmin(ssds), ssds.shape)
			# print("IDrow, IDcol \n", idrow, idcol)
			# Define best shift
			shiftx_min, shifty_min = idrow-n, idcol-n
			# print(f"Shift vector = {shiftx_min, shifty_min}\n")

			# Inset correction vectors 
			cor_vecs[mp] = shiftx_min, shifty_min
			# Base vector for satellite location
			sat_pts_forMean, __ = self.grab_inside_sat_micro(imnum, mp, 0, 0) # mean of sat points
			basex, basey = np.mean(sat_pts_forMean[:,0]), np.mean(sat_pts_forMean[:,1])
			base_vec[mp] = basex, basey

		# Create and stack point from vectors
		points_b = np.hstack((base_vec, np.zeros((len(self.ssds_curr_micro[imnum]), 1))))
		points_e = points_b + np.hstack((cor_vecs, np.zeros((len(self.ssds_curr_micro[imnum]), 1))))
		points = np.vstack((points_b, points_e))
		# print("\nBeginning of points: \n", points_b)
		# print("\nEnd of points: \n",points_e)
		# print("\nAll points: \n",points) # RETURNING THIS
		# print("\nCorrection Vectors: \n", cor_vecs)
		y_i = cor_vecs.reshape(-1,1)
		# print("\nCorrection Vectors reshaped: \n", y_i) # RETURNING THIS 

		print(f"Done image {imnum}")        

		# return y_i, points
		return cor_vecs, points 



# ================ SSD Processes ===================

	def ssd_nxn(self, n, imnum):
		"""
		NORMALIZED AND CLIPPED PROCESS
		New SSD process to run faster
		Sum of squared differences. Shifts around pixels 
		***Normalizing intensity values and clipping 
		Input: n shift amount, image number
		Output: sum of squared differences for each shift
		"""
		downs = 1 # Factor to downsample by 
		ssds = np.zeros((2*n+1,2*n+1))
		loc_pts = self.im_pts_best_guess[imnum]['pts'].copy()
		# print(loc_pts)

		for shiftx in range(-n,n+1):
			for shifty in range(-n, n+1):
				# Get points inside corners for satellite image 
				inside_pts, inside_cg = self.get_inside_sat_pts(imnum,shiftx,shifty)
				# print(inside_pts.shape)

				# Downsample pts (grab only x and y)
				downsampled_pts = inside_pts[::downs, :-1] # Take every 'downs'-th element
				downsampled_cg = inside_cg[::downs,0]
				# print("Colors of downsampled pts\n", downsampled_cg)

				# Shift points 
				shifted_loc_pts = loc_pts + np.array([shiftx,shifty,0])
				# print(shiftx,shifty)
				# if imnum == 0 and shiftx == -5 and shifty == -5:
					# self.CHECKER_PTS = shifted_loc_pts
					# self.CHECKER_C = inside_cg
				# print(shifted_loc_pts)

				# Build tree
				tree = cKDTree(shifted_loc_pts[:,:2])

				# Find nearest points and calculate intensities
				distances, indices = tree.query(downsampled_pts, k=1)
				nearest_intensities = self.im_mosaic[imnum]['color_g'][indices,0]
				self.ints2 = nearest_intensities
				# print("\nNearest Intensities\n", nearest_intensities)
				# print(distances, indices)

				# NORMALIZE
				# downsampled_cg -= np.mean(downsampled_cg)
				# nearest_intensities -= np.mean(nearest_intensities)
				# BEST case
				sat_bestcase = 0.5576082
				gnd_bestcase = 0.38640129
				bc_diff = sat_bestcase - gnd_bestcase
				# downsampled_cg -= sat_bestcase
				# nearest_intensities -= gnd_bestcase
				# Only subtracting difference in means 
				downsampled_cg -= bc_diff
				
				# Clip difference in means
				# 1. clip bottom (bc_diff) of satellite points
				# 2. clip top (bc_diff) of ground points
				downsampled_cg = np.maximum(downsampled_cg, 0)
				nearest_intensities = np.minimum(nearest_intensities, 1-bc_diff)



				# Calculate SSDS
				diffs = downsampled_cg - nearest_intensities 
				# print("\nDifferences\n", diffs)
				ssd_curr = np.sum(diffs**2)

				# Store SSD value for the current shift
				ssds[shiftx + n, shifty + n] = ssd_curr
				# print("SSD = ", ssd_curr)

		print(f"Number of points used for image {imnum}: ", diffs.shape)
		
		return ssds

	def ssd_nxn_micro(self, n, imnum, pnum):
		"""
		NORMALIZING AND CLIPPING STRATEGY
		Gets the SSD values for an individual micropatch within an image
		Input: n (for nxn pixel shift), image number, micro-patch number 
		***Normalizing intensity values (adjusting for runway)***
		Output: SSD for the micropatch and all nxn shifts
		"""

		# Downsample factor
		downs = 1
		ssds = np.zeros((2*n+1, 2*n+1))
		loc_pts = self.micro_ps_local[imnum][pnum]['pts'].copy()
		# print("Micropatch locations: \n", loc_pts)

		# Each nxn shift
		for shiftx in range(-n, n+1):
			for shifty in range(-n, n+1):
				# Get points inside corners for satellite image
				inside_pts, inside_cg = self.grab_inside_sat_micro(imnum, pnum, shiftx, shifty)
				# self.inside_pts = inside_pts # TESTING PURPOSES 
				# self.inside_cg = inside_cg # TESTING PURPOSES
				# Downsample points (grab only x and y)
				downsampled_pts = inside_pts[::downs, :-1] # Every 'downs'-th element
				downsampled_cg = inside_cg[::downs,0]
				# print("Color of downsampled satellite pts: \n", downsampled_cg)
				# Shift points 
				shifted_loc_pts = loc_pts + np.array([shiftx, shifty, 0])
				# print("Shifted micropatch pts: \n":, shifted_loc_pts)

				# Build tree
				tree = cKDTree(shifted_loc_pts[:,:2])

				# Find nearest points and calculate intensities
				distances, indices = tree.query(downsampled_pts, k=1)
				nearest_intensities = self.micro_ps_local[imnum][pnum]['color_g'][indices,0] # CHECK THIS 
				# print("Nearest intensities: \n", nearest_intensities)
				# self.intensity_check = nearest_intensities # TESTING PURPOSES 
				# self.pts_check = shifted_loc_pts[indices] # TESTING PURPOSES


				# NORMALIZE
				# BEST case
				sat_bestcase = 0.5576082
				gnd_bestcase = 0.38640129
				bc_diff = sat_bestcase - gnd_bestcase
				# downsampled_cg -= sat_bestcase
				# nearest_intensities -= gnd_bestcase
				# Only subtracting difference in means 
				downsampled_cg -= bc_diff
				
				# Clip difference in means
				# 1. clip bottom (bc_diff) of satellite points
				# 2. clip top (bc_diff) of ground points
				downsampled_cg = np.maximum(downsampled_cg, 0)
				nearest_intensities = np.minimum(nearest_intensities, 1-bc_diff)


				# Calculate SSDS
				diffs = downsampled_cg - nearest_intensities
				# print("Differences: \n", diffs)
				ssd_curr = np.sum(diffs**2)

				# Store ssd value for the current shift
				ssds[shiftx + n, shifty + n] = ssd_curr
				# print("SSD = ", ssd_curr)

		return ssds

















# VISUALIZER FUNCTIONS 


	def pose_scene_visualization(self, vis):
		"""
		Creating a visualization of pose estimations and sparse point cloud
		Input: vis (open3d)
		Output: vis (with pose estimations and point cloud)
		"""
		# Add origin axes
		axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
		vis.add_geometry(axes)

		# Add each pose estimate frame
		for p in self.poses:
			axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25).transform(p)
			vis.add_geometry(axes)

		# Add sparse point cloud
		scene_cloud = o3d.geometry.PointCloud()
		scene_cloud.points = o3d.utility.Vector3dVector(self.scene_pts)
		scene_cloud.colors = o3d.utility.Vector3dVector(self.scene_rgb)
		vis.add_geometry(scene_cloud)

		# # Size options (jupyter gives issues when running this multiple times, but it looks better)
		# render_option = vis.get_render_option()
		# render_option.point_size = 2

		# Run and destroy visualization 
		vis.run()
		vis.destroy_window()




	def visualize_ground_pts(self, pts_gnd_idx):
		"""
		Visualize just the ground plane points (RED) on top of the image scene
		Input: vis, ground index points
		"""
		pts_gnd = self.scene_pts[pts_gnd_idx]

		# Create Open3D visualizer object
		vis = o3d.visualization.Visualizer()
		vis.create_window(window_name='3D Plot with GROUND PLANE pts')

		# Add coordinate axes
		axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
		vis.add_geometry(axes)

		# Colmap sparse cloud
		scene_cloud = o3d.geometry.PointCloud()
		scene_cloud.points = o3d.utility.Vector3dVector(self.scene_pts)
		scene_cloud.colors = o3d.utility.Vector3dVector(self.scene_rgb)

		# Ground points test 
		gnd_cloud = o3d.geometry.PointCloud()
		gnd_cloud.points = o3d.utility.Vector3dVector(pts_gnd)
		gnd_cloud.paint_uniform_color([1.0,0,0])

		vis.add_geometry(scene_cloud)
		vis.add_geometry(gnd_cloud)

		# # Size options (jupyter gives issues when running this multiple times, but it looks better)
		# render_option = vis.get_render_option()
		# render_option.point_size = 1.5


		# Run the visualizer
		vis.run()
		vis.destroy_window()

	def visualize_ref_frame(self, scene_pts_ref):
		"""
		Visualizing the scene cloud in the reference frame
		Inputs: new points in the reference scene
		Output: open3d vis
		"""

		# Create Open3D visualizer object
		vis = o3d.visualization.Visualizer()
		vis.create_window(window_name='3D Plot in REFERENCE frame')

		# Add coordinate axes
		axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
		vis.add_geometry(axes)

		# Colmap sparse cloud
		scene_cloud = o3d.geometry.PointCloud()
		scene_cloud.points = o3d.utility.Vector3dVector(scene_pts_ref)
		scene_cloud.colors = o3d.utility.Vector3dVector(self.scene_rgb)


		vis.add_geometry(scene_cloud)

		# # Size options (jupyter gives issues when running this multiple times, but it looks better)
		# render_option = vis.get_render_option()
		# render_option.point_size = 1.5


		# Run the visualizer
		vis.run()
		vis.destroy_window()

	

	def visualize_grav_vec(self):
		"""
		Visualizer function for gravity vector
		"""
		pts_gnd = self.scene_pts[self.pts_gnd_idx]

		# Create Open3D visualizer object
		vis = o3d.visualization.Visualizer()
		vis.create_window(window_name='3D Plot with GROUND PLANE pts AND GRAVITY vector')

		# Add coordinate axes
		axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
		# vis.add_geometry(axes)

		# Colmap sparse cloud
		scene_cloud = o3d.geometry.PointCloud()
		scene_cloud.points = o3d.utility.Vector3dVector(self.scene_pts)
		scene_cloud.colors = o3d.utility.Vector3dVector(self.scene_rgb)

		# Ground points test 
		gnd_cloud = o3d.geometry.PointCloud()
		gnd_cloud.points = o3d.utility.Vector3dVector(pts_gnd)
		gnd_cloud.paint_uniform_color([1.0,0,0])

		vis.add_geometry(scene_cloud)
		vis.add_geometry(gnd_cloud)

		# # Size options (jupyter gives issues when running this multiple times, but it looks better)
		# render_option = vis.get_render_option()
		# render_option.point_size = 1.5




		# GRAVITY VECTOR 
		origin = np.array([0.0, 0.0, 0.0])  # or use centroid of points

		scale = 2.0  # make it visible
		end_point = origin + scale * self.grav_vec

		# Create line for vector
		line_set = o3d.geometry.LineSet()
		line_set.points = o3d.utility.Vector3dVector([origin, end_point])
		line_set.lines = o3d.utility.Vector2iVector([[0, 1]])

		# Color (green)
		line_set.colors = o3d.utility.Vector3dVector([[0, 1, 0]])

		vis.add_geometry(line_set)


		# Run the visualizer
		vis.run()
		vis.destroy_window()


	def visualize_im_projections(self):
		"""
		Visualizing image projections with the scene cloud
		Output: open3d visualization
		"""
		# PLOTTING THE NEW SCENE MOSAIC

		# Use open3d to create point cloud visualization 
		# Create visualization 
		vis = o3d.visualization.Visualizer()
		vis.create_window(window_name="Mosaic scene with satellite reference")

		# Create axes @ origin
		axis_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)

		# for i in specified_clouds:
		for i in range(self.im_num):
			cloud = o3d.geometry.PointCloud()
			cloud.points = o3d.utility.Vector3dVector(self.im_mosaic[i]['pts'])
			cloud.colors = o3d.utility.Vector3dVector(self.im_mosaic[i]['color_g'])
			vis.add_geometry(cloud)

		scene_cloud = o3d.geometry.PointCloud()
		scene_cloud.points = o3d.utility.Vector3dVector(self.scene_pts_ref)
		scene_cloud.colors = o3d.utility.Vector3dVector(self.scene_rgb)
		vis.add_geometry(scene_cloud)

		# Add necessary geometries to visualization 
		vis.add_geometry(axis_origin)


		# Run and destroy visualization 
		vis.run()
		vis.destroy_window()

	def visualize_im_projections_NEW(self):
		"""
		Visualizing image projections with the scene cloud
		Output: open3d visualization
		"""
		# PLOTTING THE NEW SCENE MOSAIC

		# Use open3d to create point cloud visualization 
		# Create visualization 
		vis = o3d.visualization.Visualizer()
		vis.create_window(window_name="Mosaic scene with satellite reference")

		# Create axes @ origin
		axis_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)

		# for i in specified_clouds:
		for i in range(self.im_num):
			cloud = o3d.geometry.PointCloud()
			cloud.points = o3d.utility.Vector3dVector(self.im_mosaic_new[i]['pts'])
			cloud.colors = o3d.utility.Vector3dVector(self.im_mosaic_new[i]['color_g'])
			vis.add_geometry(cloud)

		scene_cloud = o3d.geometry.PointCloud()
		scene_cloud.points = o3d.utility.Vector3dVector(self.scene_pts_ref)
		scene_cloud.colors = o3d.utility.Vector3dVector(self.scene_rgb)
		vis.add_geometry(scene_cloud)

		# Add necessary geometries to visualization 
		vis.add_geometry(axis_origin)


		# Run and destroy visualization 
		vis.run()
		vis.destroy_window()

	def mosaic_w_ref_visualization(self, vis):
		""" 
		Plotting the new scene mosaic 
		Input: vis (from open3d)
		Output: vis with mosaic (from open3d)
		"""

		# Create axes @ origin
		axis_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500)
		# vis.add_geometry(axis_origin)

		# for i in range(len(self.images_dict)):
		for i in range(self.im_num):
			cloud = o3d.geometry.PointCloud()
			cloud.points = o3d.utility.Vector3dVector(self.im_pts_best_guess[i]['pts'])
			cloud.colors = o3d.utility.Vector3dVector(self.im_mosaic[i]['color_g'])
			vis.add_geometry(cloud)

		# Create point cloud for reference cloud (satellite)
		ref_cloud = o3d.geometry.PointCloud()
		ref_cloud.points = o3d.utility.Vector3dVector(self.ref_pts)
		ref_cloud.colors = o3d.utility.Vector3dVector(self.ref_rgb)
		vis.add_geometry(ref_cloud)

		# # Size options (jupyter gives issues when running this multiple times, but it looks better)
		# render_option = vis.get_render_option()
		# render_option.point_size = 2

		# # Set up initial viewpoint
		# view_control = vis.get_view_control()
		# # Direction which the camera is looking
		# view_control.set_front([0, 0, -1])  # Set the camera facing direction
		# # Point which the camera revolves about 
		# view_control.set_lookat([0, 0, 0])   # Set the focus point
		# # Defines which way is up in the camera perspective 
		# view_control.set_up([0, -1, 0])       # Set the up direction
		# view_control.set_zoom(.45)           # Adjust zoom if necessary

		# view_control = vis.get_view_control()
		# view_control.set_lookat([0, 0, 0])
		# view_control.set_front([0, 0, -1])
		# view_control.set_up([0, -1, 0])
		# view_control.set_zoom(0.45)

		# Run and destroy visualization 
		vis.run()
		vis.destroy_window()


	def plot_traps_w_microps(self,n):
		""" 
		Use matplotlib to plot the patch trapezoids with the grids of the micropatches
		Input: grid size n
		Output: Subplot with trapezoids and micropatch grids
		"""

		num_imgs = self.im_num #len(self.images_dict) # setting to im_num 
		fig, axes = plt.subplots(1, num_imgs, figsize=(5*num_imgs, 5))

		for i in range(num_imgs):
		    ax = axes[i] if num_imgs > 1 else axes
		    
		    # Polygon bound of mosaic points 
		    # Get corners
		    pts_curr = self.im_pts_best_guess[i]['pts']
		    corners = self.im_pts_2d[i]['corners']
		    # Define corner indices 
		    idxs = [0, -corners[2], -1, corners[2]-1]
		    # Grab corner points
		    pts_corners = np.array(pts_curr[idxs])
		    # Create polygon 
		    poly = Polygon(pts_corners[:,:2])
		    # Draw the base trapezoid if you still have it
		    x, y = poly.exterior.xy
		    ax.fill(x, y, alpha=0.3, color='gray', label='Trapezoid')

		    # Draw micropatches from corners
		    for j in range(len(self.micro_ps_local[i])):
		        corners = self.micro_ps_local[i][j]['corners']
		        if corners is None or len(corners) == 0:
		            continue

		        # Close the polygon by repeating the first point
		        corners_closed = np.vstack([corners, corners[0]])
		        xs, ys = corners_closed[:, 0], corners_closed[:, 1]

		        ax.plot(xs, ys, color='blue', linewidth=0.7)

		    ax.set_aspect('equal')
		    ax.set_title(f"Image {i}: {n}x{n} grid")

		# Shared legend
		handles, labels = axes[0].get_legend_handles_labels() if num_imgs > 1 else ax.get_legend_handles_labels()
		fig.legend(handles, labels, loc='upper right')
		plt.tight_layout()
		plt.show()




	def plot_microps_w_sat(self):
		"""
		Plotting micropatches on top of satellite image
		"""

		# Use open3d to create point cloud visualization 
		# Create visualization 
		vis = o3d.visualization.Visualizer()
		vis.create_window(window_name="Mosaic scene with satellite reference")

		# Create axes @ origin
		axis_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10)
		# vis.add_geometry(axis_origin)

		# Add image patches
		# im_IM = np.array([1])
		for i in range(len(self.images_dict)):
		# for i in im_IM:
		    cloud = o3d.geometry.PointCloud()
		    cloud.points = o3d.utility.Vector3dVector(self.im_pts_best_guess[i]['pts'])
		    cloud.colors = o3d.utility.Vector3dVector(self.im_mosaic[i]['color_g'])
		    # vis.add_geometry(cloud)


		# Create point cloud for image points
		img_spec = np.array([1])
		# img_spec = np.arange(0,10)
		for i in img_spec:
		# for i in range(self.im_num):
		    for j in range(len(self.micro_ps[i])):
		        # print(j)
		        cloud_micro = o3d.geometry.PointCloud()
		        cloud_micro.points = o3d.utility.Vector3dVector(self.micro_ps[i][j]['pts'])
		        # cloud_micro.paint_uniform_color([.75, 0.001*j, 0.002*j])
		        cloud_micro.colors = o3d.utility.Vector3dVector(self.micro_ps[i][j]['color_g'])
		        vis.add_geometry(cloud_micro)



		# Create point cloud for reference cloud (satellite)
		ref_cloud = o3d.geometry.PointCloud()
		ref_cloud.points = o3d.utility.Vector3dVector(self.ref_pts)
		ref_cloud.colors = o3d.utility.Vector3dVector(self.ref_rgb)
		vis.add_geometry(ref_cloud)

		# Run and destroy visualization 
		vis.run()
		vis.destroy_window()


	def visualize_sat_cmap_scene(self):
		"""
		Visualizing image projections with the scene cloud 
		Output: open3d visualization 
		"""

		# PLOTTING THE NEW SCENE MOSAIC

		# Use open3d to create point cloud visualization 
		# Create visualization 
		vis = o3d.visualization.Visualizer()
		vis.create_window(window_name="Mosaic scene with satellite reference")

		# Create axes @ origin
		axis_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)

		scene_cloud = o3d.geometry.PointCloud()
		scene_cloud.points = o3d.utility.Vector3dVector(self.scene_pts_ref)
		scene_cloud.colors = o3d.utility.Vector3dVector(self.scene_rgb)
		vis.add_geometry(scene_cloud)

		# ADD THE SATELLTIE
		sat_cloud = o3d.geometry.PointCloud()
		sat_cloud.points = o3d.utility.Vector3dVector(self.sat_pts_adj)
		sat_cloud.colors = o3d.utility.Vector3dVector(self.ref_rgb)
		vis.add_geometry(sat_cloud)

		# Add necessary geometries to visualization 
		vis.add_geometry(axis_origin)


		# Run and destroy visualization 
		vis.run()
		vis.destroy_window()


	def visualize_sat_cmap_scene_mask(self):
		"""
		Visualizing satellite map with the colmap cloud and mask
		"""
		# PLOTTING THE NEW SCENE MOSAIC

		# Use open3d to create point cloud visualization 
		# Create visualization 
		vis = o3d.visualization.Visualizer()
		vis.create_window(window_name="Mosaic scene with satellite reference")

		# Create axes @ origin
		axis_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)

		# COLMAP scene
		scene_cloud = o3d.geometry.PointCloud()
		scene_cloud.points = o3d.utility.Vector3dVector(self.scene_pts_ref)
		scene_cloud.colors = o3d.utility.Vector3dVector(self.scene_rgb)
		vis.add_geometry(scene_cloud)

		# ADD THE SATELLTIE
		sat_cloud = o3d.geometry.PointCloud()
		sat_cloud.points = o3d.utility.Vector3dVector(self.sat_pts_adj)
		sat_cloud.colors = o3d.utility.Vector3dVector(self.ref_rgb)
		# vis.add_geometry(sat_cloud)

		# Add satellite MASK
		sat_mask = o3d.geometry.PointCloud()
		sat_mask.points = o3d.utility.Vector3dVector(self.sat_cloud_masked)
		# sat_mask.colors = o3d.utility.Vector3dVector(self.ref_rgb_masked) # Blue for now
		vis.add_geometry(sat_mask)

		# ADD BORDER pts
		sat_mask_border = o3d.geometry.PointCloud()
		sat_mask_border.points = o3d.utility.Vector3dVector(self.pts_border_ref)
		sat_mask_border.paint_uniform_color([1,0,0])
		vis.add_geometry(sat_mask_border)


		# Add necessary geometries to visualization 
		vis.add_geometry(axis_origin)


		# Run and destroy visualization 
		vis.run()
		vis.destroy_window()