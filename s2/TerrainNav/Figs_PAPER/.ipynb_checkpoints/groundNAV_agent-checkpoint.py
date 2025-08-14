import numpy as np
import cv2
import open3d as o3d 
import matplotlib.pyplot as plt
import plotly.graph_objects as go
import plotly.io as pio
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree
from matplotlib.path import Path

from colmapParsingUtils import *

class gNAV_agent:
	"""
	An agent which enhances ground navigation of aerial vehicles
	Initialization of agent 
	Inputs: Reference image (satellite), SfM solution (COLMAP), selected images
	"""
	def __init__(self, images_colm, cameras_colm, pts3d_colm, images, sat_ref):
		self.images_c_loc = images_colm		# Location of images file
		self.cameras_c_loc = cameras_colm	# Location of cameras file
		self.pts3d_c_loc = pts3d_colm 		# Location of ptd3d file
		self.images = images 				# Location of specific image files 
		self.sat_ref = cv2.imread(sat_ref) 	# Satellite reference image
		self.sat_ref = cv2.cvtColor(self.sat_ref, cv2.COLOR_BGR2GRAY)
		self.read_colmap_data()
		self.image_parsing()
		self.sat_im_init()
		self.im_pts_best_guess = {}
		self.ssds_curr = {}
		self.ssds1_curr = {}


	def read_colmap_data(self):
		self.images_c = read_images_text(self.images_c_loc)
		self.cameras_c = read_cameras_text(self.cameras_c_loc)
		self.pts3d_c = read_points3D_text(self.pts3d_c_loc)


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
		im_ids = np.zeros((len(self.images)), dtype=int)
		# print(self.images_c.items())

		for i, image_path in enumerate(self.images):
			# Read images and create new image variables 
			self.read_image_files(image_path,i)
			# Grab file name on end of path
			filename = image_path.split('/')[-1]
			# print(filename)
			
			# Look up corresponding ID
			for img_c_id, img_c in self.images_c.items():
				if img_c.name.startswith(filename):
					im_ids[i] = img_c_id
					break

		self.im_ids = im_ids

	def read_image_files(self, image_path, i):
		"""
		Reads in each image file to be parsed through later
		Inputs: filename, picture ID number
		Output: variable created according to image number
		"""
		image = cv2.imread(image_path)
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

		gray_vals = self.sat_ref.ravel().astype(np.float32)
		ref_rgb = np.stack([gray_vals]*3, axis=1)
		ref_rgb /= 255

		# ADDING A SHIFT TO FULL SAT IMAGE - comment out for old version
		ref_pts -= np.array([700,600,0])

		self.ref_pts = ref_pts
		self.ref_rgb = ref_rgb


		# cols = self.sat_ref.shape[0]
		# rows = self.sat_ref.shape[1]
		# # print(cols,rows)
		# n = cols*rows 
		# ref_pts = np.zeros((n,3))
		# ref_rgb = np.zeros((n,3))
		# count = 0
		# for i in range(cols):
		# 	for j in range(rows):
		# 		ref_pts[count] = [j,i,1]
		# 		ref_rgb[count] = self.sat_ref[i][j]
		# 		count += 1

		# ref_rgb /= 255
		# self.ref_pts = ref_pts
		# self.ref_rgb = ref_rgb
		# self.tree = cKDTree(ref_pts)



	def grab_pts(self, pts3d):
		"""
		Grabbing raw point cloud and RGB data from scene data
		"""
		# Loop through pts3d dictionary using keys
		raw_pts = [pts3d[key].xyz for key in pts3d.keys()]
		raw_rgb = [pts3d[key].rgb for key in pts3d.keys()]

		# Stack into numpy array 
		scene_pts =  np.vstack(raw_pts)
		rgb_data = np.vstack(raw_rgb)
		# Normalize rgb data 
		rgb_data = rgb_data/255 

		self.scene_pts = scene_pts
		self.rgb_data = rgb_data

		return scene_pts, rgb_data

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


	def set_ref_frame(self, pts_gnd_idx):
		"""
		Defines a reference coordinate frame for the matching process
		Input: ground plane points 
		Output: reference frame transformation matrix
		"""
		self.origin_w = np.array([0,0,0])
		self.pts_gnd = self.scene_pts[pts_gnd_idx]

		# Find gravity and height
		self.grav_vec = self.grav_SVD(self.pts_gnd)
		# print('Gravity vector \n', self.grav_vec)
		self.h_0 = self.height_avg(self.pts_gnd, self.origin_w)
		# print('\nHeight h_0 = ', self.h_0)

		# Get focal length 
		cam_id = list(self.cameras_c.keys())[0]
		self.focal = self.cameras_c[cam_id].params[0]
		# print("Focal length \n", self.focal)


		# Define coordinate frame 
		z_bar = self.grav_vec
		P1, P2 = self.scene_pts[pts_gnd_idx[0],:], self.scene_pts[pts_gnd_idx[5],:]
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
		yaw = np.deg2rad(220) # CHANGED FROM 220
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


	def set_ref_frame_mid(self, pts_gnd_idx):
		"""
		Defines a reference coordinate frame for the matching process
		Input: ground plane points 
		Output: reference frame transformation matrix
		"""
		self.origin_w = np.array([0,0,0])
		self.pts_gnd = self.scene_pts[pts_gnd_idx]

		# Find gravity and height
		self.grav_vec = self.grav_SVD(self.pts_gnd)
		# print('Gravity vector \n', self.grav_vec)
		self.h_0 = self.height_avg(self.pts_gnd, self.origin_w)
		# print('\nHeight h_0 = ', self.h_0)

		# Get focal length 
		cam_id = list(self.cameras_c.keys())[0]
		self.focal = self.cameras_c[cam_id].params[0]
		# print("Focal length \n", self.focal)


		# Define coordinate frame 
		z_bar = self.grav_vec
		P1, P2 = self.scene_pts[pts_gnd_idx[0],:], self.scene_pts[pts_gnd_idx[5],:]
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
		y = -6
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

	def inv_homog_transform(self, homog):
		""" Inverting a homogeneous transformation matrix
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


	def unit_vec_tform(self, pts_vec, origin, homog_t):
		"""
		Takes a set of unit vectors and transforms them according to a homogeneous transform
		**** TRYING TO SPEED UP WITH NEW VERSION ****
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

	def proj_2d_scene(self, pts):
		"""
		Take a 3d scene and returns a projection of those 3D points in 2D
		NOTE: z-value will be equal to 1
		Input: 3D points 
		Output: 2D points (where z=1)
		"""

		pts_2D = pts / pts[:, 2][:, np.newaxis]

		return pts_2D


	def plot_rect_im(self, x, y, width, height, imnum):
		"""
		Plotting a rectangle over the desired area of choice for ground plane point 
		Input: x and y starting point, width and height crop size, image number
		Output: plot with cropped section in red
		Specified by user with x, y, width, height 
		"""
		# Make figure 
		fig, ax = plt.subplots(figsize=(15,8))
		# Draw rectangle 
		rect = plt.Rectangle((x,y), width, height, linewidth=3, edgecolor='r', facecolor='none')
		# Grab correct image based on number indicator 
		im_gnd_plt = self.images_dict[imnum]
		im_gnd_plt = cv2.cvtColor(im_gnd_plt, cv2.COLOR_BGR2RGB)

		# print(im_gnd_plt)

		# Plot 
		ax.imshow(im_gnd_plt)
		ax.add_patch(rect)
		ax.axis("off")

		# Show plot 
		plt.show()


	def grab_image_pts(self, x, y, width, height, imnum):
		"""
		Grab points of an image (that we know are on ground plane)
		Based on specified starting x and y location, width, height
		Potentially automate process in future 
		"""

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

		return pts_loc, pts_rgb


	def unit_vec_c(self, imnum):
		"""
		Create unit vectors in camera frame coordinates for desired pixels 
		Using pixel location of points.
		"""
		# Get pixel locations and RGB values
		pts_loc = self.im_pts_2d[imnum]['pts']  # Shape (H, W, 2)
		# print(pts_loc)
		pts_rgb = self.im_pts_2d[imnum]['rgbc']  # Shape (H, W, 3)
		im_imnum = self.images_dict[imnum]

		shape_im_y, shape_im_x = im_imnum.shape[:2]

		# Compute shifted pixel coordinates
		Px = pts_loc[..., 0] - shape_im_x / 2  # Shape (H, W)
		Py = -pts_loc[..., 1] + shape_im_y / 2  # Shape (H, W)

		# Apply final coordinate transformations
		Px, Py = -Py, -Px  # Swap and negate as per coordinate system

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
		# Get quaternion and translation vector
		qvec = self.images_c[id].qvec
		tvec = self.images_c[id].tvec[:,None]
		# print(tvec)

		t = tvec.reshape([3,1])

		# Create rotation matrix
		Rotmat = qvec2rotmat(qvec) # Positive or negative does not matter
		# print("\n Rotation matrix \n", Rotmat)

		# Create 4x4 transformation matrix with rotation and translation
		bottom = np.array([0.0, 0.0, 0.0, 1.0]).reshape([1, 4])
		w2c = np.concatenate([np.concatenate([Rotmat, t], 1), bottom], 0)
		c2w = np.linalg.inv(w2c)

		# self.im_pts_2d[imnum]['w2c'] = w2c
		# self.im_pts_2d[imnum]['c2w'] = c2w

		return w2c, c2w


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
		# print(idxs)
		
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
		

	def ssd_nxn(self, n, imnum):
		"""
		New SSD process to run faster
		Sum of squared differences. Shifts around pixels 
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
				# print(shifted_loc_pts)

				# Build tree
				tree = cKDTree(shifted_loc_pts[:,:2])

				# Find nearest points and calculate intensities
				distances, indices = tree.query(downsampled_pts, k=1)
				nearest_intensities = self.im_mosaic[imnum]['color_g'][indices,0]
				self.ints2 = nearest_intensities
				# print("\nNearest Intensities\n", nearest_intensities)
				# print(distances, indices)

				# Calculate SSDS
				diffs = downsampled_cg - nearest_intensities 
				# print("\nDifferences\n", diffs)
				ssd_curr = np.sum(diffs**2)

				# Store SSD value for the current shift
				ssds[shiftx + n, shifty + n] = ssd_curr
				# print("SSD = ", ssd_curr)

		print(f"Number of points used for image {imnum}: ", diffs.shape)
		
		return ssds



	def ssd_nxn_NEWnew(self, n, imnum):
		"""
		New SSD process to run faster
		New lookup process instead of using the trees*****
		Sum of squared differences. Shifts around pixels 
		Input: n shift amount, image number
		Output: sum of squared differences for each shift
		"""
		self.check_pts = {} # ***JUST AS A CHECK, DELETE WHEN FIGURED OUT
		self.check_pts_sat = {} # ***JUST AS A CHECK, DELETE WHEN FIGURED OUT 
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
				downsampled_pts = inside_pts[::downs]#, :-1] # Take every 'downs'-th element
				downsampled_cg = inside_cg[::downs,0]
				self.check_pts_sat[shiftx, shifty] = downsampled_pts # ***JUST AS A CHECK, DELETE WHEN FIGURED OUT
				# print("Colors of downsampled pts\n", downsampled_cg)
				# print(downsampled_cg.shape)

				# Shift points 
				shifted_loc_pts = loc_pts + np.array([shiftx,shifty,0])
				# print(shiftx,shifty)
				# print(shifted_loc_pts.shape)

				# USE THESE POINTS TO NOW GO BACK TO AN IMAGE PT 
				# This is as a sanity check
				# For the real implementation, we are moving the satellite points (downsampled_pts) ...
				# ... through this process
				# print("Shifted local points\n", shifted_loc_pts)

				# 1. Inverse of current guess - VERIFIED TO MATCH BEFORE INIT_G IMPLEMENTATION
				# print(self.best_guess_tform)
				best_guess_inv = np.linalg.inv(self.best_guess_tform)
				# print(best_guess_inv)
				# __, loc_pts_ref, __ = self.unit_vec_tform(shifted_loc_pts, self.origin_w, best_guess_inv)
				__, loc_pts_ref, __ = self.unit_vec_tform(downsampled_pts, self.origin_w, best_guess_inv)
				loc_pts_ref[:, :2] /= self.best_guess_scale
				# print("\nShould be the local points in ref frame\n", loc_pts_ref)

				# 2. Ref plane to world coords
				__, loc_pts_wrd, __ = self.unit_vec_tform(loc_pts_ref, self.origin_w, self.tform_ref_frame)
				# print("\nShould be the local points in world cords\n", loc_pts_wrd)


				# 3. World coords to camera coords 
				# print("\nImage specific transformation\n", self.im_pts_2d[imnum]['w2c'])
				__, loc_pts_cam, __ = self.unit_vec_tform(loc_pts_wrd, self.origin_w, self.im_pts_2d[imnum]['w2c'])


				# 4. 3d camera pts --> 2d camera points
				z = loc_pts_cam[:,2]
				pts_2d = np.stack((loc_pts_cam[:,0]*self.focal/z, loc_pts_cam[:,1]*self.focal/z), axis=1)
				# print("\n2D points\n", pts_2d)

				# 5. Shift to proper coords 
				px, py = pts_2d[:,0], pts_2d[:,1]
				# Shift
				PY = -px
				PX = -py
				# Re-center
				Px = PX + (self.images_dict[imnum].shape[1])/2
				Py = -PY + (self.images_dict[imnum].shape[0])/2
				pts_2d = np.stack((Px, Py), axis=1) 
				# pts_2d = np.reshape(self.im_pts_2d[imnum]['corners'][3],self.im_pts_2d[imnum]['corners'][2],2)

				# 6. Round pixel locs - ENDING HERE 
				# print("\nNon-rounded points\n", pts_2d)
				pts_2d = np.round(pts_2d).astype(int)
				# print("\nRounded points\n", pts_2d)
				self.check_pts[shiftx, shifty] = pts_2d # ***JUST AS A CHECK, DELETE WHEN FIGURED OUT

				# 7. Get intensities from the 2D camera coords
				x, y = pts_2d[...,0], pts_2d[...,1]
				# print(x,y)
				# print(x[1965],y[1965])
				# im = cv2.cvtColor(self.images_dict[imnum], cv2.COLOR_BGR2RGB)
				rgbvals = self.images_dict[imnum][y,x]
				rgbvals = rgbvals.astype(np.float32)
				rgbvals /= 255 # Normalize
				print(rgbvals)
				# print("\nRgbvals\n", rgbvals)
				# Flatten to (N, 3)
				# rgb_flat = rgbvals.reshape(-1, 3)
				# print("\nRGBFLAT\n", rgb_flat)
				# Apply intensity formula
				intensity = (0.299 * rgbvals[:, 0] +
					0.587 * rgbvals[:, 1] +
					0.114 * rgbvals[:, 2])#.reshape(-1, 1)

				print("\nNearest Intensities\n", intensity)
				self.ints1 = intensity

				# 8. Calculate SSDS
				diffs = downsampled_cg - intensity 
				# print("\nDiffs\n", diffs)
				ssd_curr = np.sum(diffs**2)

				# Store SSD value for the current shift
				ssds[shiftx + n, shifty + n] = ssd_curr
				# print("SSD = ", ssd_curr)

		print("Number of points used: ", diffs.shape)

		return ssds


# FUNCTIONS FOR LSQUARES PROCESS 
# 1. Generate deltaY term from SSDs (input: self, n) (output: yi)
# 2. Form big jacobian J (input: self, parameters_best guess), (output: J)
# 3. Least squares dalpha (input: self, J, yi) (output: dalpha, new params)
# 4. Apply change to points (input: self, params) (output: "Done")






# NOTES: LETS CHECK THE ORDER OF THE INTENSITIES 
# They may be the right intensities, but in the wrong order
# List the first 10, then plot them 
# NO
# START WITH A FULL BLUE SECTION 
# See if the intensities are the same between the two methods (they should be)


# The issue is within the shift, trend is off, but SSD for zero shift seems close enough





























# DELETE WHEN DONE
	def ssd_nxn_NEWL(self, n, imnum):
		"""
	  *** DID NOT WORK, KEEPING HERE FOR REFERENCE ***
		New SSD process to run faster
		New lookup process instead of using the trees*****
		Sum of squared differences. Shifts around pixels 
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
				downsampled_pts = inside_pts[::downs]#, :-1] # Take every 'downs'-th element
				downsampled_cg = inside_cg[::downs,0]
				print(downsampled_pts.shape)

				# Shift points 
				shifted_loc_pts = loc_pts + np.array([shiftx,shifty,0])
				# print(shiftx,shifty)
				print(shifted_loc_pts.shape)

				# USE THESE POINTS TO NOW GO BACK TO AN IMAGE PT 
				# This is as a sanity check
				# For the real implementation, we are moving the satellite points (downsampled_pts) ...
				# ... through this process
				# print("Shifted local points\n", shifted_loc_pts)

				# Inverse of current guess - VERIFIED TO MATCH BEFORE INIT_G IMPLEMENTATION
				# print(self.best_guess_tform)
				best_guess_inv = np.linalg.inv(self.best_guess_tform)
				# print(best_guess_inv)
				# __, loc_pts_ref, __ = self.unit_vec_tform(shifted_loc_pts, self.origin_w, best_guess_inv)
				__, loc_pts_ref, __ = self.unit_vec_tform(downsampled_pts, self.origin_w, best_guess_inv)
				loc_pts_ref[:, :2] /= self.best_guess_scale
				# print("\nShould be the local points in ref frame\n", loc_pts_ref)

				# Ref plane to world coords
				__, loc_pts_wrd, __ = self.unit_vec_tform(loc_pts_ref, self.origin_w, self.tform_ref_frame)
				print("\nShould be the local points in world cords\n", loc_pts_wrd)

				# World coords to world unit vectors
				# print(self.im_pts_2d[imnum]['r'][:, np.newaxis])
				pts_vec_w = (loc_pts_wrd - self.im_pts_2d[imnum]['origin']) / self.im_pts_2d[imnum]['r'][:, np.newaxis]
				# print("\nShould be the local points in world unit vecs\n", pts_vec_w)

				# World unit vectors to camera unit vectors
				__, __, loc_vec_cam = self.unit_vec_tform(pts_vec_w, self.origin_w, self.im_pts_2d[imnum]['w2c'])
				# print("\nShould be the local unit vecs in cam coords\n", loc_vec_cam)

				# Camera unit vectors --> 2D camera coords
				# Go to camera space 
				mag_flat = self.im_pts_2d[imnum]['mag'].reshape(-1)
				# print("Mags", mag_flat)
				locs = loc_vec_cam * mag_flat[:, None]
				print("\nLOCS\n", locs)
				px, py = locs[:,0], locs[:,1]
				# # Pixel shifts
				PY = -px
				PX = -py
				# Re-center
				Px = PX + (self.images_dict[imnum].shape[1])/2
				Py = -PY + (self.images_dict[imnum].shape[0])/2
				pts_2d = np.stack((Px, Py), axis=1)
				# pts_2d = pts_2d.reshape(self.im_pts_2d[imnum]['mag'].shape[0], self.im_pts_2d[imnum]['mag'].shape[1], 2)
				pts_2d = np.round(pts_2d).astype(int)
				print("\nHopefully the 2d points\n", pts_2d)
				# print("\nShould match this\n", self.im_pts_2d[imnum]['pts'])

				# Get intensities from the 2D camera coords
				x, y = pts_2d[...,0], pts_2d[...,1]
				# print(x,y)
				rgbvals = self.images_dict[imnum][y,x].astype(np.float32)
				rgbvals /= 255 # Normalize
				
				# Flatten to (N, 3)
				rgb_flat = rgbvals.reshape(-1, 3)
				# print("\nrgbvals\n", rgb_flat.shape)
				# Apply intensity formula
				intensity = (0.299 * rgb_flat[:, 0] +
					0.587 * rgb_flat[:, 1] +
					0.114 * rgb_flat[:, 2]).reshape(-1, 1)

				print("\nIntensities\n", intensity)

				# # Build tree
				# tree = cKDTree(shifted_loc_pts[:,:2])

				# # Find nearest points and calculate intensities
				# distances, indices = tree.query(downsampled_pts, k=1)





				
	# 			# # nearest_intensities = self.im_mosaic[imnum]['color_g'][indices,0]
	# 			# print(distances, indices)

	# 			# # Calculate SSDS
	# 			# diffs = downsampled_cg - nearest_intensities 
	# 			# ssd_curr = np.sum(diffs**2)

	# 			# # Store SSD value for the current shift
	# 			# ssds[shiftx + n, shifty + n] = ssd_curr
	# 			# print("SSD = ", ssd_curr)

	# 	# print("Number of points used: ", diffs.shape)

	# 	return ssds