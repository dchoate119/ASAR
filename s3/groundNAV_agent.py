import numpy as np
import cv2
import open3d as o3d 
import matplotlib.pyplot as plt
import plotly.graph_objects as go
import plotly.io as pio
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree
from matplotlib.path import Path
import os 
from mpl_toolkits.mplot3d import Axes3D


from colmapParsingUtils import *

class gNAV_agent:
	"""
	Project for enhancing autonomous ground navigation of aerial vehicles
	Initialization of agent 
	Inputs: Reference image (satellite), SfM solution (COLMAP), selected images
	"""
	def __init__(self, images_colm, cameras_colm, pts3d_colm, images, sat_ref):
		self.images_c_loc = images_colm		# Location of images file
		self.cameras_c_loc = cameras_colm	# Location of cameras file
		self.pts3d_c_loc = pts3d_colm 		# Location of ptd3d file
		self.imss = images 					# Location of specific image folder 
		self.sat_ref = cv2.imread(sat_ref) 	# Satellite reference image
		self.sat_ref = cv2.cvtColor(self.sat_ref, cv2.COLOR_BGR2GRAY)
		self.read_colmap_data()
		self.image_parsing()
		self.sat_im_init()
		# Initial scene points and RGB data
		self.grab_pts(self.pts3d_c)
		# Initialize best guess and SSDs
		self.im_pts_best_guess = {}
		self.ssds_curr = {}
		# self.ssds1_curr = {}
		# Ground plane points - chosen MANUALLY
		self.pts_gnd_idx = np.array([25440, 25450, 25441, 25449, 25442, 25445, 103922, 103921, 103919, 103920])

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

		# Specify image ordering 
		self.im1 = self.imss + '/IMG_9475.JPEG'
		self.im2 = self.imss + '/IMG_9464.JPEG'
		self.im3 = self.imss + '/IMG_9467.JPEG'
		self.im4 = self.imss + '/IMG_9473.JPEG'
		self.im5 = self.imss + '/IMG_9476.JPEG'
		self.im6 = self.imss + '/IMG_9446.JPEG'
		self.im7 = self.imss + '/IMG_9520.JPEG'
		self.im8 = self.imss + '/IMG_9531.JPEG'
		self.im9 = self.imss + '/IMG_9542.JPEG'
		self.im10 = self.imss + '/IMG_9576.JPEG'

		self.images = [self.im1, self.im2, self.im3, self.im4, self.im5,
		self.im6, self.im7, self.im8, self.im9, self.im10]

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

		gray_vals = self.sat_ref.ravel().astype(np.float32)
		ref_rgb = np.stack([gray_vals]*3, axis=1)
		ref_rgb /= 255

		# ADDING A SHIFT TO FULL SAT IMAGE - comment out for old version
		ref_pts -= np.array([700,600,0])

		self.ref_pts = ref_pts
		self.ref_rgb = ref_rgb

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
		self.scene_rgb = rgb_data

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
		self.grab_poses(self.images_c)
		for p in self.poses:
			axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1).transform(p)
			vis.add_geometry(axes)

		# Add sparse point cloud
		scene_cloud = o3d.geometry.PointCloud()
		scene_cloud.points = o3d.utility.Vector3dVector(self.scene_pts)
		scene_cloud.colors = o3d.utility.Vector3dVector(self.scene_rgb)
		vis.add_geometry(scene_cloud)

		# Size options (jupyter gives issues when running this multiple times, but it looks better)
		render_option = vis.get_render_option()
		render_option.point_size = 2

		# Run and destroy visualization 
		vis.run()
		vis.destroy_window()

	def grab_poses(self, images_c):
		"""
		Grabs initial image poses for visualizations
		Input: Image data
		Output: Poses
		"""
		poses = []
		# Loop through each image
		for i in images_c:
			# Get quaternion and translation vector
			qvec = images_c[i].qvec
			tvec = images_c[i].tvec[:,None]
			# print(tvec)
			t = tvec.reshape([3,])
			# Create rotation matrix
			Rotmat = qvec2rotmat(qvec) # Positive or negative does not matter
			# print("\n Rotation matrix \n", Rotmat)
			# Create 4x4 transformation matrix with rotation and translation
			tform_mat = np.eye(4)
			tform_mat[:3, :3] = Rotmat
			tform_mat[:3, 3] = t
			w2c = tform_mat
			c2w = np.linalg.inv(w2c)
			poses.append(c2w)
		poses = np.stack(poses)
		self.poses = poses


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
		self.pts_gnd = self.scene_pts[self.pts_gnd_idx]

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


	def pose_scene_visualization_ref(self, vis, scene_pts_ref):
		"""
		Creating a visualization of pose estimations and sparse point cloud
		Input: vis (open3d)
		Output: vis (with pose estimations and point cloud)
		"""
		# Add origin axes
		axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
		vis.add_geometry(axes)

		# Add sparse point cloud
		scene_cloud = o3d.geometry.PointCloud()
		scene_cloud.points = o3d.utility.Vector3dVector(scene_pts_ref)
		scene_cloud.colors = o3d.utility.Vector3dVector(self.scene_rgb)
		vis.add_geometry(scene_cloud)

		# Size options (jupyter gives issues when running this multiple times, but it looks better)
		render_option = vis.get_render_option()
		render_option.point_size = 2

		# Run and destroy visualization 
		vis.run()
		vis.destroy_window()


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
		rect = plt.Rectangle((x,y), width, height, linewidth=1, edgecolor='r', facecolor='none')
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
		Looking to automate process in future - currently manually chosen pts
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

		# return pts_loc, pts_rgb

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
		rows = int(len(self.images_dict)/5)
		# Loop through each image
		for imnum in range(len(self.images_dict)):
			plt.subplot(rows,5,imnum+1)
			# Grab parameters 
			x, y, width, height = self.mosaic_params[imnum]
			# Draw rectangle 
			rect = plt.Rectangle((x,y), width, height, linewidth=1, edgecolor='r', facecolor='none')
			# Grab correct image based on number indicator 
			im_gnd_plt = self.images_dict[imnum]
			im_gnd_plt = cv2.cvtColor(im_gnd_plt, cv2.COLOR_BGR2RGB)
			# print(im_gnd_plt)

			# Plot 
			plt.imshow(im_gnd_plt)
			plt.gca().add_patch(rect)
			plt.axis("off")

		# Show plot 
		plt.show()

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

		self.im_pts_2d[imnum]['w2c'] = w2c
		self.im_pts_2d[imnum]['c2w'] = c2w

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


	def mosaic_visualization(self, vis):
		""" 
		Plotting the new scene mosaic 
		Input: vis (from open3d)
		Output: vis with mosaic (from open3d)
		"""

		# Create axes @ origin
		axis_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
		vis.add_geometry(axis_origin)

		for i in range(len(self.images_dict)):
			cloud = o3d.geometry.PointCloud()
			cloud.points = o3d.utility.Vector3dVector(self.im_mosaic[i]['pts'])
			cloud.colors = o3d.utility.Vector3dVector(self.im_mosaic[i]['color_g'])
			vis.add_geometry(cloud)

		# Run and destroy visualization 
		vis.run()
		vis.destroy_window()


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

	def implement_guess(self, tform_guess, scale):
		"""
		Implementing the newest state estimate guess for mosaic
		Input: transformation guess, scaling factor
		Output: transformed points for best guess
		"""

		# Implement tform for each image
		for i in range(len(self.images_dict)):
			loc_im_pts = self.im_mosaic[i]['pts'].copy()
			# apply scale
			loc_im_pts[:,:2] *= scale
			# apply tform
			__, loc_im_pts_guess, loc_im_vec_guess = self.unit_vec_tform(loc_im_pts, self.origin_w, tform_guess)
			# Update best guess
			self.im_pts_best_guess[i] = {'pts': loc_im_pts_guess}

	def update_guess(self, tform_guess, scale):
		"""
		Implementing the newest state estimate guess for mosaic
		Input: transformation guess, scaling factor
		Output: transformed points for best guess
		"""

		# Implement tform for each image
		for i in range(len(self.images_dict)):
			loc_im_pts = self.im_mosaic[i]['pts'].copy()
			# apply scale
			loc_im_pts[:,:2] *= scale
			# apply tform
			__, loc_im_pts_guess, loc_im_vec_guess = self.unit_vec_tform(loc_im_pts, self.origin_w, tform_guess)
			# Update best guess
			self.im_pts_best_guess[i]['pts'] = loc_im_pts_guess


	def mosaic_w_ref_visualization(self, vis):
		""" 
		Plotting the new scene mosaic 
		Input: vis (from open3d)
		Output: vis with mosaic (from open3d)
		"""

		# Create axes @ origin
		axis_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
		vis.add_geometry(axis_origin)

		for i in range(len(self.images_dict)):
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

		# Run and destroy visualization 
		vis.run()
		vis.destroy_window()

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


	def dy_from_ssd(self, n):
		"""
		Takes SSD values and creates vectors from original position to minimum SSD location 
		Inputs: n (shift max)
		Outputs: yi (to be used for jacobian)
		"""
		# Set extension pixel threshold
		extend = 10
		# Create vector from original position to minimum SSD location
		cor_vecs = np.zeros((len(self.images_dict), 2))
		base_vec = np.zeros((len(self.images_dict), 2))
		for im_cv in range(len(self.images_dict)):
			# Grabb SSDs - get ID of the minimum
			ssds = self.ssds_curr[im_cv]
			idrow, idcol = np.unravel_index(np.argmin(ssds), ssds.shape)
			# print("\nidrow, idcol\n", idrow, idcol)
			# Define best shift vector
			shiftx_min, shifty_min = idrow-n, idcol-n
			# # CHECK IF MIN SSD IS ON EDGE - leave out for now
			# if shiftx_min == n or shifty_min == n:
			# 	print(f"Need to extend search of image {im_cv}:\n")
			# 	print(f"Current shift vector = {shiftx_min, shifty_min}\n")
			# 	ssds = gnav.ssd_nxn(n+extend, im_cv)
			# 	gnav.ssds_curr[im_cv] = ssds
			# 	idrow, idcol = np.unravel_index(np.argmin(ssds), ssds.shape)
			# 	shiftx_min = idrow-(n+extend)
			# 	shifty_min = idcol-(n+extend)
			# 	print(f"\n New shift vector = {shiftx_min, shifty_min}\n")

			cor_vecs[im_cv] = shiftx_min, shifty_min
			sat_pts, __ = self.get_inside_sat_pts(im_cv, 0,0)
			basex, basey = np.mean(sat_pts[:,0]), np.mean(sat_pts[:,1])
			base_vec[im_cv] = basex, basey

		# Create and stack point from vectors 
		points_b = np.hstack((base_vec, np.zeros((len(self.images_dict), 1))))
		points_e = points_b + np.hstack((cor_vecs, np.zeros((len(self.images_dict), 1))))
		points = np.vstack((points_b, points_e))
		# print("\nBeginning of points: \n", points_b)
	    # print("\nEnd of points: \n",points_e)
	    # print("\nAll points: \n",points)

		y_i = cor_vecs.reshape(-1,1)
		# print("\nYi\n", y_i)

		return y_i


	def form_jacobian(self, parameters_best_guess):
		"""
		Forms jacobian matrix of for change across all patches 
		Input: Current best guess parameters (scale, theta, dx, dy)
		Output: Full Jacobian J (dim = len(images)*2 x 4)
		"""
		# Parameters are the current best guess
		params = parameters_best_guess
		# Form Jacobian for each image
		J = np.zeros((2*len(self.images_dict),4))
		theta = params[1][0]
		s = params[0][0]
		for i_m in range(len(self.images_dict)):
			xpi = np.mean(self.im_pts_best_guess[i_m]['pts'][:,0])
			xqi = np.mean(self.im_pts_best_guess[i_m]['pts'][:,1])
			# Jacobian value (based on derived state eqns)
			j11 = np.cos(theta)*xpi - np.sin(theta)*xqi
			j21 = np.sin(theta)*xpi + np.cos(theta)*xqi
			j12 = -params[0][0]*(np.sin(theta)*xpi + np.cos(theta)*xqi)
			j22 = params[0][0]*(np.cos(theta)*xpi - np.sin(theta)*xqi)
			j13, j23, j14, j24 = 1, 0, 0, 1

			# Construct Jacobian 
			J_upper = np.hstack((j11,j12,j13,j14)) # First row block (y_p terms)
			J_lower = np.hstack((j21,j22,j23,j24)) # Second row block (y_q terms)
			j = np.vstack((J_upper, J_lower))

			#Insert in proper index 
			J[2*(i_m):2*(i_m)+2, :] = j

		print("\nJACOBIAN\n", J)


		return J

	def param_change(self, J, y_i):
		"""
		Parameter update step for each iteration
		Input: Jacobian, delatY
		Output: change in params, new params
		"""
		JTJi = np.linalg.inv(J.T@J)
		Dalpha = JTJi@J.T@y_i

		return Dalpha

	def mapmatch_lsquares(self, n, iterations, params_best_guess):
		"""
		Implementing full least squares process for map matching
		Inputs: n (pixel shift mag), iterations, initial guess
		Output: Updated parameters (s, yaw, tp tq)
		"""
		self.params_best_guess = params_best_guess
		# Loop based on iterations
		for iter_idx in range(iterations):
			for imnum in range(len(self.images_dict)):
				# Implement SSD process
				ssds = self.ssd_nxn(n, imnum)
				self.ssds_curr[imnum] = ssds

			# Generate correction vectors y_i
			y_i = self.dy_from_ssd(n)
			print(f"Here is y_i for iteration {iter_idx}:\n", y_i)

			# Create jacobian
			J = self.form_jacobian(params_best_guess)

			# Get delta_params
			Dalpha = self.param_change(J, y_i)
			print("\nDelta Alpha:\n", Dalpha)
			# Update params 
			self.params_best_guess += Dalpha
			print("\nUpdated Params: scale, theta, tq, tp\n", self.params_best_guess)

			# Apply change 
			# Create transformation matrix
			tform_mat = self.tform_create(self.params_best_guess[2][0], self.params_best_guess[3][0], 0, 0, 0, self.params_best_guess[1][0])
			print("\nTransformation matrix\n", tform_mat)
			# Apply tform matrix to image patches
			self.update_guess(tform_mat, self.params_best_guess[0][0])



		return self.params_best_guess


	def ssd_surface_plots(self, ims, n):
		"""
		Surface plot visualizations for SSD confidence level
		Inputs: Figure, Images to display (1-10 meaining all ten images)
		Output: Figure(s) (depending on number of images)
		"""

		# X coords:
		x = np.linspace(-n,n, 2*n+1)
		y = np.linspace(-n,n, 2*n+1)
		Y, X = np.meshgrid(x,y)

		# Create plot for each image specified
		for i in range(ims):
			# Find best shift
			ssds = self.ssds_curr[i]
			idrow, idcol = np.unravel_index(np.argmin(ssds), ssds.shape)
			shiftx_min = idrow - n
			shifty_min = idcol - n
			print(f"BEST SHIFT for image {i}:", shiftx_min, shifty_min)
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
			ax.set_title(f'SSD Surface Plot: Image {i}')
			ax.legend()
			ax.view_init(elev=90, azim=-90)
			# Color bar for the surface
			# fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10, label='SSD Value')

			plt.show()


	def unit_vec_c_MICRO(self, imnum, pts):
		"""
		Create unit vectors in camera frame coordinates for desired pixels 
		Using pixel location of points.
		"""
		# Get pixel locations and RGB values
		# print("Micro patch pts: \n", pts)
		pts_rgb_micro = self.images_dict[imnum][pts[:,1], pts[:,0]]/255
		# print("Rgb: \n", pts_rgb_mirco)

		im_imnum = self.images_dict[imnum]
		shape_im_y, shape_im_x = im_imnum.shape[:2]

		# Compute shifted pixel coordinates
		Px = pts[:, 0] - shape_im_x / 2  # Shape (H, W)
		Py = -pts[:, 1] + shape_im_y / 2  # Shape (H, W)

		# Apply final coordinate transformations
		Px, Py = -Py, -Px  # Swap and negate as per coordinate system

		# Compute magnitude of vectors
		mag = np.sqrt(Px**2 + Py**2 + self.focal**2)  # Shape (H, W)
		# self.im_pts_2d[imnum]['mag'] = mag

		# Compute unit vectors
		pts_vec_c = np.stack((Px / mag, Py / mag, np.full_like(Px, self.focal) / mag), axis=-1)  # Shape (H, W, 3)
		# print("Unit vectors cam coords:\n", pts_vec_c)

		return pts_vec_c, pts_rgb_micro


	def implement_guess_micro(self, tform_guess, scale):
		"""
		Implementing the newest state estimate guess for mosaic
		Input: transformation guess, scaling factor
		Output: transformed points for best guess
		"""

		# Implement tform for each image
		for i in range(len(self.images_dict)):
			for b, pts in self.bins_2d.items():
				if b[0] != i:
					continue
				loc_im_pts = self.im_mosaic_micro[i][b]['pts'].copy()
				# apply scale
				loc_im_pts[:,:2] *= scale
				# apply tform
				__, loc_im_pts_guess, loc_im_vec_guess = self.unit_vec_tform(loc_im_pts, self.origin_w, tform_guess)
				# Update best guess
				# Initialize the dictionary for this image if not yet created
				if i not in self.im_pts_best_guess_micro:
					self.im_pts_best_guess_micro[i] = {}
				self.im_pts_best_guess_micro[i][b] = {'pts': loc_im_pts_guess}


		

