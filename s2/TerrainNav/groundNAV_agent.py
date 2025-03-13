import numpy as np
import cv2
import open3d as o3d 
import matplotlib.pyplot as plt
import plotly.graph_objects as go
import plotly.io as pio
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree

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
		self.sat_ref = cv2.cvtColor(self.sat_ref, cv2.COLOR_RGB2GRAY)
		self.read_colmap_data()
		self.image_parsing()
		self.sat_im_init()


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
		cols = self.sat_ref.shape[0]
		rows = self.sat_ref.shape[1]
		# print(cols,rows)
		n = cols*rows 
		ref_pts = np.zeros((n,3))
		ref_rgb = np.zeros((n,3))
		count = 0
		for i in range(cols):
			for j in range(rows):
				ref_pts[count] = [j,i,1]
				ref_rgb[count] = self.sat_ref[i][j]
				count += 1

		ref_rgb /= 255
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
		Defines a reference coordinate frame for the ****** 

		"""
		self.origin_w = np.array([0,0,0])
		self.pts_gnd = self.scene_pts[pts_gnd_idx]

		# Find gravity and height
		self.grav_vec = self.grav_SVD(self.pts_gnd)
		print('Gravity vector \n', self.grav_vec)
		self.h_0 = self.height_avg(self.pts_gnd, self.origin_w)
		print('\nHeight h_0 = ', self.h_0)

		# Get focal length 
		cam_id = list(self.cameras_c.keys())[0]
		self.focal = self.cameras_c[cam_id].params[0]
		print("Focal length \n", self.focal)


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
		yaw = np.deg2rad(220)
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
		Input: Unit vectors, transform 
		Output: Origin of new unit vectors, end points of new unit vectors, new unit vectors
		"""
		# Get new origin
		origin_o = origin
		origin_o = np.append(origin_o,1)
		origin_o = origin_o.reshape(-1,1)
		origin_n = homog_t @ origin_o
		origin_n = origin_n[:-1]
		origin_n = origin_n.flatten()

		# Array for new locations and vectors 
		pts_loc_n = np.zeros((len(pts_vec),3))
		pts_vec_n = np.zeros((len(pts_vec),3))

		for i in range(len(pts_vec)):
			v = pts_vec[i]
			# Make homogeneous
			v = np.append(v,1)
			v = v.reshape(-1,1)
			# Transform 
			v_trans = homog_t @ v

			# Grab new pt and vector
			pt = v_trans[:-1]
			pt = pt.flatten()
			vec = pt - origin_n

			# Add to arrays
			pts_loc_n[i] = pt
			pts_vec_n[i] = vec


		return origin_n, pts_loc_n, pts_vec_n

	def proj_2d_scene(self, pts):
		"""
		Take a 3d scene and returns a projection of those 3D points in 2D
		NOTE: z-value will be equal to 1
		Input: 3D points 
		Output: 2D points (where z=1)
		"""

		pts_2D = pts / pts[:, 2][:, np.newaxis]

		return pts_2D


		




   