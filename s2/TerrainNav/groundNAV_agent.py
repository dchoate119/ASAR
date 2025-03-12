import numpy as np
import cv2
import open3d as o3d 
import matplotlib.pyplot as plt
import plotly.graph_objects as go
import plotly.io as pio

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
		self.read_colmap_data()
		# Read colmap data 
		# self.im_gnd_{i} = cv2.imread(image) for image in self.images


	def read_colmap_data(self):
		self.images_c = read_images_text(self.images_c_loc)
		self.cameras_c = read_cameras_text(self.cameras_c_loc)
		self.pts3d_c = read_points3D_text(self.pts3d_c_loc)


	# def find_image_id_by_name(self, images, target_name):
	# 	for image_id, image in images.items():
	# 		if image.name.startswith(target_name):  # Use startswith to ignore file extensions if needed
	# 			return image_id
	# 	return None


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

		# X Direction as ZxV
		x_dir = np.cross(z_bar, v)
		x_bar = x_dir/np.linalg.norm(x_dir)
		print("\nX unit vector \n", x_bar)
		# Y Direction as ZxX
		y_dir = np.cross(z_bar, x_bar)
		y_bar = y_dir/np.linalg.norm(y_dir)
		print("\nY unit vector \n", y_bar)

		# Rotation matrix 
		rotmat = np.column_stack((x_bar, y_bar, z_bar))
		print("\nRotation Matrix\n", rotmat)
		# Translation Vector
		trans = P1.reshape([3,1])

		# Form transformation matrix 
		bottom = np.array([0.0, 0.0, 0.0, 1.0]).reshape([1,4])
		tform = np.concatenate([np.concatenate([rotmat, trans],1),bottom],0)
		print("\nTransformation matrix to ground \n", tform)

		# Translation from ground to desired height 

		return 1

		




   