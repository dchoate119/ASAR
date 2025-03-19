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
	"""
	def __init__(self, images_c, cameras_c, pts3d_c, image):
		self.images_c = images_c		# Location of images file
		self.cameras_c = cameras_c		# Location of cameras file
		self.pts3d_c = pts3d_c			# Location of ptd3d file
		self.image = image 				# Location of specific image file
		# Read colmap data 
		self.images = read_images_text(self.images_c)
		self.cameras = read_cameras_text(self.cameras_c)
		self.pts3d = read_points3D_text(self.pts3d_c)
		self.im_gnd = cv2.imread(image)


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

	def grab_image_pts(self, x, y, width, height):
		"""
		Grab points of an image (that we know are on ground plane)
		Based on specified starting x and y location, width, height
		Potentially automate process in future 
		"""
		# Define new variables for x, y, width, height 
		self.x = x
		self.y = y
		self.width = width
		self.height = height

		pts_loc = np.zeros((height, width, 2), dtype=int)
		pts_rgb = np.zeros((height, width, 3), dtype=int)
		# self.im_gnd_plt = cv2.cvtColor(self.im_gnd, cv2.COLOR_BGR2RGB)

		# Loop through each pixel 
		for i in range(x, x+width):
			for j in range(y, y+height):
				# Pixel position in x and y
				Px = i
				Py = j
				# Location in the array 
				array_loc_col = Px-x
				array_loc_row = Py-y
				# RGB value
				rgb = self.im_gnd[j,i]
				rgb = rgb.astype(int)

				# Place Px and Py in proper location 
				pts_loc[array_loc_row][array_loc_col] = [Px, Py]
				# Place rgb value in same location 
				pts_rgb[array_loc_row][array_loc_col] = rgb

		# self.pts_loc = pts_loc
		# self.pts_rgb = pts_rgb 

		return pts_loc, pts_rgb


	def plot_rect_im(self, x, y, width, height):
		"""
		Plotting a rectangle over the desired area of choice for ground plane point 
		Specified by user with x, y, width, height 
		"""
		# Make figure 
		fig, ax = plt.subplots(figsize=(15,8))
		# Draw rectangle 
		rect = plt.Rectangle((x,y), width, height, linewidth=1, edgecolor='r', facecolor='none')
		# Change color scheme to match matplotlib 
		self.im_gnd_plt = cv2.cvtColor(self.im_gnd, cv2.COLOR_BGR2RGB)

		# Plot 
		ax.imshow(self.im_gnd_plt)
		ax.add_patch(rect)
		ax.axis("off")

		# Show plot 
		plt.show()



	def unit_vec_c(self, pts_loc, pts_rgb):
		"""
		Create unit vectors in camera frame coords for desired pixels 
		Using pixel location of points
		"""
		# Define focal length 
		focal = self.cameras[2].params[0]
		self.focal = focal
		# print(focal)
		shape_im_y = self.im_gnd.shape[0]
		shape_im_x = self.im_gnd.shape[1]

		# Calculate number of pixels for vector array
		n = self.width * self.height
		pts_vec_c = np.zeros((n,3))
		pts_rgb_gnd = np.zeros((n,3))


		count = 0
		for i in range(self.height):
			for j in range(self.width):
				Px = pts_loc[i][j][0]
				Py = pts_loc[i][j][1]

				# print(Px, Py) # DEBUGGING

				# SHIFTING FOR IMAGE COORD FRAME 
				# (wierd frame based on camera locations)
				# x positive is DOWN, y positive is to LEFT, z into pg
				Px = Px - shape_im_x/2
				Py = -Py + shape_im_y/2
				# Intermediate values for last shift
				x_i = Px
				y_i = Py
				# Final pixel values 
				Py = -x_i
				Px = -y_i

				# Magnituge of vector
				mag = (Px**2 + Py**2 + focal**2)**0.5

				# Place vector
				pts_vec_c[count] = [Px/mag, Py/mag, focal/mag]

				# RGB value
				rgb_val = pts_rgb[i][j]
				pts_rgb_gnd[count] = rgb_val

				# Update
				count += 1


		pts_rgb_gnd = pts_rgb_gnd/255 # SCALED 

		# self.pts_vec_c = pts_vec_c
		# self.pts_rgb_gnd = pts_rgb_gnd

		return pts_vec_c, pts_rgb_gnd



	# def (plot_unit_vec):
	# 	"""
	# 	Write a function that plots unit vectors
	# 	(Later)
	# 	"""


	def get_pose_id(self, id):
		"""
		Get the pose transformation for a specific image id
		Input: Image ID
		Output: transform from camera to world coordinates
		"""
		# Get quaternion and translation vector
		qvec = self.images[id].qvec
		tvec = self.images[id].tvec[:,None]
		# print(tvec)

		t = tvec.reshape([3,1])

		# Create rotation matrix
		Rotmat = qvec2rotmat(qvec) # Positive or negative does not matter
		# print("\n Rotation matrix \n", Rotmat)

		# Create 4x4 transformation matrix with rotation and translation
		bottom = np.array([0.0, 0.0, 0.0, 1.0]).reshape([1, 4])
		m = np.concatenate([np.concatenate([Rotmat, t], 1), bottom], 0)
		c2w = np.linalg.inv(m)

		return c2w

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



	def grav_SVD(self, pts_gnd_idx):
		"""
		Getting the gravity vector for a set of points on the ground plane
		Input: Indices for the ground plane pts
		Output: Gravity vector 
		Note: potentially automate ground point process in the future 
		"""

		# Grab points based on indices
		pts_gnd = self.scene_pts[pts_gnd_idx]

		# Subtract centroid for SVD
		centroid = np.mean(pts_gnd, axis=0)
		centered_points = pts_gnd - centroid

		# Singular value decomposition (SVD)
		U, S, Vt = np.linalg.svd(centered_points)

		grav_vec = Vt[-1,:]

		self.grav_vec = grav_vec

		return grav_vec


	def height_avg(self, pts_gnd_idx, origin):
		"""
		Get the initial height of the origin above the ground plane 
		Input: Indices for the ground plane pts
		Output: Average h_0
		"""

		# Grab points based on indices
		pts_gnd = self.scene_pts[pts_gnd_idx]

		# Multiple h0's
		h0s = np.zeros((len(pts_gnd)))
		for i in range(len(pts_gnd)):
			h0i = np.dot(self.grav_vec, pts_gnd[i]-origin)
			h0s[i] = h0i

		# Average 
		h_0 = np.mean(h0s)

		self.h_0 = h_0

		return h_0



	def pt_range(self, pts_vec, homog_t, origin):
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

		ranges = np.zeros((len(pts_vec),1))
		new_pts = np.zeros((len(pts_vec),3))

		for i in range(len(pts_vec)):
			# Range
			u_w = pts_vec[i]
			denom = np.dot(u_w, self.grav_vec)
			r = num/denom

			# New point 
			new_vec = u_w*r
			new_pt = origin + new_vec

			ranges[i] = r
			new_pts[i] = new_pt

		return ranges, new_pts


