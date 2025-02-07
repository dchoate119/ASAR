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

	def read_colmap_data(self, images_c, cameras_c, pts3d_c, image):
		"""
		Transform the colmap data files into desired form
		"""
		images = read_images_text(self.images_c)
		cameras = read_cameras_text(self.cameras_c)
		pts3d = read_points3d_text(self.pts3d_c)
		im_gnd = cv2.imread(image)

		return images, cameras, pts3d, im_gnd


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

		return scene_pts, rgb_data 

	def grab_image_pts(x, y, width, height):
		"""
		Grab points of an image (that we know are on ground plane)
		"""

