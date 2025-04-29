import numpy as np
import cv2
import open3d as o3d
import plotly.graph_objects as go
import plotly.io as pio
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree
import imageio
# %matplotlib qt
import matplotlib.pyplot as plt
from matplotlib.path import Path

from groundNAV_agent import *
from colmapParsingUtils import *

# # SAVE YOUR WORK
# %load_ext autoreload
# %autoreload 2
# %autosave 180

# CLOSER IMAGES 
# Load in necessary parameters for gNAV agent 
# Define Class Parameters 

images_colm  = "/home/daniel-choate/ASAR/s2/TerrainNav/TTurf/test/images.txt"
cameras_colm = "/home/daniel-choate/ASAR/s2/TerrainNav/TTurf/test/cameras.txt"
pts3d_colm = "/home/daniel-choate/ASAR/s2/TerrainNav/TTurf/test/points3D_f.txt"

# Images selected for local corrections
# image_1 = "/home/daniel-choate/Datasets/COLMAP/TTurfSAT/TTurf_Im/IMG_9449.JPEG"
image_1 = "/home/daniel-choate/Datasets/COLMAP/TTurfSAT/TTurf_Im/IMG_9475.JPEG"
# image_2 = "/home/daniel-choate/Datasets/COLMAP/TTurfSAT/TTurf_Im/IMG_9459.JPEG"
image_2 = "/home/daniel-choate/Datasets/COLMAP/TTurfSAT/TTurf_Im/IMG_9464.JPEG"
image_3 = "/home/daniel-choate/Datasets/COLMAP/TTurfSAT/TTurf_Im/IMG_9467.JPEG"
image_4 = "/home/daniel-choate/Datasets/COLMAP/TTurfSAT/TTurf_Im/IMG_9473.JPEG"
image_5 = "/home/daniel-choate/Datasets/COLMAP/TTurfSAT/TTurf_Im/IMG_9476.JPEG"
# Load in satellite reference image
sat_ref = "TTurf/TurfSat.jpg"
# sat_ref = cv2.imread('TTurf/TurfSat.jpg')

# Organize for agent params
images = [image_1, image_2, image_3, image_4, image_5]



# Create class
gnav = gNAV_agent(images_colm, cameras_colm, pts3d_colm, images, sat_ref)

# Grab raw points and RGB data for scene and reference cloud
scene_pts, rgb_data = gnav.grab_pts(gnav.pts3d_c)
ref_pts, ref_rgb = gnav.ref_pts, gnav.ref_rgb



# Use ground plane pts to set reference frame 
# Need gravity and height
pts_gnd_idx = np.array([25440, 25450, 25441, 25449, 25442, 25445, 103922, 103921, 103919, 103920])
# tform_ref_frame = gnav.set_ref_frame(pts_gnd_idx) # THIS IS WHAT I AM CHANGING 
tform_ref_frame = gnav.set_ref_frame_mid(pts_gnd_idx) # NEW VERSION
tform_ref_frame_pts = gnav.inv_homog_transform(tform_ref_frame)
print("\nReference frame transformation\n", tform_ref_frame_pts)



# Transform all points to the new coordinate system 
# Not necessary since we aren't using the cloud, but a good visual check for coord frame
tform_ref_frame_inv = gnav.inv_homog_transform(tform_ref_frame)
origin_ref, scene_pts_ref, scene_vec_ref = gnav.unit_vec_tform(scene_pts, gnav.origin_w, tform_ref_frame_inv)
# print(origin_ref)
# Transform scene cloud to 2D (also as a visual check)
# Note: 2d projection will look off with z=-1; see TTurf_v2 for cropping method
scene_ref_2d = gnav.proj_2d_scene(scene_pts_ref)
# print(scene_ref_2d)





# Image 1 - OS in Jumbos OR top half of S

# # OS in Jumbos 
# imnum = 0
# # x,y = 150,1725
# # side_x = 2250 # WIDTH
# # side_y = 750 # HEIGHT
# x,y = 150, 2000
# side_x = 1500
# side_y = 250 # HEIGHT

# # Top half of S
# imnum = 0
# x,y = 300,2100
# side_x = 1300 # WIDTH
# side_y = 750 # HEIGHT
# x,y = 300,2200
# side_x = 1300 # WIDTH
# side_y = 600 # HEIGHT

# # Bottom of M 
# imnum = 0
# x,y = 1100,2000
# side_x = 1300 # WIDTH
# side_y = 600 # HEIGHT

# # 10 yd hash 
imnum = 0
x,y = 1500,1000
side_x = 900 # WIDTH
side_y = 500 # HEIGHT

# Plot to visualize
# gnav.plot_rect_im(x, y, side_x, side_y, imnum) 

# Get necessary location and rgb data 
pts_loc, pts_rgb = gnav.grab_image_pts(x, y, side_x, side_y, imnum)
# print(pts_loc)
# print(pts_rgb)





# Image 2 - 40yd (far), OR O in jumbos

# # 40yd (far)
# imnum = 1
# x,y = 1150,1150
# side_x = 1100 # WIDTH
# side_y = 900 # HEIGHT

# # O in jumbos 
# imnum = 1
# x,y = 300,1700
# side_x = 1000 # WIDTH
# side_y = 900 # HEIGHT

# Bottom of S
imnum = 1
x,y = 1400,2300
side_x = 800 # WIDTH
side_y = 1200 # HEIGHT

# Plot to visualize
# gnav.plot_rect_im(x, y, side_x, side_y, imnum) 

# Get necessary location and rgb data 
pts_loc, pts_rgb = gnav.grab_image_pts(x, y, side_x, side_y, imnum)
# print(pts_rgb)






# Image 3 - Jumbo logo OR B in jumbos

# # Jumbo logo
# imnum = 2
# x,y = 400,1500
# side_x = 1500 # WIDTH
# side_y = 750 # HEIGHT

# B in jumbos 
imnum = 2
x,y = 800,2100
side_x = 1400 # WIDTH
side_y = 500 # HEIGHT

# Plot to visualize
# gnav.plot_rect_im(x, y, side_x, side_y, imnum) 

# Get necessary location and rgb data 
pts_loc, pts_rgb = gnav.grab_image_pts(x, y, side_x, side_y, imnum)
# print(pts_rgb)





# Image 4 - 30yd (near) OR edge to endzone 

# 30yd (near)
# imnum = 3
# x,y = 1350,1450
# side_x = 1200 # WIDTH
# side_y = 500 # HEIGHT

# Edge to endzone
imnum = 3
x,y = 1150,900
side_x = 1200 # WIDTH
side_y = 500 # HEIGHT

# Plot to visualize
# gnav.plot_rect_im(x, y, side_x, side_y, imnum) 

# Get necessary location and rgb data 
pts_loc, pts_rgb = gnav.grab_image_pts(x, y, side_x, side_y, imnum)
# print(pts_rgb)




# Image 5 - 10yd marker
imnum = 4
x,y = 1200,1300
side_x = 1000 # WIDTH
side_y = 700 # HEIGHT

# Plot to visualize
# gnav.plot_rect_im(x, y, side_x, side_y, imnum) 

# Get necessary location and rgb data 
pts_loc, pts_rgb = gnav.grab_image_pts(x, y, side_x, side_y, imnum)
# print(pts_rgb)







# Generate projection of image sections 
for i in range(len(images)):
# Just for the first image for now
# for i in range(1):
    # Unit vectors in camera coords 
    pts_vec_c, pts_rgb_gnd = gnav.unit_vec_c(i)
    gnav.im_mosaic[i] = {'rgbc': pts_rgb_gnd}

    # Get transformation matrix that move from camera coords to world coords
    id = gnav.im_ids[i]
    homog_w2c, homog_c2w = gnav.get_pose_id(id,i)
    # print('Homogeneous transformation from world to camera \n', homog_c2w)
    # print('\n Homogeneous transformation from camera to world \n', homog_w2c)

    # Transform to world coords
    origin_c, pts_loc_w, pts_vec_w = gnav.unit_vec_tform(pts_vec_c, gnav.origin_w, homog_c2w)
    # print('\n New camera frame origin = ', origin_c)
    
    # Get new points 
    ranges, new_pts_w = gnav.pt_range(pts_vec_w, homog_c2w, origin_c, i)
    # print('\nNew Points \n', new_pts_w)

    # Transfer points to reference frame
    __, new_pts_r, pts_vec_r = gnav.unit_vec_tform(new_pts_w, gnav.origin_w, tform_ref_frame_pts)

    # Convert points to grayscale 
    gray_c = gnav.conv_to_gray(gnav.im_mosaic[i]['rgbc'],i)
    # print(gray_c)

    # Put new points and grayscale colors in image mosaic
    gnav.im_mosaic[i]['pts'] = new_pts_r
    gnav.im_mosaic[i]['color_g'] = gray_c
    
    print("\nDone image ", i)





# Implementing an initial guess for the local image 

# SCALE for initial guess 
# scale = gnav.focal/39
scale = 80

# Avg guess for OLD IMAGES  
x = -55
y = 20
yaw = np.deg2rad(140)


tform_guess = gnav.tform_create(x,y,0,0,0,yaw)
gnav.best_guess_tform = tform_guess
gnav.best_guess_scale = scale
# print(tform_guess)

for i in range(len(images)):
# Just for the first image for now
# for i in range(1):
    loc_im_pts = gnav.im_mosaic[i]['pts'].copy()
    # print(loc_im_pts)
    loc_im_pts[:, :2] *= scale
    # Get new points 
    __, loc_im_pts_guess, loc_im_vec_guess = gnav.unit_vec_tform(loc_im_pts, gnav.origin_w, tform_guess)
    gnav.im_pts_best_guess[i] = {'pts': loc_im_pts_guess}
    # gnav.im_pts_best_guess[i]['tree'] = cKDTree(loc_im_pts_guess) # UNECESSARY 

    print("\nDone image ", i)




# PLOTTING THE NEW SCENE MOSAIC

# Use open3d to create point cloud visualization 
# Create visualization 
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Mosaic scene with satellite reference")

# Create axes @ origin
axis_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=40)

# Create point cloud for image points
im0_cloud = o3d.geometry.PointCloud()
im0_cloud.points = o3d.utility.Vector3dVector(gnav.im_pts_best_guess[0]['pts'])
im0_cloud.colors = o3d.utility.Vector3dVector(gnav.im_mosaic[0]['color_g'])

# Create point cloud for image points
im1_cloud = o3d.geometry.PointCloud()
im1_cloud.points = o3d.utility.Vector3dVector(gnav.im_pts_best_guess[1]['pts'])
im1_cloud.colors = o3d.utility.Vector3dVector(gnav.im_mosaic[1]['color_g'])

# Create point cloud for image points
im2_cloud = o3d.geometry.PointCloud()
im2_cloud.points = o3d.utility.Vector3dVector(gnav.im_pts_best_guess[2]['pts'])
im2_cloud.colors = o3d.utility.Vector3dVector(gnav.im_mosaic[2]['color_g'])

# Create point cloud for image points
im3_cloud = o3d.geometry.PointCloud()
im3_cloud.points = o3d.utility.Vector3dVector(gnav.im_pts_best_guess[3]['pts'])
im3_cloud.colors = o3d.utility.Vector3dVector(gnav.im_mosaic[3]['color_g'])

# Create point cloud for image points
im4_cloud = o3d.geometry.PointCloud()
im4_cloud.points = o3d.utility.Vector3dVector(gnav.im_pts_best_guess[4]['pts'])
im4_cloud.colors = o3d.utility.Vector3dVector(gnav.im_mosaic[4]['color_g'])

# Create point cloud for reference cloud (satellite)
ref_cloud = o3d.geometry.PointCloud()
ref_cloud.points = o3d.utility.Vector3dVector(gnav.ref_pts)
ref_cloud.colors = o3d.utility.Vector3dVector(gnav.ref_rgb)

# Add necessary geometries to visualization 
vis.add_geometry(axis_origin)
vis.add_geometry(ref_cloud)
vis.add_geometry(im0_cloud)
vis.add_geometry(im1_cloud)
vis.add_geometry(im2_cloud)
vis.add_geometry(im3_cloud)
vis.add_geometry(im4_cloud)


# # Size options (jupyter gives issues when running this multiple times, but it looks better)
# render_option = vis.get_render_option()
# render_option.point_size = 2

# Set up initial viewpoint
view_control = vis.get_view_control()
# Direction which the camera is looking
view_control.set_front([0, .01, -1])  # Set the camera facing direction
# Point which the camera revolves about 
view_control.set_lookat([0, 0, 0])   # Set the focus point
# Defines which way is up in the camera perspective 
view_control.set_up([0, -1, 0])       # Set the up direction
view_control.set_zoom(.45)           # Adjust zoom if necessary

# Run and destroy visualization 
vis.run()
vis.destroy_window()



# PLOTTING THE NEW SCENE MOSAIC

# Use open3d to create point cloud visualization 
# Create visualization 
vis2 = o3d.visualization.Visualizer()
vis2.create_window(window_name="Mosaic scene with satellite reference")

# Create axes @ origin
axis_origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=40)

# Create point cloud for image points
im0_cloud = o3d.geometry.PointCloud()
im0_cloud.points = o3d.utility.Vector3dVector(gnav.im_pts_best_guess[0]['pts'])
im0_cloud.colors = o3d.utility.Vector3dVector(gnav.im_mosaic[0]['color_g'])

# Create point cloud for image points
im1_cloud = o3d.geometry.PointCloud()
im1_cloud.points = o3d.utility.Vector3dVector(gnav.im_pts_best_guess[1]['pts'])
im1_cloud.colors = o3d.utility.Vector3dVector(gnav.im_mosaic[1]['color_g'])

# Create point cloud for image points
im2_cloud = o3d.geometry.PointCloud()
im2_cloud.points = o3d.utility.Vector3dVector(gnav.im_pts_best_guess[2]['pts'])
im2_cloud.colors = o3d.utility.Vector3dVector(gnav.im_mosaic[2]['color_g'])

# Create point cloud for image points
im3_cloud = o3d.geometry.PointCloud()
im3_cloud.points = o3d.utility.Vector3dVector(gnav.im_pts_best_guess[3]['pts'])
im3_cloud.colors = o3d.utility.Vector3dVector(gnav.im_mosaic[3]['color_g'])

# Create point cloud for image points
im4_cloud = o3d.geometry.PointCloud()
im4_cloud.points = o3d.utility.Vector3dVector(gnav.im_pts_best_guess[4]['pts'])
im4_cloud.colors = o3d.utility.Vector3dVector(gnav.im_mosaic[4]['color_g'])

# Create point cloud for reference cloud (satellite)
ref_cloud = o3d.geometry.PointCloud()
ref_cloud.points = o3d.utility.Vector3dVector(gnav.ref_pts)
ref_cloud.colors = o3d.utility.Vector3dVector(gnav.ref_rgb)

# Add necessary geometries to visualization 
vis2.add_geometry(axis_origin)
vis2.add_geometry(ref_cloud)
vis2.add_geometry(im0_cloud)
vis2.add_geometry(im1_cloud)
vis2.add_geometry(im2_cloud)
vis2.add_geometry(im3_cloud)
vis2.add_geometry(im4_cloud)


# # Size options (jupyter gives issues when running this multiple times, but it looks better)
# render_option = vis.get_render_option()
# render_option.point_size = 2

# Set up initial viewpoint
view_control = vis2.get_view_control()
# Direction which the camera is looking
view_control.set_front([0, 0, -1])  # Set the camera facing direction
# Point which the camera revolves about 
view_control.set_lookat([0, 0, 0])   # Set the focus point
# Defines which way is up in the camera perspective 
view_control.set_up([0, -1, 0])       # Set the up direction
view_control.set_zoom(.45)           # Adjust zoom if necessary

# Run and destroy visualization 
vis2.run()
vis2.destroy_window()