import open3d as o3d
import imageio
import numpy as np
import os
import matplotlib.pyplot as plt

# Create your initial coordinate frame geometry
axis_orig = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=np.array([0.,0.,0]))

# Initialize Open3D visualizer
vis = o3d.visualization.Visualizer()
vis.create_window(window_name='3D points from specified image id', width=1000, height=1000)

vis.add_geometry(axis_orig)

# Capture frames for GIF
frames = []
num_frames = 100  # Number of frames to capture

# Set up custom camera parameters manually to avoid JSON dependency
view_control = vis.get_view_control()
view_control.set_front([0, 0, -1])  # Set the camera facing direction
view_control.set_lookat([0, 0, 0])  # Set the focus point
view_control.set_up([0, 1, 0])  # Set the up direction
view_control.set_zoom(1.0)  # Adjust zoom if necessary

# Manually capture frames without needing JSON files
def custom_draw_geometry_with_camera_trajectory(pcd):
    custom_draw_geometry_with_camera_trajectory.index = -1
    custom_draw_geometry_with_camera_trajectory.vis = o3d.visualization.Visualizer()

    if not os.path.exists("./TestData/image/"):
        os.makedirs("./TestData/image/")
    if not os.path.exists("./TestData/depth/"):
        os.makedirs("./TestData/depth/")

    def move_forward(vis):
        ctr = vis.get_view_control()
        glb = custom_draw_geometry_with_camera_trajectory
        if glb.index >= 0:
            print("Capture image {:05d}".format(glb.index))
            depth = vis.capture_depth_float_buffer(False)
            image = vis.capture_screen_float_buffer(False)
            # Save depth and image frames
            plt.imsave("./TestData/depth/{:05d}.png".format(glb.index), np.asarray(depth), dpi=1)
            plt.imsave("./TestData/image/{:05d}.png".format(glb.index), np.asarray(image), dpi=1)
            
            # Capture current frame to append for GIF creation
            image_8bit = (np.asarray(image) * 255).astype(np.uint8)
            frames.append(image_8bit)
        
        glb.index += 1
        if glb.index < num_frames:
            ctr.rotate(0.02, 0.0, 0.0)  # Rotate the camera by 0.02 radians
        else:
            custom_draw_geometry_with_camera_trajectory.vis.register_animation_callback(None)
        
        return False


    vis = custom_draw_geometry_with_camera_trajectory.vis
    vis.create_window()
    vis.add_geometry(pcd)
    vis.register_animation_callback(move_forward)
    vis.run()
    vis.destroy_window()

# Run the trajectory capture with your original geometry
custom_draw_geometry_with_camera_trajectory(axis_orig)

# After capturing all frames, save the GIF
if frames:
    imageio.mimsave('test.gif', frames, duration=0.05)  # Adjust frame duration as needed
else:
    print("No frames captured! Please check if the camera is properly moving.")
