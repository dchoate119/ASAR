import open3d as o3d
import imageio
import numpy as np


vis = o3d.visualization.Visualizer()
vis.create_window(window_name='3D points from specified image id', width=1000, height=1000)

# Create initial axes 
axis_orig = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=np.array([0.,0.,0]))
# axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25, origin = np.array([0.,0.,0.])).transform(homog_t)


vis.add_geometry(axis_orig)

vis.add_geometry(axis_orig)
vis.poll_events()
vis.update_renderer()

# Set up initial viewpoint
view_control = vis.get_view_control()
# Direction which the camera is looking
view_control.set_front([0, 0, -1])  # Set the camera facing direction
# Point which the camera revolves about 
view_control.set_lookat([0, 0, 0])   # Set the focus point
# Defines which way is up in the camera perspective 
view_control.set_up([-1, 0, 0])       # Set the up direction
view_control.set_zoom(1.25)           # Adjust zoom if necessary


# Capture frames for GIF
frames = []
num_frames = 30  # Adjust the number of frames
angle_step = 180/num_frames

for i in range(num_frames):
	# Rotate the view
    view_control.rotate(angle_step, angle_step)  # (horizontal, vertical)

    # vis.update_geometry(axis_orig) # Only if I move it myself?
    vis.poll_events()
    vis.update_renderer()

    # Capture frame directly into memory
    image = vis.capture_screen_float_buffer(False)
    image_8bit = (np.asarray(image) * 255).astype(np.uint8)  # Convert to 8-bit
    frames.append(image_8bit)


for i in range(num_frames):
	# Rotate the view
	view_control.rotate(-angle_step, -angle_step)  # (horizontal, vertical)

	# vis.update_geometry(axis_orig) # Only if I move it myself?
	vis.poll_events()
	vis.update_renderer()

	# Capture frame directly into memory
	image = vis.capture_screen_float_buffer(False)
	image_8bit = (np.asarray(image) * 255).astype(np.uint8)  # Convert to 8-bit
	frames.append(image_8bit)

for i in range(num_frames):
	# Rotate the view
	view_control.rotate(-angle_step, angle_step)  # (horizontal, vertical)

	# vis.update_geometry(axis_orig) # Only if I move it myself?
	vis.poll_events()
	vis.update_renderer()

	# Capture frame directly into memory
	image = vis.capture_screen_float_buffer(False)
	image_8bit = (np.asarray(image) * 255).astype(np.uint8)  # Convert to 8-bit
	frames.append(image_8bit)

for i in range(num_frames):
	# Rotate the view
	view_control.rotate(angle_step, -angle_step)  # (horizontal, vertical)

	# vis.update_geometry(axis_orig) # Only if I move it myself?
	vis.poll_events()
	vis.update_renderer()

	# Capture frame directly into memory
	image = vis.capture_screen_float_buffer(False)
	image_8bit = (np.asarray(image) * 255).astype(np.uint8)  # Convert to 8-bit
	frames.append(image_8bit)





# Create GIF
# Ensure frames are in the correct format
frames = [frame.astype("uint8") for frame in frames]

# Use imageio to save as GIF
imageio.mimsave("test.gif", frames, fps=30, loop=0)  # Adjust fps if necessary


# # Use FFmpeg writer
# writer = imageio.get_writer("test.mp4", fps=60, format="FFMPEG")

# for frame in frames:
#     writer.append_data(frame)

# writer.close()


vis.run()
vis.destroy_window()