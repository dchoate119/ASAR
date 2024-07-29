import numpy as np
import open3d as o3d

# Step 1: Generate random 3D points
def generate_random_point_cloud(num_points=1000):
    # Generate random coordinates
    points = np.random.rand(num_points, 3)  # Points in the range [0, 1)
    return points

# Step 2: Create Open3D PointCloud object
def create_point_cloud(points):
    # Create an Open3D PointCloud object
    pcd = o3d.geometry.PointCloud()
    # Assign the points to the PointCloud object
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

# Step 3: Visualize the PointCloud
def visualize_point_cloud(pcd):
    # Visualize the PointCloud
    o3d.visualization.draw_geometries([pcd], window_name='Random Point Cloud')

if __name__ == "__main__":
    # Generate random points
    num_points = 1000  # Adjust the number of points as needed
    points = generate_random_point_cloud(num_points)
    
    # Create and visualize the PointCloud
    pcd = create_point_cloud(points)
    visualize_point_cloud(pcd)
