import numpy as np 
import open3d as o3d

def voxel_distribution(width, min_x, min_y, min_z, max_x, max_y, max_z, ptCloud):
    """ Takes a point cloud and returns voxel placement of each point
    Inputs: voxel width, min pt in x, y, z, max pt in x,y,z, and
    the point cloud 
    Output: Indices with the voxel number of each point
    """
    vox_x = np.ceil((max_x - min_x)/width)
    vox_y = np.ceil((max_y - min_y)/width)
    vox_z = np.ceil((max_z - min_z)/width)
    vox_dist = np.zeros((len(ptCloud),1))
    num_vox = vox_x*vox_y*vox_z
    for i in range(len(ptCloud)):
        vox_indx = np.ceil((ptCloud[i,0] - min_x)/width)
        vox_indy = np.ceil((ptCloud[i,1] - min_y)/width)
        vox_indz = np.ceil((ptCloud[i,2] - min_z)/width)
        vox_ind = vox_indx + (vox_x*(vox_indy-1)) + (vox_y*vox_x*(vox_indz-1))
        vox_dist[i,0] = vox_ind
    return vox_dist


# Coloring the points of a specific voxel for visualization 

def plot_spec_vox(vis, ptCloud, vox_dist, vox_ID, color): #, rbg_data = None):
    """ Coloring the points on an open3d plot for a specific voxel 
    Inputs: visualization graph (open3d), pt cloud, voxel distribution,
    voxel ID
    Outputs: visualization with specified points colored 
    """
    ind_spec = np.argwhere(vox_dist == vox_ID) # specified indices
    pts_spec = ptCloud[ind_spec[:,0],:] # points from specified indices

    # Create points to be added to visualization 
    vox_pts = o3d.geometry.PointCloud()
    vox_pts.points = o3d.utility.Vector3dVector(pts_spec)
    vox_pts.paint_uniform_color(color)

    # Add axes, scene, and voxel points to vis
    # vis.add_geometry(axes)
    # vis.add_geometry(scene_pts)
    vis.add_geometry(vox_pts)
    
    return vis

# Fitting a gaussian distribution for a specific set of point 

def fit_gaussian(pts):
    """ Find the gaussian distribution from a set of points 
    Inputs: pt cloud, or set of points 
    Outputs: muh (mean), and sigma (st.dev) of pts as a covariance matrix
    """
    muh_pts = np.mean(pts, axis=0)
    cov_pts = np.cov(pts, rowvar=False)
    return muh_pts, cov_pts


# Create a function for plotting an ellipse and bounding box on a voxel
def plot_ellipse_box(vis, muh, cov, color):
    """ Plotting an ellipse and bounding box for a group of pts
    in a voxel 
    Inputs: visualization, mean of the points, covariance of the points, color of ellipse
    Outputs: Visualization with ellipse and bounding box
    """
    # Get eigenvalues and eigenvectors from covariance matrix 
    eig_vals, eig_vect = np.linalg.eig(cov)
    d_eig = (np.sqrt(eig_vals))*np.eye(3)
    # Multiply by two for two standard deviations 
    d_eig = d_eig*2

    # Form the homogeneous transformation matrix 
    A_trans = np.matmul(eig_vect, d_eig)
    # Needs to be 4x4 for homogeneous, adding translation 
    homo_trans = np.append(np.append(A_trans, np.zeros([3,1]), axis = 1), np.array([[0,0,0,1]]), axis = 0)
    homo_trans[:3,-1] = muh

    # Create a sphere mesh and transform into matrix using open3d
    muh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius = 1)
    muh_sphere.transform(homo_trans)
    muh_sphere.paint_uniform_color(color)

    # Create a bounding box for the sphere 
    muh_box = muh_sphere.get_oriented_bounding_box()

    # Attain points and lines to plot the bounding box
    points = np.array(muh_box.get_box_points())
    
    # Correlate points to form edges of the box
    # NOTE: DO NOT MESS with this form for vertices
    edges = [
    [0, 1], [0, 3], [6, 1], [3, 6],  # Bottom face
    [2, 7], [4, 7], [4, 5], [2, 5],  # Top face
    [0, 2], [1, 7], [4, 6], [3, 5]   # Vertical edges
    ]

    # Create a LineSet object
    lines = o3d.geometry.LineSet()

    # Set points and edges
    lines.points = o3d.utility.Vector3dVector(points)
    lines.lines = o3d.utility.Vector2iVector(edges)

    # Optionally, set the color of the lines to black
    # lines.colors = o3d.utility.Vector3dVector([[0, 0, 0]] * len(edges))
    lines.paint_uniform_color([0,0,0])
    
    # Add the resulting geometries
    vis.add_geometry(muh_sphere)
    vis.add_geometry(muh_box)
    vis.add_geometry(lines)

    return vis

    

# Drawing a simple transform with a homogeneous transformation matrix 
def draw_transform_axes_o3d(vis,homo_trans):
    """ Creating an axes xyz to represent a transform of pts
    Inputs: the visualization, and homogeneous transformation
    Outputs: the visualization with the new axes post-transform
    """
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
    axes1 = copy.deepcopy(axes).transform(homo_trans)
    vis.add_geometry(axes1)

    return vis