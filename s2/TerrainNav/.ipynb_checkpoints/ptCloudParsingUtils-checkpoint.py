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


# Making a function that creates a plot of the point cloud with the specified voxel

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