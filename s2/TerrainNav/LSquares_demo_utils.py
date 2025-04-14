# Utilities file for the LSquares demo 
# For efficiency purposes during the demonstration 


import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from scipy.spatial import cKDTree



# SSD necessary functions 

def get_inside_sat_pts(i, shiftx, shifty):
    """
    Getting points inside the satellite image
    Input: image number, shiftx, shifty
    Output: Points inside corners from satellite image 
    """

    # Get corners 
    corners = im_pts_2d[i]['corners']
    # Define corner indices 
    idxs = [0, -corners[2], -1, corners[2]-1]
    # print(idxs)
    
    # Grab corner points 
    points = np.array(im_pts_curr[i]['pts'])[idxs]
    # Shift corners of points
    points[:,0] += shiftx
    points[:,1] += shifty
    # print(points)
    points2d = points[:,:-1]
    # print(points2d)

    # Define polygon path 
    polygon_path = Path(points2d)
    # Points within polygon 
    mask = polygon_path.contains_points(ref_pts[:,:-1])
    inside_pts = ref_pts[mask]
    inside_cg = ref_cg[mask]

    return inside_pts, inside_cg

def ssd_nxn(n, i):
    """
    New SSD process to run faster
    Sum of squared differences. Shifts around pixels 
    Input: n shift amount, image number
    Output: sum of squared differences for each shift
    """
    downs = 1 # Factor to downsample by 
    ssds = np.zeros((2*n+1,2*n+1))
    loc_pts = im_pts_curr[i]['pts'].copy()
    # print(loc_pts)

    for shiftx in range(-n,n+1):
        for shifty in range(-n, n+1):
            # Get points inside corners for satellite image 
            inside_pts, inside_cg = get_inside_sat_pts(i,shiftx,shifty)
            # print(inside_pts)
            # print(inside_pts.shape)

            # Downsample pts (grab only x and y)
            downsampled_pts = inside_pts[::downs, :-1] # Take every 'downs'-th element
            downsampled_cg = inside_cg[::downs,0]
            # print("Colors of downsampled pts\n", downsampled_cg)

            # Shift points 
            shifted_loc_pts = loc_pts + np.array([shiftx,shifty,0])
            # print(shiftx,shifty)
            # print(shifted_loc_pts)

            # Build tree
            tree = cKDTree(shifted_loc_pts[:,:2])

            # Find nearest points and calculate intensities
            distances, indices = tree.query(downsampled_pts, k=1)
            nearest_intensities = im_pts_2d[i]['rgbc'][indices,0]
            # print(nearest_intensities)
            # self.ints2 = nearest_intensities
            # print("\nNearest Intensities\n", nearest_intensities)
            # print(distances, indices)

            # Calculate SSDS
            diffs = downsampled_cg - nearest_intensities 
            # print("\nDifferences\n", diffs)
            ssd_curr = np.sum(diffs**2)

            # Store SSD value for the current shift
            ssds[shiftx + n, shifty + n] = ssd_curr
            # print("SSD = ", ssd_curr)

    print(f"Number of points used for image {i}: ", diffs.shape)
    
    return ssds

def tform_create(x,y,z,roll,pitch,yaw):
    """
    Creates a transformation matrix 
    Inputs: translation in x,y,z, rotation in roll, pitch, yaw (DEGREES)
    Output: Transformation matrix (4x4)
    """
    # Rotation
    roll_r, pitch_r, yaw_r = np.deg2rad([roll, pitch, yaw])
    euler_angles = [roll_r, pitch_r, yaw_r]
    rotmat = R.from_euler('xyz', euler_angles).as_matrix()

    # Translation
    trans = np.array([x,y,z]).reshape([3,1])

    # Create 4x4
    bottom = np.array([0.0, 0.0, 0.0, 1.0]).reshape([1,4])
    tform = np.concatenate([np.concatenate([rotmat, trans], 1), bottom], 0)
    # print("\nTransformation matrix \n", tform)

    return tform 

def unit_vec_tform(pts_vec, origin, homog_t):
    """
    Takes a set of unit vectors and transforms them according to a homogeneous transform
    **** TRYING TO SPEED UP WITH NEW VERSION ****
    Input: Unit vectors, transform 
    Output: Origin of new unit vectors, end points of new unit vectors, new unit vectors
    """
    # Get new origin
    origin_o = np.append(origin,1).reshape(-1,1)
    origin_n = (homog_t @ origin_o)[:-1].flatten()

    # Unit vectors to homogeneous coords 
    pts_homog = np.hstack((pts_vec, np.ones((pts_vec.shape[0], 1)))).T

    # Apply transformation
    pts_trans = (homog_t @ pts_homog)[:-1].T

    # New vectors 
    pts_vec_n = pts_trans - origin_n

    return origin_n, pts_trans, pts_vec_n





def grab_image_pts(x,y,w,h,i):
    """
    Grabs specific image points we want for match process
    """
    x_coords = np.arange(x, x+w)
    y_coords = np.arange(y, y+h)
    Px, Py = np.meshgrid(x_coords, y_coords, indexing='xy')
    # print(Px, Py)

    # Stack points for array 
    pts_loc = np.stack((Px, Py, np.ones_like(Px)), axis=-1) # -1 forms a new axis 
    # print(pts_loc)
    pts_loc = pts_loc.reshape(-1,3)
    # print(pts_loc)

    # Extract RGB
    pts_rgb = sat_ref[y:y+h, x:x+w].astype(np.float64).reshape(-1,1)
    pts_rgb = np.hstack([pts_rgb, pts_rgb, pts_rgb])
    pts_rgb /= 255
    # print(pts_rgb)

    # Store in dict
    corners = np.array([x,y,w,h])
    im_pts_2d[i] = {'pts': pts_loc}
    im_pts_2d[i]['rgbc'] = pts_rgb
    im_pts_2d[i]['corners'] = corners

    return pts_loc, pts_rgb