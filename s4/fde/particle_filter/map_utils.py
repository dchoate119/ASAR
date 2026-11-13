# Utility file for manipulating the satellite image map into a format to be used for the particle filter 
# Usable for 1D localization to be extended to 2D eventually


import numpy as np
import cv2
from scipy.ndimage import map_coordinates


def rectify_strip(img, p0, p1, half_w):
    """
    Resample strip along centerline 
    x = along track, y = across track 
    Assumes straight line across center line from p0 to p1
    Input: grayscale image, centerline endpoints, strip half-width (px) 
    Output: strip (2*half_w)
    """
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    length = int(np.linalg.norm(p1 - p0)) # Runway length (pixel coords), samples along track 
    u = (p1-p0) / np.linalg.norm(p1 - p0) # Unit vector p0 to p1
    n = np.array([-u[1], u[0]]) # Normal vector to u

    s = np.arange(length)
    t = np.arange(-half_w, half_w + 1)
    S, T = np.meshgrid(s,t)
    xs = p0[0] + S * u[0] + T *n[0]
    ys = p0[1] + S * u[1] + T * n[1]
    strip = map_coordinates(img, [ys, xs], order=1)
    return strip, u, n
