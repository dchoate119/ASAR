"""
Helpers for large scale ambiguity (LSA) detection on satellite maps
"""
import numpy as np
import cv2
import torch
from scipy.ndimage import map_coordinates


def rectify_strip(img, p0, p1, half_w):
	"""
	Resample a straight strip along a centerline so x = along-track, y = across-track
	NOTE: assumes a straight centerline between hand-picked endpoints p0 -> p1 (x, y pixels)
	Inputs: grayscale image, centerline endpoints, strip half-width (px)
	Output: strip (2*half_w+1, length), unit along-track vector u, unit across-track vector n
	"""
	p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
	length = int(np.linalg.norm(p1 - p0))
	u = (p1 - p0) / np.linalg.norm(p1 - p0)
	n = np.array([-u[1], u[0]])

	s = np.arange(length)
	t = np.arange(-half_w, half_w + 1)
	S, T = np.meshgrid(s, t)
	xs = p0[0] + S * u[0] + T * n[0]
	ys = p0[1] + S * u[1] + T * n[1]
	strip = map_coordinates(img, [ys, xs], order=1)
	return strip, u, n


def tile_profiles(p, L, stride):
	"""
	Slide a window of length L along 1D profile p
	Output: tiles (N, L), tile start positions (N,)
	"""
	starts = np.arange(0, len(p) - L + 1, stride)
	tiles = np.stack([p[s:s + L] for s in starts])
	return tiles, starts


def tile_hog(strip, L, stride, cell=8):
	"""
	Slide a window of length L along a 2D strip and compute a HOG vector per tile
	9 orientation bins per cell, 2x2-cell blocks, block stride = 1 cell
	NOTE: strip height is trimmed to a multiple of cell (e.g. 81 -> 80 rows) for cv2 HOG
	Output: features (N, D), tile start positions (N,)
	"""
	h = (strip.shape[0] // cell) * cell
	im = (np.clip(strip[:h], 0, 1) * 255).astype(np.uint8)
	hog = cv2.HOGDescriptor((L, h), (2*cell, 2*cell), (cell, cell), (cell, cell), 9)

	starts = np.arange(0, im.shape[1] - L + 1, stride)
	feats = np.stack([hog.compute(im[:, s:s + L]).ravel() for s in starts])
	return feats, starts


def tile_dino(strip_rgb, L, stride, model, scale=2, device='cuda'):
	"""
	Slide a window of length L along an RGB strip and compute a DINOv2 patch-grid vector per tile
	Each patch token is L2-normalized, then flattened in grid order (keeps spatial layout)
	NOTE: tile resized by ~scale to the nearest multiple of 14 px (ViT patch size) -> slight stretch
	Inputs: strip_rgb (H, W, 3) float in [0, 1], DINOv2 model, upsample scale
	Output: features (N, D), tile start positions (N,)
	"""
	h = int(round(strip_rgb.shape[0] * scale / 14)) * 14
	w = int(round(L * scale / 14)) * 14
	mean = np.array([0.485, 0.456, 0.406], np.float32) # ImageNet normalization
	std = np.array([0.229, 0.224, 0.225], np.float32)

	starts = np.arange(0, strip_rgb.shape[1] - L + 1, stride)
	ims = np.stack([cv2.resize(strip_rgb[:, s:s + L], (w, h)) for s in starts])
	x = torch.from_numpy((ims - mean) / std).permute(0, 3, 1, 2).float().to(device)

	with torch.no_grad():
		tok = model.forward_features(x)['x_norm_patchtokens'] # (N, n_patches, C)
	tok = torch.nn.functional.normalize(tok, dim=-1)
	return tok.flatten(1).cpu().numpy(), starts


def ncc_matrix(tiles):
	"""
	Normalized cross-correlation between every pair of tiles (zero shift)
	Output: (N, N) similarity matrix in [-1, 1]
	"""
	z = tiles - tiles.mean(axis=1, keepdims=True)
	z /= np.linalg.norm(z, axis=1, keepdims=True) + 1e-12
	return z @ z.T
