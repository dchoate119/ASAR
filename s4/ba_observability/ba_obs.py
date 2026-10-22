# BA observability study: how a wrong shared focal length is absorbed by BA
# Steps: Jacobian -> Hessian -> Schur out points + gauge fix -> weak directions -> focal bias -> validate


import numpy as np
from scipy.spatial.transform import Rotation
from scipy.sparse import coo_matrix


def load_colmap_txt(path):
	"""
	Read cameras.txt, images.txt, points3D.txt
	Returns cams {id: params}, ims {id: dict(R, t, cam, name, xy, pid)}, pts {id: xyz} (file order)
	cams params as in COLMAP: SIMPLE_RADIAL = (f, cx, cy, k1); other models kept raw (poses/points only)
	Observations with no 3D point (pid = -1) are dropped
	"""
	cams = {}
	for line in open(f"{path}/cameras.txt"):
		if line.startswith('#'):
			continue
		s = line.split()
		cams[int(s[0])] = tuple(float(v) for v in s[4:])

	ims = {}
	lines = [l for l in open(f"{path}/images.txt") if not l.startswith('#')]
	for hdr, obs in zip(lines[0::2], lines[1::2]):
		s = hdr.split()
		qw, qx, qy, qz = (float(v) for v in s[1:5])
		o = np.array(obs.split(), dtype=float).reshape(-1,3)
		o = o[o[:,2] != -1]
		ims[int(s[0])] = dict(R=Rotation.from_quat([qx, qy, qz, qw]).as_matrix(),
							  t=np.array(s[5:8], dtype=float),
							  cam=int(s[8]), name=s[9],
							  xy=o[:,:2], pid=o[:,2].astype(int))

	pts = {}
	for line in open(f"{path}/points3D.txt"):
		if line.startswith('#'):
			continue
		s = line.split()
		pts[int(s[0])] = np.array(s[1:4], dtype=float)

	return cams, ims, pts


def project(f, k1, cx, cy, R, t, X):
	"""
	X (N,3) world -> (N,2) pixels: P = RX + t, x' = x/z, u = f(1 + k1 r^2)x' + cx
	"""
	P = X @ R.T + t
	xn = P[:,:2] / P[:,2:3]
	d = 1 + k1 * np.sum(xn**2, axis=1, keepdims=True)
	return f * d * xn + np.array([cx, cy])


def skew(a):
	"""
	(N,3) -> (N,3,3) cross-product matrices, [a]x b = a x b
	"""
	S = np.zeros((len(a),3,3))
	S[:,0,1], S[:,0,2], S[:,1,2] = -a[:,2], a[:,1], -a[:,0]
	return S - S.transpose(0,2,1)


def jacobian_obs(f, k1, cx, cy, R, t, X):
	"""
	Analytic Jacobian of (u,v) per observation, (N,2,11)
	Columns: [f, k1 | w (3) | t (3) | X (3)]
	Rotation perturbed on the left: R -> (I + [dw]x) R
	cx, cy held fixed (COLMAP does not refine them by default)
	"""
	RX = X @ R.T
	P = RX + t
	z = P[:,2]
	xp, yp = P[:,0]/z, P[:,1]/z
	r2 = xp**2 + yp**2
	d = 1 + k1*r2
	N = len(X)

	# Intrinsics block (direct)
	J_K = np.stack([np.stack([d*xp, f*r2*xp], -1),
					np.stack([d*yp, f*r2*yp], -1)], 1)

	# Factor 1: d(u,v)/d(x',y')
	F1 = f * np.stack([np.stack([d + 2*k1*xp**2, 2*k1*xp*yp], -1),
					   np.stack([2*k1*xp*yp, d + 2*k1*yp**2], -1)], 1)

	# Factor 2: d(x',y')/dP
	F2 = np.zeros((N,2,3))
	F2[:,0,0] = F2[:,1,1] = 1/z
	F2[:,0,2], F2[:,1,2] = -xp/z, -yp/z

	A = F1 @ F2					# d(u,v)/dP

	# Factor 3 per block: dP/dw = -[RX]x, dP/dt = I, dP/dX = R
	J_w = A @ -skew(RX)
	J_t = A
	J_X = A @ R

	return np.concatenate([J_K, J_w, J_t, J_X], axis=2)


def jacobian_fd(f, k1, cx, cy, R, t, X, eps=1e-6):
	"""
	Central finite-difference version of jacobian_obs, for checking it
	"""
	def proj(th):
		Rp = Rotation.from_rotvec(th[2:5]).as_matrix() @ R
		return project(th[0], th[1], cx, cy, Rp, th[5:8], th[8:11][None])[0]

	J = np.zeros((len(X),2,11))
	for n, Xn in enumerate(X):
		th0 = np.concatenate([[f, k1], np.zeros(3), t, Xn])
		for k in range(11):
			dth = np.zeros(11)
			dth[k] = eps * max(1, abs(th0[k]))
			J[n,:,k] = (proj(th0 + dth) - proj(th0 - dth)) / (2*dth[k])
	return J


def build_jacobian(cams, ims, pts):
	"""
	Full sparse BA Jacobian, all observations of all images
	Column layout: [f, k1 | pose 0 (w, t) | pose 1 | ... | point 0 (X) | point 1 | ...]
	Assumes one shared camera (single f, k1 column pair)
	Returns J (2*N_obs, 2 + 6*N_im + 3*N_pt) csr, im_ids, pt_ids (column order)
	"""
	assert len(cams) == 1, "shared camera only"
	assert len(next(iter(cams.values()))) == 4, "SIMPLE_RADIAL only"
	f, cx, cy, k1 = next(iter(cams.values()))
	im_ids, pt_ids = sorted(ims), sorted(pts)
	pt_col = {p: 2 + 6*len(im_ids) + 3*n for n, p in enumerate(pt_ids)}

	rows, cols, vals = [], [], []
	r0 = 0
	for i, iid in enumerate(im_ids):
		im = ims[iid]
		X = np.array([pts[p] for p in im['pid']])
		Jb = jacobian_obs(f, k1, cx, cy, im['R'], im['t'], X)		# (N,2,11)
		N = len(X)

		# Column index of each of the 11 entries, per observation
		c = np.empty((N,11), dtype=int)
		c[:,:2] = [0, 1]
		c[:,2:8] = 2 + 6*i + np.arange(6)
		c[:,8:] = np.array([pt_col[p] for p in im['pid']])[:,None] + np.arange(3)

		rows.append(np.repeat(r0 + np.arange(2*N), 11))
		cols.append(np.repeat(c, 2, axis=0).ravel())
		vals.append(Jb.reshape(-1))
		r0 += 2*N

	J = coo_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
				   shape=(r0, 2 + 6*len(im_ids) + 3*len(pt_ids))).tocsr()
	return J, im_ids, pt_ids


def schur_points(J, n_keep, sigma):
	"""
	Reduced Hessian over the first n_keep columns of J, all later columns (3 per point) eliminated
	H = J^T J / sigma^2,  S = H_cc - H_cX H_XX^-1 H_Xc  (H_XX block diagonal, 3x3 per point)
	"""
	H = (J.T @ J).tocsr() / sigma**2
	A = H[:n_keep, :n_keep].toarray()
	B = H[:n_keep, n_keep:]
	C = H[n_keep:, n_keep:]

	# Invert H_XX block by block
	idx = np.arange(C.shape[0]).reshape(-1,3)
	r, c = np.repeat(idx, 3, axis=1).ravel(), np.tile(idx, 3).ravel()
	blocks = np.asarray(C[r, c]).reshape(-1,3,3)
	C_inv = coo_matrix((np.linalg.inv(blocks).ravel(), (r, c)), shape=C.shape).tocsr()

	return A - (B @ C_inv @ B.T).toarray()


def reduced_hessian(J, im_ids, pt_ids, sigma, pid_fix=None):
	"""
	Points eliminated (Schur). Without pid_fix: S over [f, k1, all poses], keeps the 7-DOF gauge null space
	With pid_fix: gauge fixed -> S over [f, k1, poses of im_ids[1:]]
		pose of im_ids[0] fixed (6: shift + rotation), world Y of point pid_fix fixed (1: scale)
		Scale fixed vertically so the along-track stretch stays free
		(assumes world axes ~ first camera axes: Y down, Z along-track)
	Fixing = deleting rows/cols of S, same as deleting columns of J before the Schur step
	"""
	nc = 2 + 6*len(im_ids)
	if pid_fix is None:
		return schur_points(J, nc, sigma)

	# Move fixed point's 3 columns next to the camera columns so it is not eliminated
	p = 3*pt_ids.index(pid_fix)
	pt_cols = np.arange(nc, J.shape[1])
	perm = np.r_[np.arange(nc), pt_cols[p:p+3], np.delete(pt_cols, np.arange(p, p+3))]
	S = schur_points(J[:, perm], nc + 3, sigma)

	# Fix gauge: drop first pose and fixed point Y
	keep = np.delete(np.arange(nc + 3), np.r_[2:8, nc + 1])
	S = S[np.ix_(keep, keep)]

	# Eliminate fixed point X, Z (last 2)
	b = S[:-2, -2:]
	return S[:-2, :-2] - b @ np.linalg.solve(S[-2:, -2:], b.T)


def cam_centers(ims, im_ids):
	"""
	Camera centers in world frame, C = -R^T t, (N,3)
	"""
	return np.array([-ims[i]['R'].T @ ims[i]['t'] for i in im_ids])


def focal_bias(S, ims, im_ids, df):
	"""
	Linear response when f is held wrong by df and everything else re-adjusts (k1, poses, points via Schur)
	S from reduced_hessian with gauge: [f, k1, poses of im_ids[1:]]
	dc = -S_cc^-1 S_cf df ;  camera center shift dC = -R^T (dt + t x dw)
	Returns dC (N,3) for all im_ids (first image fixed -> 0), dk1
	"""
	dc = -np.linalg.solve(S[1:, 1:], S[1:, 0]) * df
	dpose = dc[1:].reshape(-1,6)
	dC = np.zeros((len(im_ids),3))
	for n, iid in enumerate(im_ids[1:], 1):
		R, t = ims[iid]['R'], ims[iid]['t']
		dw, dt = dpose[n-1, :3], dpose[n-1, 3:]
		dC[n] = -R.T @ (dt + np.cross(t, dw))
	return dC, dc[0]


def ground_plane(ims, pts, ids, v_min=1000, n_iter=5):
	"""
	Robust plane fit to near-field ground: points seen below pixel row v_min in any image of ids
	(v_min = 1000 of 1376 assumes the bottom band of the image is road/runway)
	Iterative SVD fit, dropping points beyond 3 robust std (MAD) each pass
	Returns point on plane g0, unit normal n
	"""
	pid = np.unique(np.concatenate([ims[i]['pid'][ims[i]['xy'][:,1] > v_min] for i in ids]))
	G = np.array([pts[p] for p in pid])
	keep = np.ones(len(G), dtype=bool)
	for _ in range(n_iter):
		g0 = G[keep].mean(0)
		n = np.linalg.svd(G[keep] - g0, full_matrices=False)[2][-1]
		r = (G - g0) @ n
		med = np.median(r[keep])
		keep = np.abs(r - med) < 3 * 1.4826 * np.median(np.abs(r[keep] - med))
	return g0, n


def track_axes(C, g0, n):
	"""
	Along-track direction (travel direction, PCA of C, projected into ground plane) and camera height
	C ordered in time. Returns along (3,), camera height above plane (median)
	"""
	a = np.linalg.svd(C - C.mean(0), full_matrices=False)[2][0]
	a = a - (a @ n)*n
	a /= np.linalg.norm(a)
	if (C[-1] - C[0]) @ a < 0:
		a = -a
	return a, np.median(np.abs((C - g0) @ n))


def along_track(path, ref, min_obs=500):
	"""
	Along-track position of each camera relative to image ref, in camera-height units (gauge free)
	Images with < min_obs observations treated as misregistered and dropped
	Returns {image name: position}, camera height (COLMAP units)
	"""
	cams, ims, pts = load_colmap_txt(path)
	ids = sorted((i for i in ims if len(ims[i]['pid']) >= min_obs), key=lambda i: ims[i]['name'])
	names = [ims[i]['name'] for i in ids]
	C = cam_centers(ims, ids)
	a, h = track_axes(C, *ground_plane(ims, pts, ids))
	return dict(zip(names, (C - C[names.index(ref)]) @ a / h)), h
