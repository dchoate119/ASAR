# COLMAP run with VT intrinsics FIXED - only poses + 3D points solved
# Same 121 images and exhaustive matching as the April run
# Run: bash run_colmap_vt.sh
set -e   # stop at the first failed command

IMS=~/Datasets/COLMAP/VTTI_airport_6fps_April16/Images_TSR
OUT=~/Datasets/COLMAP/VTTI_airport_6fps_VT_fixed   # all output here (outside repo) - copy txt files into repo after
# VT intrinsics read from the ROS yaml -> FULL_OPENCV order: fx fy cx cy k1 k2 p1 p2 k3 k4 k5 k6
# (yaml rational_polynomial coeffs are already in this k1..k6 order; needs PyYAML - run in py312)
YAML=~/ASAR/s4/datasets/KBCB_config/camera_forward.yaml
VT=$(python3 -c "
import yaml; c = yaml.safe_load(open('$YAML'))
K, d = c['camera_matrix']['data'], c['distortion_coefficients']['data']
print(','.join(map(str, [K[0], K[4], K[2], K[5]] + d)))")
echo "VT params: $VT"   # check before COLMAP runs

mkdir -p $OUT/sparse   # mapper needs the output folder to exist

colmap feature_extractor --database_path $OUT/database.db --image_path $IMS \
  --ImageReader.camera_model FULL_OPENCV --ImageReader.single_camera 1 --ImageReader.camera_params $VT
# Exhaustive = all pairs, same as April (its db has matches at every frame gap 1-120)
# Sequential only gave gaps 1,2,4..64 -> no wide-baseline initial pair, mapper failed
colmap exhaustive_matcher --database_path $OUT/database.db
colmap mapper --database_path $OUT/database.db --image_path $IMS --output_path $OUT/sparse \
  --Mapper.ba_refine_focal_length 0 --Mapper.ba_refine_principal_point 0 --Mapper.ba_refine_extra_params 0 \
  --Mapper.init_min_tri_angle 4 --Mapper.init_max_forward_motion 1.0 --Mapper.max_extra_param 200
# Init relaxed (defaults 16 deg / 0.95, same as April's project.ini): forward driving (z ~0.9998) -> neighbor pairs
# fail the angle/forward checks, far pairs have ~30 inliers -> "bad initial pair". Only affects the start pair.
# max_extra_param (default 1): any |distortion coeff| > 1 marks the camera "bogus" -> 0 pts triangulated.
# VT rational coeffs go up to ~96, so raise the limit (does not change the coeffs - they stay fixed).
colmap model_converter --input_path $OUT/sparse/0 --output_path $OUT --output_type TXT
