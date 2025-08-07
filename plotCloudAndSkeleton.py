import numpy as np
import matplotlib.pyplot as plt
import argparse
from scipy.spatial.transform import Rotation as R
import pdb

# ------------------------------------------------------------
# PARSE ARGS
# ------------------------------------------------------------

parser = argparse.ArgumentParser(description="Plot 3D point cloud, joints, and camera pose")
parser.add_argument("--points", required=True, help="Path to .npy file with (H,W,3) world points")
parser.add_argument("--joints", required=True, help="Path to .npy file with (N,3) world joints")
parser.add_argument("--camera_location", required=True, help="Path to .npy file with camera position (3,)")
parser.add_argument("--camera_rotation", required=True, help="Path to .npy file with camera rotation euler (3,)")
parser.add_argument("--sample", type=int, default=2, help="Subsampling factor for dense point cloud (default=2)")
args = parser.parse_args()

# ------------------------------------------------------------
# LOAD DATA
# ------------------------------------------------------------

print("[INFO] Loading data...")
points = np.load(args.points)
joints = np.load(args.joints)
camera_pos = np.load(args.camera_location)
camera_rot = np.load(args.camera_rotation)

print(f"Loaded points: {points.shape}")
print(f"Loaded joints: {joints.shape}")
print(f"Camera position: {camera_pos}")
print(f"Camera rotation (Euler XYZ radians): {camera_rot}")
print(f"Joints COM: {np.mean(joints, axis=0)}")


# ------------------------------------------------------------
# SUBSAMPLE POINT CLOUD
# ------------------------------------------------------------

points_sub = points[::args.sample, ::args.sample, :].reshape(-1, 3)
points_sub = points_sub[points_sub[:,2] > -100]

# ------------------------------------------------------------
# COMPUTE CAMERA FACING DIRECTION
# ------------------------------------------------------------

# Blender's camera default looks along -Z in local space
cam_forward_local = np.array([0, 0, -1])

# Convert Euler angles to rotation matrix
rot = R.from_euler('XYZ', camera_rot)
cam_forward_world = rot.apply(cam_forward_local)

print(f"Camera forward direction in world space: {cam_forward_world}")

# ------------------------------------------------------------
# PLOT
# ------------------------------------------------------------

fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Point cloud
ax.scatter(points_sub[:, 0], points_sub[:, 1], (points_sub[:, 2]),
           s=0.5, c=points_sub[:, 2], cmap='gray', alpha=0.5, label='Point Cloud')

# Joints
ax.scatter(joints[:, 0], joints[:, 1], joints[:, 2],
           s=40, c='red', label='Joints')

# Camera position
ax.scatter(camera_pos[0], camera_pos[1], camera_pos[2],
           s=50, c='blue', label='Camera Position')

# Camera facing direction as arrow
arrow_length = 50
ax.quiver(
    camera_pos[0], camera_pos[1], camera_pos[2],
    cam_forward_world[0], cam_forward_world[1], cam_forward_world[2],
    length=arrow_length,
    color='blue',
    normalize=True
)

# ------------------------------------------------------------
# LABELS AND VIEW
# ------------------------------------------------------------

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('3D Point Cloud with Joints and Camera Pose')
ax.legend()
ax.view_init(elev=30, azim=45)
plt.tight_layout()
plt.show()

