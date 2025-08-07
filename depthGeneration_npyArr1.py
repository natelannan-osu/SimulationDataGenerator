import bpy
import sys
import os
import argparse
import numpy as np
from mathutils import Vector
import math
import pdb

# ------------------------------------------------------------
# PARSE ARGS (Blender-friendly)
# ------------------------------------------------------------

if "--" in sys.argv:
    argv = sys.argv[sys.argv.index("--") + 1:]
else:
    argv = []

parser = argparse.ArgumentParser(description="Blender BVH Retarget + Depth Render Script with Defaults")
parser.add_argument("--bvh", default="./cmu-mocap/data/001/01_01.bvh", help="Path to BVH motion file (default: ./cmu-mocap/data/001/01_01.bvh)")
parser.add_argument("--smpl", default="./smpl_data/basicModel_m_lbs_10_207_0_v1.0.2.fbx", help="Path to SMPL FBX file (default: ./smpl_data/basicModel_m_lbs_10_207_0_v1.0.2.fbx)")
parser.add_argument("--output-depth", default="./output/depth/", help="Output dir for depth EXRs")
parser.add_argument("--output-joints", default="./output/joints/", help="Output dir for joints")
parser.add_argument("--output-points", default="./output/points/", help="Output dir for point clouds")
parser.add_argument("--fps", type=int, default=60, help="Frames per second (default: 60)")

args = parser.parse_args(argv)

BVH_PATH = args.bvh
SMPL_PATH = args.smpl
OUTPUT_DEPTH_DIR = args.output_depth
OUTPUT_JOINTS_DIR = args.output_joints
OUTPUT_POINTS_DIR = args.output_points
FRAME_RATE = args.fps

CAMERA_DISTANCE = 375.0
# CAMERA_HEIGHT = 85.0 calculate from COM
# CAMERA_DISTANCE = .08
# CAMERA_HEIGHT = .01
# IMAGE_RESOLUTION = (512, 512)  # Default
IMAGE_RESOLUTION = (640, 480)  # kinect v1
# IMAGE_RESOLUTION = (512, 424)  # kinect v2
# FX = FY = 500.0
# CX = IMAGE_RESOLUTION[0] / 2.0
# CY = IMAGE_RESOLUTION[1] / 2.0


# ------------------------------------------------------------
# CAMERA SETTING NOTES
#
# Kinect v1 (Xbox 360)
#    Resolution: 640×480
#    Horizontal FOV: ~57°
#    Vertical FOV: ~43°
#    Focal length (approx):
#        fx ≈ fy ≈ 575 pixels
#    Sensor width (approximated): ~25 mm
#    Recommended Blender settings:
# cam_data.lens = 35.0  # in mm (approximate for 57° FOV with 25 mm sensor)
# cam_data.sensor_width = 25.0
# scene.render.resolution_x = 640
# scene.render.resolution_y = 480
#
# Kinect v2 (Xbox One)
#     Resolution: 512×424
#     Horizontal FOV: ~70.6°
#     Vertical FOV: ~60°
#     Focal length (approx):
#         fx ≈ 365, fy ≈ 365
#     Sensor width: ~24 mm
#     Recommended Blender settings:
# cam_data.lens = 25.0  # for wider FOV
# cam_data.sensor_width = 24.0
# scene.render.resolution_x = 512
# scene.render.resolution_y = 424

# To match Kinect behavior more closely:
#     Set cam_data.sensor_fit = 'HORIZONTAL' (default)
#     Ensure camera type is 'PERSP' (not orthographic)
#     Position camera 1–2 meters away from the subject
#     Keep subject within ~0.5 to 4.5 meters (Kinect v2 working range)
# ------------------------------------------------------------


# ------------------------------------------------------------
# UTILS
# ------------------------------------------------------------

def ensure_directory(path):
    if not os.path.exists(path):
        os.makedirs(path)

ensure_directory(OUTPUT_DEPTH_DIR)
ensure_directory(OUTPUT_JOINTS_DIR)
ensure_directory(OUTPUT_POINTS_DIR)

# ------------------------------------------------------------
# CLEAN SCENE
# ------------------------------------------------------------

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
# bpy.context.scene.unit_settings.system = 'METRIC'
# bpy.context.scene.unit_settings.scale_length = 1.0


# ------------------------------------------------------------
# ENABLE DEPTH PASS
# ------------------------------------------------------------

bpy.context.view_layer.use_pass_z = True

# ------------------------------------------------------------
# LOAD SMPL FBX
# ------------------------------------------------------------

print(f"Importing SMPL FBX from {SMPL_PATH}")
bpy.ops.import_scene.fbx(filepath=SMPL_PATH, global_scale=1.0)

# --- Find the imported armature ---
smpl_armature = None
for obj in bpy.context.selected_objects:
    if obj.type == 'ARMATURE':
        smpl_armature = obj
        break

if not smpl_armature:
    raise Exception("Could not find SMPL armature in FBX!")

print(f"Found SMPL armature: {smpl_armature.name}")

# --- RESCALE FROM CM TO M ---
smpl_armature.scale = (100.0, 100.0, 100.0)
bpy.context.view_layer.update()

# --- APPLY SCALE ---
bpy.context.view_layer.objects.active = smpl_armature
bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)

print(f"Found SMPL armature: {smpl_armature.name}")
print(f"Armature scale: {smpl_armature.scale}")
# for bone in smpl_armature.pose.bones:
#     world_pos = smpl_armature.matrix_world @ bone.head
#     print(f"{bone.name}: {world_pos}")

# ------------------------------------------------------------
# CALCULATE CENTER OF MASS (COM) OF SKELETON
# SOMETHING WRONG HERE!!!!!!
# ------------------------------------------------------------
def get_armature_com(armature):
    joints = []
    for bone in armature.pose.bones:
        world_pos = armature.matrix_world @ bone.head
        joints.append(np.array(world_pos))
    joints = np.stack(joints)
    return joints.mean(axis=0)

skeleton_com = get_armature_com(smpl_armature)
print(f"[INFO] Skeleton COM: {skeleton_com}")

CAMERA_HEIGHT = .85*skeleton_com[2]  # armature is defined z down

# ------------------------------------------------------------
# IMPORT BVH
# ------------------------------------------------------------

print(f"Importing BVH from {BVH_PATH}")
bpy.ops.import_anim.bvh(filepath=BVH_PATH, axis_forward='-Z', axis_up='Y')
bvh_armature = [obj for obj in bpy.context.scene.objects if obj.type == 'ARMATURE' and obj != smpl_armature][0]
print(f"Found BVH armature: {bvh_armature.name}")

# # ------------------------------------------------------------
# # RETARGETING -- old
# # ------------------------------------------------------------

print("Adding retargeting constraints...")
for bone in smpl_armature.pose.bones:
    constraint = bone.constraints.new(type='COPY_ROTATION')
    try:
        constraint.target = bvh_armature
        constraint.subtarget = bone.name
    except Exception as e:
        print(f"Warning: Could not set constraint for bone {bone.name}: {e}")

# ------------------------------------------------------------
# SET FRAMERATE
# ------------------------------------------------------------

scene.render.fps = FRAME_RATE

# ------------------------------------------------------------
# ADD CAMERA
# ------------------------------------------------------------

camera_data = bpy.data.cameras.new("DepthCam")
camera_obj = bpy.data.objects.new("DepthCam", camera_data)
bpy.context.collection.objects.link(camera_obj)

# camera_obj.location = (0, -CAMERA_DISTANCE, 20.0)
# camera_obj.location = (-CAMERA_DISTANCE, 0,  1.0)
# camera_obj.location = (-CAMERA_DISTANCE, 0,  1.0)
camera_obj.location = (0, CAMERA_HEIGHT, CAMERA_DISTANCE+skeleton_com[1])

# camera_obj.rotation_euler = (math.pi / 2, 0, 0)
camera_obj.rotation_euler = (0, 0, 0)
scene.camera = camera_obj

print(camera_obj.location)
print(camera_obj.location.z)



# ------------------------------------------------------------
# CALCULATE AND SAVE CAMERA INTRINSICS ONCE
# ------------------------------------------------------------
scene = bpy.context.scene
cam_data = camera_obj.data

# lens default is 50mm
cam_data.lens = 35 # kinect v1
# cam_data.lens = 25 # kinect v2

# sensor width default is 36mm
cam_data.sensor_width = 25.0 # Kinect v1
# cam_data.sensor_width = 24.0 # Kinect v2

f_mm = cam_data.lens
sensor_width = cam_data.sensor_width
sensor_height = cam_data.sensor_height

fit = cam_data.sensor_fit
if fit == 'VERTICAL':
    FY = f_mm / sensor_height * IMAGE_RESOLUTION[1]
    FX = FY
else:
    FX = f_mm / sensor_width * IMAGE_RESOLUTION[0]
    FY = FX
# FX = f_mm / sensor_width * IMAGE_RESOLUTION[0]
# FY = f_mm / sensor_height * IMAGE_RESOLUTION[1]
CX = IMAGE_RESOLUTION[0]/ 2
CY = IMAGE_RESOLUTION[1] / 2

intrinsics_array = np.array([FX, FY, CX, CY])
np.save(os.path.join(OUTPUT_POINTS_DIR, "camera_intrinsics.npy"), intrinsics_array)

print(f"[INFO] Saved camera intrinsics: FX={FX:.2f}, FY={FY:.2f}, CX={CX}, CY={CY}")
# print(cam_data.sensor_fit)
# ------------------------------------------------------------
# RENDER SETTINGS
# ------------------------------------------------------------

scene.render.image_settings.file_format = 'OPEN_EXR'
scene.render.image_settings.color_depth = '32'
scene.render.resolution_x = IMAGE_RESOLUTION[0]
scene.render.resolution_y = IMAGE_RESOLUTION[1]
scene.use_nodes = True

# ------------------------------------------------------------
# SETUP COMPOSITOR
# ------------------------------------------------------------

tree = scene.node_tree
tree.nodes.clear()

rlayers = tree.nodes.new(type='CompositorNodeRLayers')
depth_output = tree.nodes.new(type='CompositorNodeOutputFile')
depth_output.base_path = OUTPUT_DEPTH_DIR
depth_output.format.file_format = 'OPEN_EXR'


tree.links.new(rlayers.outputs['Depth'], depth_output.inputs[0])
# tree.links.new(rlayers.outputs['Depth'], depth_output.inputs["Image"])

# ------------------------------------------------------------
# UTILS TO GET JOINTS
# ------------------------------------------------------------

def get_world_joint_positions(armature):
    result = {}
    for bone in armature.pose.bones:
        world_pos = armature.matrix_world @ bone.head
        result[bone.name] = np.array(world_pos)
    return result

def world_to_camera_coords(camera_obj, world_coords):
    cam_matrix_inv = camera_obj.matrix_world.inverted()
    return {
        name: np.array(cam_matrix_inv @ Vector(coord))
        for name, coord in world_coords.items()
    }

# ------------------------------------------------------------
# FRAME LOOP
# ------------------------------------------------------------

frame_start = scene.frame_start
#frame_end = scene.frame_end
frame_end = 10

print(f"camera location: {camera_obj.location}")# camera moving <-------------------- debug
print(f"COM: {skeleton_com}")

print(f"Rendering frames {frame_start} to {frame_end} at {FRAME_RATE} fps")
for frame in range(frame_start, frame_end + 1):
    print(f"Rendering frame {frame}...")

    scene.frame_set(frame)

    # ------------------------------------------------------------
    # UPDATE CAMERA
    # ------------------------------------------------------------
    # 1. Compute skeleton COM
    skeleton_com = get_armature_com(smpl_armature)

    # 2. Set camera position to follow subject
    cam_target = Vector((skeleton_com[0], 0, skeleton_com[1]))  # forward plane only
    cam_offset = Vector((0, 0, CAMERA_DISTANCE))  # fixed distance in -Z direction
    camera_obj.location = cam_target + cam_offset
    camera_obj.location.y = CAMERA_HEIGHT  # maintain fixed Y height
    camera_obj.rotation_euler = (0, 0, 0)  # change to specific angle if needed

    print(f"camera location: {camera_obj.location}")# camera moving <-------------------- debug
    print(f"COM: {skeleton_com}")


    # ------------------------------------------------------------
    # Render and write EXR to disk
    # ------------------------------------------------------------
    depth_output.file_slots[0].path = f"depth_{frame:04d}_"
    bpy.ops.render.render(write_still=True)

    # ------------------------------------------------------------
    # Load depth EXR just written
    # ------------------------------------------------------------
    import glob

    matching_files = sorted(glob.glob(os.path.join(OUTPUT_DEPTH_DIR, f"depth_{frame:04d}_*.exr")))
    if not matching_files:
        raise Exception(f"No EXR files found for frame {frame} in {OUTPUT_DEPTH_DIR}!")
    exr_path = matching_files[-1]  # Use the most recent EXR
    
    depth_image = bpy.data.images.load(exr_path)
    pixels = np.array(depth_image.pixels[:])
    if pixels.size == 0:
        raise Exception(f"EXR file {exr_path} is empty or invalid!")

    # Extract R channel (depth)
    depth_values = pixels[::4].reshape((IMAGE_RESOLUTION[1], IMAGE_RESOLUTION[0]))
    depth_image.user_clear()
    bpy.data.images.remove(depth_image)

    # ------------------------------------------------------------
    # Back-project to camera space
    # ------------------------------------------------------------
    u_grid, v_grid = np.meshgrid(np.arange(IMAGE_RESOLUTION[0]), np.arange(IMAGE_RESOLUTION[1]))
    # Z = depth_values
    Z = -depth_values
    X = (u_grid - CX) * Z / FX
    Y = -(v_grid - CY) * Z / FY
    points_camera = np.stack([X, Y, Z], axis=-1)

    # ------------------------------------------------------------
    # Transform to world space
    # ------------------------------------------------------------
    cam_matrix = camera_obj.matrix_world
    points_flat = points_camera.reshape(-1, 3)
    points_world = [cam_matrix @ Vector(p) for p in points_flat]
    points_world = np.array([p[:] for p in points_world], dtype=np.float32).reshape((IMAGE_RESOLUTION[1], IMAGE_RESOLUTION[0], 3))

    # ------------------------------------------------------------
    # Save point cloud
    # ------------------------------------------------------------
    np.save(os.path.join(OUTPUT_POINTS_DIR, f"frame_{frame:04d}_points.npy"), points_world)

    # ------------------------------------------------------------
    # Export joints
    # ------------------------------------------------------------
    world_joints = get_world_joint_positions(smpl_armature)
    camera_joints = world_to_camera_coords(camera_obj, world_joints)
    world_array = np.array([world_joints[k] for k in sorted(world_joints.keys())])
    cam_array = np.array([camera_joints[k] for k in sorted(camera_joints.keys())])
    np.save(os.path.join(OUTPUT_JOINTS_DIR, f"frame_{frame:04d}_world.npy"), world_array)
    np.save(os.path.join(OUTPUT_JOINTS_DIR, f"frame_{frame:04d}_camera.npy"), cam_array)

    # ------------------------------------------------------------
    # Export camera info
    # ------------------------------------------------------------
    cam_matrix = camera_obj.matrix_world
    position = np.array(cam_matrix.to_translation())
    print(position)
    rotation = np.array(cam_matrix.to_euler())
    print(rotation)

    np.save(os.path.join(OUTPUT_POINTS_DIR, f"frame_{frame:04d}_camera_location.npy"), position)
    np.save(os.path.join(OUTPUT_POINTS_DIR, f"frame_{frame:04d}_camera_rotation.npy"), rotation)

print("All frames rendered and saved!")
