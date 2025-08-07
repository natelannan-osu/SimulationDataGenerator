import bpy
import sys
import os
import argparse
import numpy as np

# ------------------------------------------------------------
# PARSE ARGS (Blender-friendly)
# ------------------------------------------------------------

if "--" in sys.argv:
    argv = sys.argv[sys.argv.index("--") + 1:]
else:
    argv = []

parser = argparse.ArgumentParser(description="Blender BVH Retarget + Depth Render Script with Defaults")
parser.add_argument("--bvh", default="./cmu-mocap/data/001/01_01.bvh", help="Path to BVH motion file (default: ./cmu-mocap/data/001/01_01.bvh")
parser.add_argument("--smpl", default="./smpl_data/basicModel_m_lbs_10_207_0_v1.0.2.fbx", help="Path to SMPL FBX file (default: ./smpl_data/basicModel_m_lbs_10_207_0_v1.0.2.fbx)")
parser.add_argument("--output-depth", default="./output/depth/", help="Output directory for depth images (default: ./output/depth/)")
parser.add_argument("--output-joints", default="./output/joints/", help="Output directory for joint positions (default: ./output/joints/)")
parser.add_argument("--fps", type=int, default=60, help="Frames per second for animation (default: 60)")

args = parser.parse_args(argv)

BVH_PATH = args.bvh
SMPL_PATH = args.smpl
OUTPUT_DEPTH_DIR = args.output_depth
OUTPUT_JOINTS_DIR = args.output_joints
FRAME_RATE = args.fps

CAMERA_DISTANCE = 1.0
IMAGE_RESOLUTION = (512, 512)

# ------------------------------------------------------------
# UTILS
# ------------------------------------------------------------

def ensure_directory(path):
    if not os.path.exists(path):
        os.makedirs(path)

ensure_directory(OUTPUT_DEPTH_DIR)
ensure_directory(OUTPUT_JOINTS_DIR)

# ------------------------------------------------------------
# CLEAN SCENE
# ------------------------------------------------------------

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene

# ------------------------------------------------------------
# LOAD SMPL MODEL FROM FBX
# ------------------------------------------------------------

print(f"Importing SMPL FBX from {SMPL_PATH}")
bpy.ops.import_scene.fbx(filepath=SMPL_PATH)

# Identify imported objects (armature and mesh)
smpl_armature = None
smpl_mesh = None

for obj in bpy.context.selected_objects:
    if obj.type == 'ARMATURE':
        smpl_armature = obj
    elif obj.type == 'MESH':
        smpl_mesh = obj

if not smpl_armature:
    raise Exception("Could not find SMPL armature in imported FBX!")

print(f"Found SMPL armature: {smpl_armature.name}")
if smpl_mesh:
    print(f"Found SMPL mesh: {smpl_mesh.name}")

# ------------------------------------------------------------
# IMPORT BVH MOTION
# ------------------------------------------------------------

print(f"Importing BVH motion from {BVH_PATH}")
bpy.ops.import_anim.bvh(filepath=BVH_PATH, axis_forward='-Z', axis_up='Y')
bvh_armature = [obj for obj in bpy.context.scene.objects if obj.type == 'ARMATURE' and obj != smpl_armature][0]

print(f"Found BVH armature: {bvh_armature.name}")

# ------------------------------------------------------------
# RETARGETING - Add simple Copy Rotation constraints
# ------------------------------------------------------------

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

# Place 1m in front on -Y
camera_obj.location = (0, -CAMERA_DISTANCE, 1.0)
camera_obj.rotation_euler = (np.pi / 2, 0, 0)  # pointing +Y

scene.camera = camera_obj

# ------------------------------------------------------------
# SET RENDER SETTINGS
# ------------------------------------------------------------

scene.render.image_settings.file_format = 'OPEN_EXR'
scene.render.image_settings.color_depth = '32'
scene.render.resolution_x = IMAGE_RESOLUTION[0]
scene.render.resolution_y = IMAGE_RESOLUTION[1]
scene.use_nodes = True

# ------------------------------------------------------------
# CONFIGURE DEPTH NODE OUTPUT
# ------------------------------------------------------------
bpy.context.view_layer.use_pass_z = True  # <-- this line enables the Z (Depth) pass!

tree = scene.node_tree
tree.nodes.clear()

render_layers = tree.nodes.new(type='CompositorNodeRLayers')
depth_file_output = tree.nodes.new(type='CompositorNodeOutputFile')
depth_file_output.base_path = OUTPUT_DEPTH_DIR
depth_file_output.format.file_format = 'OPEN_EXR'

tree.links.new(render_layers.outputs['Depth'], depth_file_output.inputs[0])

# ------------------------------------------------------------
# UTILS TO GET JOINTS
# ------------------------------------------------------------

def get_world_joint_positions(armature):
    world_positions = {}
    for bone in armature.pose.bones:
        world_pos = armature.matrix_world @ bone.head
        world_positions[bone.name] = np.array(world_pos)
    return world_positions

from mathutils import Vector

def world_to_camera_coords(camera_obj, world_coords):
    camera_matrix = camera_obj.matrix_world.inverted()
    return {
        name: camera_matrix @ Vector(coord)
        for name, coord in world_coords.items()
    }

# ------------------------------------------------------------
# RENDER LOOP
# ------------------------------------------------------------

frame_start = scene.frame_start
frame_end = scene.frame_end

print(f"Rendering frames {frame_start} to {frame_end} at {FRAME_RATE} fps")
for frame in range(frame_start, frame_end + 1):
    scene.frame_set(frame)
    
    # Export joints
    world_joints = get_world_joint_positions(smpl_armature)
    camera_joints = world_to_camera_coords(camera_obj, world_joints)
    
    world_array = np.array([world_joints[k] for k in sorted(world_joints.keys())])
    cam_array = np.array([camera_joints[k] for k in sorted(camera_joints.keys())])
    
    np.save(os.path.join(OUTPUT_JOINTS_DIR, f"frame_{frame:04d}_world.npy"), world_array)
    np.save(os.path.join(OUTPUT_JOINTS_DIR, f"frame_{frame:04d}_camera.npy"), cam_array)
    
    # Render depth
    depth_file_output.file_slots[0].path = f"depth_{frame:04d}_"
    bpy.ops.render.render(write_still=True)

print("All frames rendered and joint coordinates saved!")