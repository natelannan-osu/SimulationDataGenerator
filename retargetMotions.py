import bpy
import os
import sys
import json
import argparse
from mathutils import Matrix

# ------------------------------------------------------------
# ARGS (Blender-friendly)
# ------------------------------------------------------------
if "--" in sys.argv:
    argv = sys.argv[sys.argv.index("--") + 1:]
else:
    argv = []

parser = argparse.ArgumentParser(description="Batch retarget CMU BVH motions to MPFB2 human and save .blend files")
parser.add_argument("--input-dir", required=True, help="Root directory containing CMU BVH files (nested)")
parser.add_argument("--reference-pose", required=True, help="Path to reference_pose.json")
parser.add_argument("--base-blend", required=True, help="Path to base .blend file (MPFB2 human in T-pose)")
parser.add_argument("--output-dir", required=True, help="Where to save .blend files (mirrored folder structure)")
parser.add_argument("--bvh-scale", type=float, default=0.056444, help="BVH import scale (default=0.056444)")

args = parser.parse_args(argv)

# ------------------------------------------------------------
# Helpers
# ------------------------------------------------------------
def load_reference_pose(armature, json_path):
    with open(json_path, 'r') as f:
        pose_data = json.load(f)
    for bone_name, matrix_values in pose_data.items():
        if bone_name not in armature.pose.bones:
            continue
        if len(matrix_values) != 16:
            print(f"[WARN] Bone {bone_name} does not have a 4x4 matrix")
            continue
        matrix_4x4 = Matrix([matrix_values[i:i+4] for i in range(0, 16, 4)])
        armature.pose.bones[bone_name].matrix_basis = matrix_4x4

def find_bvh_files(root):
    for dirpath, _, filenames in os.walk(root):
        for fname in filenames:
            if fname.lower().endswith(".bvh"):
                yield os.path.join(dirpath, fname)

# ------------------------------------------------------------
# Main Loop
# ------------------------------------------------------------
for bvh_path in find_bvh_files(args.input_dir):
    rel_path = os.path.relpath(bvh_path, args.input_dir)
    rel_no_ext = os.path.splitext(rel_path)[0]
    blend_output_path = os.path.join(args.output_dir, rel_no_ext + ".blend")
    os.makedirs(os.path.dirname(blend_output_path), exist_ok=True)

    print(f"[INFO] Processing: {bvh_path}")
    print(f"[INFO] Will save to: {blend_output_path}")

    bpy.ops.wm.open_mainfile(filepath=args.base_blend)

    # Get MPFB armature
    mpfb_armature = next((obj for obj in bpy.data.objects if obj.type == "ARMATURE"), None)
    if mpfb_armature is None:
        raise RuntimeError("No armature found in base blend file!")

    bpy.context.view_layer.objects.active = mpfb_armature
    bpy.ops.object.mode_set(mode='POSE')
    load_reference_pose(mpfb_armature, args.reference_pose)
    bpy.ops.object.mode_set(mode='OBJECT')

    # Import BVH
    bpy.ops.import_anim.bvh(filepath=bvh_path, global_scale=args.bvh_scale, axis_forward='-Z', axis_up='Y')
    bvh_armature = [obj for obj in bpy.context.selected_objects if obj.type == "ARMATURE"][0]

    # Retarget using new Rokoko (rsl) operators
    try:
        bpy.ops.object.select_all(action='DESELECT')

        # Select source (BVH) first, target (MPFB) last and active
        bvh_armature.select_set(True)
        mpfb_armature.select_set(True)
        bpy.context.view_layer.objects.active = mpfb_armature

        # Call Rokoko retargeting operator without arguments
        bpy.ops.rsl.retarget_animation()

    except Exception as e:
        print(f"[WARN] Retargeting failed for {bvh_path}: {e}")
        continue

    bpy.ops.wm.save_as_mainfile(filepath=blend_output_path)

print("Done retargeting all BVH files.")
