import bpy
import sys
import argparse
import os

# ------------------------------------------------------------
# PARSE ARGS (Blender-friendly)
# ------------------------------------------------------------
if "--" in sys.argv:
    argv = sys.argv[sys.argv.index("--") + 1:]
else:
    argv = []

parser = argparse.ArgumentParser(description="Load MPFB2 character, import BVH motion, and save .blend")
parser.add_argument("--bvh", default="./cmu-mocap/data/001/01_01.bvh", help="Path to BVH motion file (default: ./cmu-mocap/data/001/01_01.bvh)")
parser.add_argument("--output-blend", default="./output/blend", help="Output dir for blend file")
args = parser.parse_args(argv)

BVH_PATH = args.bvh
OUTPUT_BLEND_PATH = args.output_blend

# ------------------------------------------------------------
# CLEAN SCENE
# ------------------------------------------------------------
bpy.ops.wm.read_factory_settings(use_empty=True)

# ------------------------------------------------------------
# LOAD MPFB2 SAMPLE CHARACTER
# ------------------------------------------------------------
# Ensure MPFB2 is enabled and this asset path exists
bpy.ops.mpfb.load_character(character="mpfb2_smplx_male", import_as="HUMANOID", rigify=True)

# ------------------------------------------------------------
# IMPORT BVH
# ------------------------------------------------------------
bpy.ops.import_anim.bvh(filepath=BVH_PATH, axis_forward='-Z', axis_up='Y')

# ------------------------------------------------------------
# SAVE BLEND FILE
# ------------------------------------------------------------
bpy.ops.wm.save_as_mainfile(filepath=OUTPUT_BLEND_PATH)
print(f"[INFO] Scene saved to {OUTPUT_BLEND_PATH}")
