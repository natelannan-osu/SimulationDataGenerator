import OpenEXR
import Imath
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
import sys

# ------------------------------------------------------------
# ARGPARSE
# ------------------------------------------------------------
parser = argparse.ArgumentParser(description="View and export EXR R-channel depth map as normalized PNG")
parser.add_argument("--exr", required=True, help="Path to input EXR file")
parser.add_argument("--out", help="Optional output PNG path")
args = parser.parse_args()

exr_path = args.exr
if not os.path.isfile(exr_path):
    print(f"Error: file not found: {exr_path}")
    sys.exit(1)

# ------------------------------------------------------------
# LOAD EXR FILE
# ------------------------------------------------------------
exr_file = OpenEXR.InputFile(exr_path)
header = exr_file.header()

dw = header['dataWindow']
width = dw.max.x - dw.min.x + 1
height = dw.max.y - dw.min.y + 1

# ------------------------------------------------------------
# READ R CHANNEL AS FLOAT
# ------------------------------------------------------------
pt = Imath.PixelType(Imath.PixelType.FLOAT)

if 'R' not in header['channels']:
    print(f"Error: No 'R' channel found in EXR file {exr_path}")
    sys.exit(1)

r_channel = exr_file.channel('R', pt)
r_data = np.frombuffer(r_channel, dtype=np.float32).reshape((height, width))

# ------------------------------------------------------------
# NORMALIZE FOR VISUALIZATION
# ------------------------------------------------------------
r_min, r_max = np.min(r_data), np.max(r_data)
if r_max - r_min == 0:
    r_norm = np.zeros_like(r_data)
else:
    r_norm = (r_data - r_min) / (r_max - r_min)

# ------------------------------------------------------------
# OUTPUT PATH
# ------------------------------------------------------------
if args.out:
    out_path = args.out
else:
    base = os.path.splitext(os.path.basename(exr_path))[0]
    out_path = f"{base}_r_preview.png"

# ------------------------------------------------------------
# SAVE & SHOW
# ------------------------------------------------------------
plt.imsave(out_path, r_norm, cmap="plasma")
print(f"Saved normalized preview to: {out_path}")

plt.imshow(r_norm, cmap='plasma')
plt.title("Depth Map (from R channel, normalized)")
plt.colorbar(label="Normalized Depth")
plt.show()