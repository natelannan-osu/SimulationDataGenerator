import os
import glob

# Directories to clean
directories = [
    "./output/depth/",
    "./output/joints/",
    "./output/points/"
]

for dir_path in directories:
    if os.path.exists(dir_path):
        files = glob.glob(os.path.join(dir_path, "*"))
        for f in files:
            try:
                os.remove(f)
                print(f"Deleted: {f}")
            except Exception as e:
                print(f"Failed to delete {f}: {e}")
    else:
        print(f"Directory not found: {dir_path}")