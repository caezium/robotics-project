import pybullet as p
import pybullet_data
import os

# List of URDFs to inspect (add more as needed)
similar_objects = [
    "065-a_cups.urdf",
    "065-b_cups.urdf",
    "065-c_cups.urdf",
    "065-d_cups.urdf",
    "065-e_cups.urdf",
    "065-f_cups.urdf",
    "065-g_cups.urdf",
    "065-h_cups.urdf",
    "065-i_cups.urdf",
    "065-j_cups.urdf",
    "024_bowl.urdf",
    "029_plate.urdf",
    "022_windex_bottle.urdf",
    "021_bleach_cleanser.urdf",
    "006_mustard_bottle.urdf",
    "005_tomato_soup_can.urdf",
    "007_tuna_fish_can.urdf",
    "002_master_chef_can.urdf",
    "003_cracker_box.urdf",
    "004_sugar_box.urdf",
    "010_potted_meat_can.urdf",
]

urdf_dir = "../assets/urdf/ycb"

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)  # No gravity
plane_id = p.loadURDF("plane.urdf", basePosition=[0, 0, -1.0])

# Grid parameters
num_cols = 6
spacing = 0.5

for idx, urdf_file in enumerate(similar_objects):
    urdf_path = os.path.join(urdf_dir, urdf_file)
    if not os.path.exists(urdf_path):
        print(f"File not found: {urdf_path}")
        continue
    row = idx // num_cols
    col = idx % num_cols
    x = col * spacing
    y = -row * spacing
    print(f"Loading: {urdf_file} at ({x}, {y}, 0.1)")
    p.loadURDF(urdf_path, basePosition=[x, y, 0.1], globalScaling=0.25)

print("All objects loaded in a grid. Press Ctrl+C in this terminal to exit.")
while True:
    p.stepSimulation()
