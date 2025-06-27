import pybullet as p
import pybullet_data
import time
import os
import sys

# Set up the physics client
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
plane = p.loadURDF("plane.urdf")
control_dt = 1. / 240.

# Camera properties
camera_distance = 0.8
camera_yaw = 135.0
camera_pitch = -45.0
camera_target_position = [0.0, 0.0, 0.3]
p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

# Path to YCB assets
YCB_ASSETS_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), 'ycb'))
YCB_VARIANTS_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), 'ycb_variants'))

# Debug: print working directory and search paths
print(f"Current working directory: {os.getcwd()}")
print(f"YCB_ASSETS_PATH: {YCB_ASSETS_PATH}")
print(f"YCB_VARIANTS_PATH: {YCB_VARIANTS_PATH}")

# --- CONFIG: Choose which set to load ---
LOAD_VARIANTS = True  # Set to True to load variants, False for originals, idk why if we set both for additional search path it just wont load anything

# Only add the search path for the set being loaded
if LOAD_VARIANTS:
    p.setAdditionalSearchPath(YCB_VARIANTS_PATH)
else:
    p.setAdditionalSearchPath(YCB_ASSETS_PATH)

def load_ycb_object(urdf_name, position, orientation_euler, scaling=0.08):
    orientation_q = p.getQuaternionFromEuler(orientation_euler)
    return p.loadURDF(urdf_name, position, orientation_q, useFixedBase=False, globalScaling=scaling)

objects_to_load = [
    ("002_master_chef_can.urdf", [0.0, 0.0, 0.1], [0.0, 0.0, 0.0]),
    ("003_cracker_box.urdf", [-0.5, 0.0, 0.1], [0.0, 0.0, 0.0]),
    ("004_sugar_box.urdf", [0.0, -0.5, 0.1], [0.0, 0.0, 0.0]),
]

variant_objects_to_load = [
    ("002_master_chef_can_var0.urdf", [0.0, 0.0, 0.1], [0.0, 0.0, 0.0]),
    ("002_master_chef_can_var1.urdf", [-0.5, 0.0, 0.1], [0.0, 0.0, 0.0]),
    ("002_master_chef_can_var2.urdf", [0.0, -0.5, 0.1], [0.0, 0.0, 0.0]),
]

# Load the objects
if LOAD_VARIANTS:
    for urdf_name, pos, orn in variant_objects_to_load:
        load_ycb_object(urdf_name, pos, orn)
else:
    for urdf_name, pos, orn in objects_to_load:
        load_ycb_object(urdf_name, pos, orn)

# Keep the simulation running
while True:
    p.stepSimulation()
    time.sleep(control_dt) 