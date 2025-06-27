import pybullet as p
import pybullet_data
import time
import os

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
YCB_ASSETS_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../data/ycb_urdfs/ycb_assets'))
p.setAdditionalSearchPath(YCB_ASSETS_PATH)

def load_ycb_object(urdf_name, position, orientation_euler, scaling=0.08):
    orientation_q = p.getQuaternionFromEuler(orientation_euler)
    urdf_path = os.path.join('ycb_assets', urdf_name)
    return p.loadURDF(urdf_path, position, orientation_q, useFixedBase=False, globalScaling=scaling)

# Example objects to load
objects_to_load = [
    ("077_rubiks_cube.urdf", [0.0, 0.0, 0.1], [0.0, 0.0, 0.0]),
    ("073-g_lego_duplo.urdf", [-0.5, 0.0, 0.1], [0.0, 0.0, 0.0]),
    ("071_nine_hole_peg_test.urdf", [0.0, -0.5, 0.1], [0.0, 0.0, 0.0]),
]

# Load the objects
for urdf_name, pos, orn in objects_to_load:
    load_ycb_object(urdf_name, pos, orn)

# Keep the simulation running
while True:
    p.stepSimulation()
    time.sleep(control_dt) 