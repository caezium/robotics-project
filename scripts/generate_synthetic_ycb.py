import os
import random
import pybullet as p
import pybullet_data
import numpy as np
import cv2
from glob import glob

# --- CONFIG ---
URDF_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '../assets/urdf/ycb'))
OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '../data/synthetic')
IMG_DIR = os.path.join(OUTPUT_DIR, 'images')
LBL_DIR = os.path.join(OUTPUT_DIR, 'labels')
CLASSES_FILE = os.path.join(OUTPUT_DIR, 'classes.txt')

NUM_IMAGES = 1000  # number of synthetic images to generate
OBJECTS_PER_SCENE = (1, 3)  # min, max objects per scene
IMG_WIDTH, IMG_HEIGHT = 640, 480 # dimensions

# Camera parameters (top-down)
CAMERA_POS = [0, 0, 2]  # position, topdown
CAMERA_TARGET = [0, 0, 0]  # lookat
CAMERA_UP = [0, 10, 0]
FOV = 60  # in degrees
NEAR, FAR = 0.1, 100.0  # Near and far clipping planes for the camera

# --- UTILS ---
def get_urdf_objects(urdf_dir):
    urdf_files = glob(os.path.join(urdf_dir, '*.urdf'))
    class_names = [os.path.splitext(os.path.basename(f))[0] for f in urdf_files]
    return urdf_files, class_names

def ensure_dirs():
    os.makedirs(IMG_DIR, exist_ok=True)
    os.makedirs(LBL_DIR, exist_ok=True)

def save_classes(class_names):
    with open(CLASSES_FILE, 'w') as f:
        for name in class_names:
            f.write(f"{name}\n")

# --- MAIN GENERATION LOOP ---
def main():
    ensure_dirs()
    urdf_files, class_names = get_urdf_objects(URDF_DIR)
    print("Looking for URDFs in:", URDF_DIR)
    save_classes(class_names)
    
    # Start PyBullet
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    for i in range(NUM_IMAGES):
        p.resetSimulation()
        p.loadURDF("plane.urdf")
        n_objs = random.randint(*OBJECTS_PER_SCENE)
        objs = []
        obj_classes = []
        obj_infos = []  # Store (obj_id, class_id, urdf) for later
        for _ in range(n_objs):
            idx = random.randint(0, len(urdf_files)-1)
            urdf = urdf_files[idx]
            class_id = idx
            pos = [random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1), 0.2]  # raised z
            orn = p.getQuaternionFromEuler([0, 0, random.uniform(0, 2*np.pi)])
            obj_id = p.loadURDF(urdf, pos, orn, useFixedBase=False)
            objs.append(obj_id)
            obj_classes.append(class_id)
            obj_infos.append((obj_id, class_id, urdf))
            
            # Debug print for object position
            base_pos, base_orn = p.getBasePositionAndOrientation(obj_id)
            print(f"Object {obj_id} position: {base_pos}, orientation: {base_orn}")
        
        # Let objects settle
        for _ in range(100):  # add more steps for settling
            p.stepSimulation()
        
        # Camera setup
        print(f"Camera position: {CAMERA_POS}, target: {CAMERA_TARGET}, up: {CAMERA_UP}")
        view_matrix = p.computeViewMatrix(CAMERA_POS, CAMERA_TARGET, CAMERA_UP)
        proj_matrix = p.computeProjectionMatrixFOV(FOV, IMG_WIDTH/IMG_HEIGHT, NEAR, FAR)
        img_arr = p.getCameraImage(IMG_WIDTH, IMG_HEIGHT, view_matrix, proj_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        rgb = np.reshape(img_arr[2], (IMG_HEIGHT, IMG_WIDTH, 4))[:, :, :3]
        
        # Save image
        img_path = os.path.join(IMG_DIR, f"img_{i:05d}.png")
        cv2.imwrite(img_path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        
        # Prepare YOLO label using 3D AABB projected to 2D
        label_lines = []
        for obj_id, class_id, urdf in obj_infos:
            aabb_min, aabb_max = p.getAABB(obj_id)
            # 8 corners of the AABB
            corners = np.array([
                [aabb_min[0], aabb_min[1], aabb_min[2]],
                [aabb_min[0], aabb_min[1], aabb_max[2]],
                [aabb_min[0], aabb_max[1], aabb_min[2]],
                [aabb_min[0], aabb_max[1], aabb_max[2]],
                [aabb_max[0], aabb_min[1], aabb_min[2]],
                [aabb_max[0], aabb_min[1], aabb_max[2]],
                [aabb_max[0], aabb_max[1], aabb_min[2]],
                [aabb_max[0], aabb_max[1], aabb_max[2]],
            ])
            # Project 3D points to 2D
            def project(pt):
                pt_hom = np.array([pt[0], pt[1], pt[2], 1.0])
                vm = np.array(view_matrix).reshape(4,4,order='F')
                pm = np.array(proj_matrix).reshape(4,4,order='F')
                clip = pm @ (vm @ pt_hom)
                ndc = clip[:3] / clip[3]
                x = int((ndc[0] * 0.5 + 0.5) * IMG_WIDTH)
                y = int((1.0 - (ndc[1] * 0.5 + 0.5)) * IMG_HEIGHT)
                return x, y
            pts_2d = np.array([project(corner) for corner in corners])
            x_min, y_min = pts_2d.min(axis=0)
            x_max, y_max = pts_2d.max(axis=0)
            # Clamp to image
            x_min = max(0, min(IMG_WIDTH-1, x_min))
            x_max = max(0, min(IMG_WIDTH-1, x_max))
            y_min = max(0, min(IMG_HEIGHT-1, y_min))
            y_max = max(0, min(IMG_HEIGHT-1, y_max))
            # Skip if not visible
            if x_max <= x_min or y_max <= y_min:
                continue
            # YOLO format
            x_center = (x_min + x_max) / 2 / IMG_WIDTH
            y_center = (y_min + y_max) / 2 / IMG_HEIGHT
            w = (x_max - x_min) / IMG_WIDTH
            h = (y_max - y_min) / IMG_HEIGHT
            label_lines.append(f"{class_id} {x_center:.6f} {y_center:.6f} {w:.6f} {h:.6f}")
        # Save label file only if there are objects
        label_path = os.path.join(LBL_DIR, f"img_{i:05d}.txt")
        with open(label_path, 'w') as f:
            f.write('\n'.join(label_lines))
        if (i+1) % 50 == 0:
            print(f"Generated {i+1}/{NUM_IMAGES} images")
    
    p.disconnect()
    print("Done generating synthetic dataset!")

if __name__ == "__main__":
    main() 