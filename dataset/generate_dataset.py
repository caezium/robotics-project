import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time
import os
from math import radians

# ----- Camera Class -----
class TopDownCamera:
    def __init__(self, img_width, img_height, camera_position, floor_plane_size):
        self._img_width = img_width
        self._img_height = img_height
        self._camera_position = camera_position
        self._floor_plane_size = floor_plane_size
        self._roll, self._pitch, self._yaw = 0, -90, 90

        target = camera_position.copy()
        target[2] = 0

        self._view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=target,
            distance=camera_position[2],
            yaw=self._yaw,
            pitch=self._pitch,
            roll=self._roll,
            upAxisIndex=2
        )

        aspect = img_width / img_height
        near, far = 0.01, 10
        fov = 2 * np.degrees(np.arctan((floor_plane_size / 2) / camera_position[2]))

        self._projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

    def get_image(self):
        img_arr = p.getCameraImage(
            width=self._img_width,
            height=self._img_height,
            viewMatrix=self._view_matrix,
            projectionMatrix=self._projection_matrix
        )
        rgba = np.reshape(np.array(img_arr[2], dtype=np.uint8), (self._img_height, self._img_width, 4))
        return rgba[:, :, :3]

    def get_pixel_world_coords(self, pixel_x, pixel_y):
        u = pixel_x / self._img_width
        v = 1.0 - (pixel_y / self._img_height)
        world_y = (u * self._floor_plane_size) - self._floor_plane_size / 2
        world_x = -(v * self._floor_plane_size - self._floor_plane_size / 2)
        return [world_x, world_y, 0.0]

# ----- PyBullet Setup -----
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

plane_id = p.loadURDF("plane.urdf", basePosition=[0, 0, -1.0])


# Conveyor belt
belt_length, belt_width, belt_height = 5, 1, 0.02
belt_pos = [-0.3, 0, belt_height / 2]
belt_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[belt_length/2, belt_width/2, belt_height/2])
belt_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[belt_length/2, belt_width/2, belt_height/2], rgbaColor=[0, 0, 0, 1])
belt_id = p.createMultiBody(0, belt_col, belt_vis, belt_pos)

"""
# Trash bins
bin_recycling = p.loadURDF("src/trash_bin.urdf", basePosition=[0.25, 0.8, -0.25], globalScaling=2.0, useFixedBase=True)
bin_trash = p.loadURDF("src/trash_bin.urdf", basePosition=[2.75, 0, -0.25], globalScaling=2.0, useFixedBase=True)

def set_color(body_id, color):
    visual_shapes = p.getVisualShapeData(body_id)
    for shape in visual_shapes:
        link_index = shape[1]
        p.changeVisualShape(body_id, link_index, rgbaColor=color)

blue = [0, 0, 1, 1]
black = [0, 0, 0, 1]
set_color(bin_recycling, blue)
set_color(bin_trash, black)

"""



# Overhead camera
camera = TopDownCamera(img_width=512, img_height=512, camera_position=[0, 0, 2], floor_plane_size=1)
CAMERA_POS = [0, 0, 2]  # position, topdown
CAMERA_TARGET = [0, 0, 0]  # lookat
CAMERA_UP = [0, 10, 0]
FOV = 60  # in degrees
NEAR, FAR = 0.1, 100.0  # Near and far clipping planes for the camera
IMG_WIDTH = 512
IMG_HEIGHT=512

view_matrix = p.computeViewMatrix(CAMERA_POS, CAMERA_TARGET, CAMERA_UP)
proj_matrix = p.computeProjectionMatrixFOV(FOV, IMG_WIDTH/IMG_HEIGHT, NEAR, FAR)

def project(pt):
    pt_hom = np.array([pt[0], pt[1], pt[2], 1.0])
    vm = np.array(view_matrix).reshape(4,4,order='F')
    pm = np.array(proj_matrix).reshape(4,4,order='F')
    clip = pm @ (vm @ pt_hom)
    ndc = clip[:3] / clip[3]
    x = int((ndc[0] * 0.5 + 0.5) * IMG_WIDTH)
    y = int((1.0 - (ndc[1] * 0.5 + 0.5)) * IMG_HEIGHT)
    return x, y


#load objects
#all cans, bottles, and cups need to have roll adjusted to lie flat on belt
pitch_adjust_list = ["src/urdf/ycb/002_master_chef_can.urdf", "src/urdf/ycb/003_cracker_box.urdf", "src/urdf/ycb/004_sugar_box.urdf", "src/urdf/ycb/005_tomato_soup_can.urdf", "src/urdf/ycb/006_mustard_bottle.urdf", "src/urdf/ycb/007_tuna_fish_can.urdf", "src/urdf/ycb/008_pudding_box.urdf", "src/urdf/ycb/009_gelatin_box.urdf", "src/urdf/ycb/010_potted_meat_can.urdf", "src/urdf/ycb/021_bleach_cleanser.urdf", "src/urdf/ycb/022_windex_bottle.urdf", "src/urdf/ycb/065-a_cups.urdf", "src/urdf/ycb/065-b_cups.urdf", "src/urdf/ycb/065-c_cups.urdf", "src/urdf/ycb/065-e_cups.urdf", "src/urdf/ycb/065-f_cups.urdf", "src/urdf/ycb/065-g_cups.urdf", "src/urdf/ycb/065-h_cups.urdf", "src/urdf/ycb/065-i_cups.urdf", "src/urdf/ycb/065-j_cups.urdf"]
urdf_dir = "src/urdf/ycb"
image_dir = "dataset/images"
label_dir = "dataset/labels"

class_id = 0
for i, file in enumerate(os.listdir(urdf_dir)):
    if i>4:
        break
    path = os.path.join(urdf_dir, file)
    print(path, class_id)
    if not os.path.isfile(path):
        continue
    if not file.endswith(".urdf"):
        continue
    for j in range(36):
        if file in pitch_adjust_list:
            rotation = [0,-90,10*j]
        else:
            rotation = [0,0,10*j]
        quaternion = p.getQuaternionFromEuler([radians(x) for x in rotation])
        object_id=p.loadURDF(path, basePosition=[0,0,1], baseOrientation=quaternion, globalScaling=0.25)
        for _ in range(120):
            p.stepSimulation()
            #comment out time.sleep for actual generation
            #time.sleep(1/240)

        img = camera.get_image()
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        base = os.path.splitext(os.path.basename(file))[0]
        img_name= base + f"{j}.jpeg"
        img_path = os.path.join(image_dir, img_name)
        cv2.imwrite(img_path, img_bgr)

        label = []
        aabb_min, aabb_max = p.getAABB(object_id)
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
        label.append(f"{class_id} {x_center:.6f} {y_center:.6f} {w:.6f} {h:.6f}")

        label_name = base+f"{j}.txt"
        label_path = os.path.join(label_dir, label_name)
        with open(label_path, 'w') as f:
            f.write("\n".join(label) + "\n")

       
        
        p.removeBody(object_id)
        p.stepSimulation()
    class_id += 1


# ----- Simulation Loop -----
while True:
    # Get camera image and display it
    img = camera.get_image()
    cv2.imshow("Camera View", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)
    
    # Step simulation
    p.stepSimulation()
    #comment out time.sleep for actual generation
    #time.sleep(1./240.)
