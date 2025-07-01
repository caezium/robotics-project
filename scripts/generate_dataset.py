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
        return [world_x, world_y]
    
    def get_pixel_coords(self, world_x, world_y):
        u = (world_y + self._floor_plane_size / 2) / self._floor_plane_size
        v = (self._floor_plane_size / 2 - world_x) / self._floor_plane_size

        pixel_x = round(u * self._img_width, 6)
        pixel_y = round((1.0 - v) * self._img_height, 6)

        return pixel_x, pixel_y

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
bin_recycling = p.loadURDF("assets/urdf/trash_bin.urdf", basePosition=[0.25, 0.8, -0.25], globalScaling=2.0, useFixedBase=True)
bin_trash = p.loadURDF("assets/urdf/trash_bin.urdf", basePosition=[2.75, 0, -0.25], globalScaling=2.0, useFixedBase=True)

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
camera = TopDownCamera(img_width=512, img_height=512, camera_position=[0, 0, 3], floor_plane_size=1)
CAMERA_POS = [0, 0, 3]  # position, topdown
CAMERA_TARGET = [0, 0, 0]  # lookat
CAMERA_UP = [0, 10, 0]
FOV = 60  # in degrees
NEAR, FAR = 0.1, 100.0  # Near and far clipping planes for the camera
IMG_WIDTH = 512
IMG_HEIGHT=512



#load objects
#all cans, bottles, and cups need to have roll adjusted to lie flat on belt
pitch_adjust_list = [
    "assets/urdf/ycb/002_master_chef_can.urdf", "assets/urdf/ycb/003_cracker_box.urdf", "assets/urdf/ycb/004_sugar_box.urdf", "assets/urdf/ycb/005_tomato_soup_can.urdf", "assets/urdf/ycb/006_mustard_bottle.urdf", "assets/urdf/ycb/007_tuna_fish_can.urdf", "assets/urdf/ycb/010_potted_meat_can.urdf", "assets/urdf/ycb/021_bleach_cleanser.urdf", "assets/urdf/ycb/022_windex_bottle.urdf", "assets/urdf/ycb/065-a_cups.urdf", "assets/urdf/ycb/065-b_cups.urdf", "assets/urdf/ycb/065-c_cups.urdf", "assets/urdf/ycb/065-d_cups.urdf", "assets/urdf/ycb/065-e_cups.urdf", "assets/urdf/ycb/065-f_cups.urdf", "assets/urdf/ycb/065-g_cups.urdf", "assets/urdf/ycb/065-h_cups.urdf", "assets/urdf/ycb/065-i_cups.urdf", "assets/urdf/ycb/065-j_cups.urdf"
]
urdf_dir = "assets/urdf/ycb"
image_dir = "data/processed/images"
label_dir = "data/processed/labels"

class_id = 0
for i, file in enumerate(os.listdir(urdf_dir)):
    path = os.path.join(urdf_dir, file).replace('\\', '/')
    print(path, class_id)
    if not os.path.isfile(path):
        continue
    if not file.endswith(".urdf"):
        continue
    for j in range(30):
        if path in pitch_adjust_list:
            rotation = [0,-90,12*j]
        else:
            rotation = [0,0,12*j]
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
        top_corners = [
        [aabb_max[0], aabb_min[1], aabb_max[2]],  # Bottom-left of 2d camera
        [aabb_min[0], aabb_min[1], aabb_max[2]],  # Top-left of 2d camera
        [aabb_max[0], aabb_max[1], aabb_max[2]],  # Bottom-right of 2d camera
        [aabb_min[0], aabb_max[1], aabb_max[2]]   # Top-right of 2d camera
        ]
       
        pixels = [camera.get_pixel_coords(x, y) for x, y, z in top_corners]


        horizontal_center_pixels = ((pixels[3][0] - pixels[1][0])/2) + pixels[1][0]
        vertical_center_pixels = ((pixels[2][1] - pixels[3][1])/2) + pixels[3][1]
        yolo_horizontal_center = round(horizontal_center_pixels / IMG_WIDTH, 6)
        yolo_vertical_center = round(vertical_center_pixels / IMG_HEIGHT, 6)

        pixel_width = pixels[3][0] - pixels[1][0]
        pixel_height = pixels[2][1] - pixels[3][1]
        yolo_width = round(pixel_width / IMG_WIDTH, 6)
        yolo_height = round(pixel_height / IMG_HEIGHT, 6)

        '''

        print("=== Bounding Box Calculation Debug ===")

        print(f"Min coords: {aabb_min}" + "\n" + f"Max coords: {aabb_max}")
        print("Bottom-left:", aabb_min[0], aabb_max[1])
        print("Top-left:", aabb_min[0], aabb_min[1])
        print("Bottom-right:", aabb_max[0], aabb_max[1])
        print("Top-right:", aabb_max[0], aabb_min[1])
        """
        print(f"pixels[0] (Bottom-left): {pixels[0]}")
        print(f"pixels[1] (Top-left): {pixels[1]}")
        print(f"pixels[3] (Top-right): {pixels[3]}")
        print(f"pixels[2] (Bottom-right): {pixels[2]}")
        """
        print(f"horizontal_center_pixels = (({pixels[3][0]} - {pixels[1][0]}) / 2) + {pixels[1][0]} = {horizontal_center_pixels}")
        print(f"vertical_center_pixels = (({pixels[2][1]} - {pixels[3][1]}) / 2) + {pixels[3][1]} = {vertical_center_pixels}")

        print(f"yolo_horizontal_center = round({horizontal_center_pixels} / {IMG_WIDTH}, 6) = {yolo_horizontal_center}")
        print(f"yolo_vertical_center = round({vertical_center_pixels} / {IMG_HEIGHT}, 6) = {yolo_vertical_center}")

        print(f"pixel_width = {pixels[3][0]} - {pixels[1][0]} = {pixel_width}")
        print(f"pixel_height = {pixels[2][1]} - {pixels[3][1]} = {pixel_height}")

        print(f"yolo_width = round({pixel_width} / {IMG_WIDTH}, 6) = {yolo_width}")
        print(f"yolo_height = round({pixel_height} / {IMG_HEIGHT}, 6) = {yolo_height}")
        print("======================================")
        '''

        label.extend(map(str, [class_id, yolo_horizontal_center, yolo_vertical_center, yolo_width, yolo_height]))


        label_name = base+f"{j}.txt"
        label_path = os.path.join(label_dir, label_name)
        with open(label_path, 'w') as f:
            f.write(" ".join(label) + "\n")

       
        
        p.removeBody(object_id)
        p.stepSimulation()
    class_id += 1

while True:
    # Get camera image and display it
    img = camera.get_image()
    cv2.imshow("Camera View", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)
    
    # Step simulation
    p.stepSimulation()
    time.sleep(1./240.)