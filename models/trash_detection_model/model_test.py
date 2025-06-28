import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time
import os
from math import radians
from ultralytics import YOLO

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
p.loadURDF(r"src\urdf\ycb\012_strawberry.urdf", basePosition=[0,0,1], globalScaling=0.25)


# Conveyor belt
belt_length, belt_width, belt_height = 5, 1, 0.02
belt_pos = [-0.3, 0, belt_height / 2]
belt_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[belt_length/2, belt_width/2, belt_height/2])
belt_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[belt_length/2, belt_width/2, belt_height/2], rgbaColor=[0, 0, 0, 1])
belt_id = p.createMultiBody(0, belt_col, belt_vis, belt_pos)

for _ in range(120):
    p.stepSimulation()


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
camera = TopDownCamera(img_width=512, img_height=512, camera_position=[0, 0, 3], floor_plane_size=1)
CAMERA_POS = [0, 0, 3]  # position, topdown
CAMERA_TARGET = [0, 0, 0]  # lookat
CAMERA_UP = [0, 10, 0]
FOV = 60  # in degrees
NEAR, FAR = 0.1, 100.0  # Near and far clipping planes for the camera
IMG_WIDTH = 512
IMG_HEIGHT=512





# Load your trained YOLO model
model = YOLO(r'runs\detect\train4\weights\trash4n100.pt')  # replace with your actual model path



# Get image from your PyBullet camera
img = camera.get_image()  # This is RGB format (H, W, 3) numpy array

# YOLO expects BGR images or RGB? Ultraytics YOLO usually works with either, but safest to convert
img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

# Run inference
results = model(img_bgr)  # or model.predict(img_bgr) depending on your ultralytics version

# `results` contains detection info - boxes, classes, scores
# You can draw boxes using the results

# Example drawing boxes on the image:
for result in results:
    boxes = result.boxes  # boxes detected in this image
    for box in boxes:
        # Each box contains xyxy coordinates, confidence, class id
        x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
        conf = box.conf[0].cpu().numpy()
        cls_id = int(box.cls[0].cpu().numpy())
        label = model.names[cls_id]  # class name
        
        # Draw rectangle on the image
        cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Draw label + confidence
        cv2.putText(img_bgr, f'{label} {conf:.2f}', (x1, y1 - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        


# Show the image with bounding boxes
cv2.imshow("YOLO Detections", img_bgr)
cv2.waitKey(0)
cv2.destroyAllWindows()

        

       
        
"""

# ----- Simulation Loop -----
while True:
    # Get camera image and display it
    #img = camera.get_image()
    cv2.imshow("Camera View", cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)
    
    # Step simulation
    p.stepSimulation()
    #comment out time.sleep for actual generation
    #time.sleep(1./240.)
"""