import pybullet as p
import pybullet_data
import numpy as np
import cv2
import os
import random
from glob import glob
from ultralytics import YOLO

# Automatically find all URDFs in the ycb folder
urdf_dir = "../assets/urdf/ycb/"
urdf_list = glob(os.path.join(urdf_dir, "*.urdf"))
print(f"Found {len(urdf_list)} URDFs to test.")

# Output directory for annotated images
output_dir = "output/images"
os.makedirs(output_dir, exist_ok=True)

# Load your trained YOLO model
model = YOLO('../models/trash_detector/weights/new_best_model.pt')
print("Loaded YOLO model.")

# Camera class
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

# PyBullet setup
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf", basePosition=[0, 0, -1.0])
belt_length, belt_width, belt_height = 5, 3, 0.02
belt_pos = [-0.3, 0, belt_height / 2]
belt_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[belt_length/2, belt_width/2, belt_height/2])
belt_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[belt_length/2, belt_width/2, belt_height/2], rgbaColor=[0, 0, 0, 1])
p.createMultiBody(0, belt_col, belt_vis, belt_pos)

camera = TopDownCamera(img_width=512, img_height=512, camera_position=[0, 0, 3], floor_plane_size=1)

for urdf_path in urdf_list:
    print(f"Processing URDF: {urdf_path}")
    # Remove all bodies except plane and conveyor
    for i in reversed(range(p.getNumBodies())):
        if i >= 2:
            body_id = p.getBodyUniqueId(i)
            p.removeBody(body_id)
    # Load object at random position
    rand_pos = [random.uniform(-0.2, 0.2), random.uniform(-0.2, 0.2), 1]
    obj_id = p.loadURDF(urdf_path, basePosition=rand_pos, globalScaling=0.25)
    if obj_id < 0:
        print(f"Failed to load URDF: {urdf_path}")
        continue
    print(f"Loaded object ID: {obj_id}")
    for _ in range(120):
        p.stepSimulation()
    img = camera.get_image()
    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    results = model(img_bgr)
    print(f"YOLO inference done for {urdf_path}. Results: {results}")
    # Draw boxes and labels
    for result in results:
        boxes = result.boxes
        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
            conf = box.conf[0].cpu().numpy()
            cls_id = int(box.cls[0].cpu().numpy())
            label = model.names[cls_id] if hasattr(model, 'names') and cls_id < len(model.names) else str(cls_id)
            cv2.rectangle(img_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img_bgr, f'{label} {conf:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
    # Save result image
    urdf_name = os.path.splitext(os.path.basename(urdf_path))[0]
    out_path = os.path.join(output_dir, f"{urdf_name}_detected.jpg")
    cv2.imwrite(out_path, img_bgr)
    if os.path.exists(out_path):
        print(f"Saved detection result for {urdf_name} to {out_path}")
    else:
        print(f"Failed to save image for {urdf_name}")

p.disconnect()