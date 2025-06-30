import pybullet as p
import pybullet_data
import numpy as np
import cv2
import matplotlib.pyplot as plt
import time

from utils.camera import TopDownCamera

# ---------------- Simulation Setup ----------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

plane_id = p.loadURDF("plane.urdf")
kuka_id = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0, 0], useFixedBase=True)
num_joints = p.getNumJoints(kuka_id)

# Load a red sphere as the object to pick
SPHERE_X, SPHERE_Y, sphereRadius = 0.3, 0.2, 0.05
col_id = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
vis_id = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius, rgbaColor=[1, 0, 0, 1])
sphere_id = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=col_id, baseVisualShapeIndex=vis_id, basePosition=[SPHERE_X, SPHERE_Y, sphereRadius])
print(f"Red sphere loaded at: ({SPHERE_X}, {SPHERE_Y})")

# ---------------- Camera Init ----------------
camera = TopDownCamera(img_width=200, img_height=200, camera_position=[0, 0, 2], floor_plane_size=1.0)
# Create camera object
camera = TopDownCamera(
    img_width=200, 
    img_height=200, 
    camera_position=[0, 0, 2], 
    floor_plane_size=1.0
)

# Get image from camera
img = camera.get_image()

# Convert RGB to HSV
hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

# Red color range in HSV
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([179, 255, 255])

mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
mask = cv2.bitwise_or(mask1, mask2)

lower_blue = np.array([105, 150, 150])  # Hue, Saturation, Value
upper_blue = np.array([125, 255, 255])
'''
mask = cv2.inRange(hsv, lower_blue, upper_blue)
'''


print("Applied HSV threshold for red. Nonzero mask pixels:", np.sum(mask > 0))
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
if len(contours) > 0:
    cnt = max(contours, key=cv2.contourArea)
    M = cv2.moments(cnt)
    if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
    else:
        cx, cy = None, None
    print(f"Detected center at pixel coordinates: ({cx}, {cy})")
else:
    cx, cy = None, None
    print("No red object detected.")

# Draw the detected center on the mask image
output_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
if cx is not None:
    cv2.circle(output_img, (cx, cy), 2, (0, 255, 0), -1)

plt.imshow(output_img)
plt.xlabel('Pixel X')
plt.ylabel('Pixel Y')
plt.title('Red HSV Mask with Center')
plt.grid(True)
plt.show(block=False)
plt.pause(4)
cx, cy = None, None
if contours:
    cnt = max(contours, key=cv2.contourArea)
    M = cv2.moments(cnt)
    if M['m00'] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        print(f"Detected red object at pixel: ({cx}, {cy})")
else:
    print("No red object detected.")

# ---------------- Move Robot Arm ----------------
if cx is not None and cy is not None:
    world_x, world_y, world_z = camera.get_pixel_world_coords(cx, cy)
    above_object = [world_x, world_y, 0.3]
    on_object = [world_x, world_y, sphereRadius + 0.1]
    drop_pos = [0.5, 0.5, 0.3]

    # Move above object
    joint_positions = p.calculateInverseKinematics(kuka_id, 6, above_object)
    for j in range(num_joints):
        p.setJointMotorControl2(kuka_id, j, p.POSITION_CONTROL, joint_positions[j], force=500)
    for _ in range(240):
        p.stepSimulation()
        time.sleep(1./240.)

    # Move down to object
    joint_positions = p.calculateInverseKinematics(kuka_id, 6, on_object)
    for j in range(num_joints):
        p.setJointMotorControl2(kuka_id, j, p.POSITION_CONTROL, joint_positions[j], force=500)
    for _ in range(240):
        p.stepSimulation()
        time.sleep(1./240.)

    # Simulate suction (attach sphere to gripper)
    constraint_id = p.createConstraint(
        parentBodyUniqueId=kuka_id,
        parentLinkIndex=6,
        childBodyUniqueId=sphere_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, 0]
    )
    print("Suction grip applied!")

    # Lift object
    joint_positions = p.calculateInverseKinematics(kuka_id, 6, above_object)
    for j in range(num_joints):
        p.setJointMotorControl2(kuka_id, j, p.POSITION_CONTROL, joint_positions[j], force=500)
    for _ in range(240):
        p.stepSimulation()
        time.sleep(1./240.)

    # Move to drop position
    joint_positions = p.calculateInverseKinematics(kuka_id, 6, drop_pos)
    for j in range(num_joints):
        p.setJointMotorControl2(kuka_id, j, p.POSITION_CONTROL, joint_positions[j], force=500)
    for _ in range(480):
        p.stepSimulation()
        time.sleep(1./240.)

    # Release object
    p.removeConstraint(constraint_id)
    print("Object released.")

# ---------------- Run Simulation ----------------
while True:
    p.stepSimulation()
    time.sleep(1./240.)