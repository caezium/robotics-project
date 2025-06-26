"""
Merged Vision + PyBullet Robotics Demo
- Detects colored object in either a synthetic or real image
- Maps pixel center to world coordinates
- Visualizes in PyBullet simulation (GUI)
- Moves robot end effector above detected object
"""

# =====================
# Imports and Setup
# =====================
import cv2
import numpy as np
import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import time
import os

# =====================
# User Options
# =====================
# Choose which image to use: 'synthetic' or 'real'
IMAGE_MODE = 'synthetic'  # Change to 'real' to use sample_objects.jpg

# =====================
# Image Loading & Color Detection
# =====================
if IMAGE_MODE == 'synthetic':
    # Create a blank image and draw a red circle
    image = np.zeros((200, 200, 3), dtype=np.uint8)
    px, py = 150, 150  # Circle center
    cv2.circle(image, (px, py), 50, (0, 0, 255), -1)  # BGR red
    print("Generated synthetic red-circle image.")
    # HSV range for red
    lower_color = np.array([0, 100, 100])
    upper_color = np.array([10, 255, 255])
else:
    # Load real image (sample_objects.jpg)
    image_path = os.path.join(os.path.dirname(__file__), 'sample_objects.jpg')
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"Could not load image at {image_path}")
    print(f"Loaded real image: {image_path}, shape: {image.shape}")
    # HSV range for green (adjust as needed)
    lower_color = np.array([35, 80, 80])
    upper_color = np.array([90, 255, 255])

# Convert to HSV and threshold
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, lower_color, upper_color)
print("Applied HSV threshold. Nonzero mask pixels:", np.sum(mask > 0))

# Find contours
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
if len(contours) > 0:
    # Use largest contour
    cnt = max(contours, key=cv2.contourArea)
    M = cv2.moments(cnt)
    if M['m00'] != 0:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        print(f"Detected center at pixel coordinates: ({cx}, {cy})")
    else:
        cx, cy = None, None
        print("Contour with zero area.")
else:
    cx, cy = None, None
    print("No object detected.")

# Draw detection for debug
if cx is not None:
    display_image = image.copy()
    cv2.circle(display_image, (cx, cy), 5, (0, 255, 0), -1)  # green dot
    plt.imshow(cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB))
    plt.title('Detected Center')
    plt.axis('off')
    plt.show()
else:
    print("No center to display.")

# =====================
# Pixel to World Mapping
# =====================
if cx is not None:
    image_height, image_width = image.shape[:2]
    plane_size = 1.0  # meters (synthetic)
    if IMAGE_MODE == 'real':
        # For real image, assume it covers 1x1m for demo, or adjust as needed
        plane_size = 1.0
    world_x = (cx / image_width) * plane_size - plane_size/2
    world_y = (cy / image_height) * plane_size - plane_size/2
    print(f"Mapped to world coordinates: x = {world_x:.2f} m, y = {world_y:.2f} m (z=0 assumed)")
else:
    world_x, world_y = 0, 0  # fallback

# =====================
# PyBullet Simulation
# =====================
def run_pybullet_demo(world_x, world_y):
    print("\n=== Starting PyBullet Simulation ===")
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -10)
    # Load plane
    planeId = p.loadURDF("plane.urdf")
    print("Loaded PyBullet plane.")
    # Load KUKA robot
    kukaId = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0,0,0], useFixedBase=True)
    num_joints = p.getNumJoints(kukaId)
    print(f"Loaded KUKA with {num_joints} joints.")
    # Place a red sphere at detected world coordinates
    sphereRadius = 0.05
    colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
    visSphereId = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius, rgbaColor=[1, 0, 0, 1])
    sphereId = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=colSphereId,
                                 baseVisualShapeIndex=visSphereId,
                                 basePosition=[world_x, world_y, sphereRadius])
    print("Sphere placed at world coordinates:", (world_x, world_y))
    # Move KUKA end effector above the sphere
    end_effector_index = 6
    target_pos = [world_x, world_y, sphereRadius + 0.1]
    joint_positions = p.calculateInverseKinematics(kukaId, end_effector_index, target_pos)
    for j in range(num_joints):
        p.resetJointState(kukaId, j, joint_positions[j])
    print("Moved KUKA end-effector near the target position.")
    # Run simulation loop
    print("Simulation running. Close the window to exit.")
    try:
        while True:
            p.stepSimulation()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Simulation stopped by user.")
    p.disconnect()

# =====================
# Main
# =====================
if __name__ == "__main__":
    run_pybullet_demo(world_x, world_y)
