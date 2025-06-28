import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time
# if not already imported
from utils.camera import TopDownCamera
from utils.pybullet_helpers import move_arm_to, wait_for_arm_to_reach, grab_object, release_object

release_time = None
cooldown_duration = 2.0  # seconds cooldown after release before repicking allowed

# ----- PyBullet Setup -----
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

plane_id = p.loadURDF("plane.urdf", basePosition=[0, 0, -1.0])

# b
belt_length, belt_width, belt_height = 5, 0.3, 0.02
belt_pos = [-0.3, 0, belt_height / 2]
belt_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[belt_length/2, belt_width/2, belt_height/2])
belt_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[belt_length/2, belt_width/2, belt_height/2], rgbaColor=[0, 0, 0, 1])
belt_id = p.createMultiBody(0, belt_col, belt_vis, belt_pos)

kuka_id = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0.4, 0], useFixedBase=True)
num_joints = p.getNumJoints(kuka_id)

# red bal
ball_radius = 0.04
ball_start_pos = [-1.4, 0, 0.1]
ball_col = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
ball_vis = p.createVisualShape(p.GEOM_SPHERE, radius=ball_radius, rgbaColor=[1, 0, 0, 1])
ball_id = p.createMultiBody(0.1, ball_col, ball_vis, ball_start_pos)

belt_velocity = 3
p.resetBaseVelocity(ball_id, linearVelocity=[belt_velocity, 0, 0])
pos, orn = p.getBasePositionAndOrientation(ball_id)
corrected_pos = [pos[0], 0, pos[2]]
p.resetBasePositionAndOrientation(ball_id, corrected_pos, orn)

#---cam---
camera = TopDownCamera(img_width=200, img_height=200, camera_position=[0, 0, 2], floor_plane_size=1)

picked = False
tracking = False
constraint_id = None
drop_position = [0.5, 0.5, 0.3]

# Red HSV ranges for detection
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([179, 255, 255])

# ----- Simulation Loop -----
while True:
    contacts = p.getContactPoints(bodyA=ball_id, bodyB=belt_id)
    if contacts and not picked:
        p.resetBaseVelocity(ball_id, linearVelocity=[belt_velocity, 0, 0])

    img = camera.get_image()
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    output_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    if 'cx2' in locals():
        cv2.circle(output_img, (cx2, cy2), 5, (0, 255, 0), -1)
    cv2.imshow("HSV Detection", output_img)
# Show HSV mask from beginning
    output_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    if contours:
        cnt = max(contours, key=cv2.contourArea)
        M = cv2.moments(cnt)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(output_img, (cx, cy), 5, (0, 255, 0), -1)

    cv2.imshow("HSV Detection", output_img)
    cv2.waitKey(1)

    current_time = time.time()
    if contours and not picked and (release_time is None or current_time - release_time > cooldown_duration):
        grabbed = False
        ball_pos, _ = p.getBasePositionAndOrientation(ball_id)
        safe_height = ball_pos[2] + 0.01  # 1 cm above ball to avoid collision yet close enough

        target_offset = 0.0

        while not grabbed:
            img = camera.get_image()
            hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
            contours2, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours2:
                cnt2 = max(contours2, key=cv2.contourArea)
                M2 = cv2.moments(cnt2)
                if M2['m00'] != 0:
                    cx2 = int(M2['m10'] / M2['m00'])
                    cy2 = int(M2['m01'] / M2['m00'])

                    _, world_y2, _ = camera.get_pixel_world_coords(cx2, cy2)
                    ball_pos, _ = p.getBasePositionAndOrientation(ball_id)
                    safe_height = ball_pos[2] + 0.01  # 1 cm above ball to avoid collision
                    #track_target = [ball_pos[0], world_y2, safe_height]
                    track_target = [ball_pos[0], ball_pos[1], ball_pos[2] + safe_height]

                    # IK + move arm
                    move_arm_to(kuka_id, num_joints, track_target)
                    # Wait for arm to reach
                    reached = wait_for_arm_to_reach(kuka_id, track_target)

                    ee_pos = p.getLinkState(kuka_id, 6)[0]
                    dist = np.linalg.norm(np.array(ee_pos) - np.array(track_target))

                    if dist < 0.1:  # When close enough, grab
                        constraint_id = grab_object(kuka_id, ball_id)
                        picked = True
                        tracking = True
                        grabbed = True
                        print("Suction applied: caught moving red object.")

            p.stepSimulation()
            time.sleep(1./240.)

    if picked and tracking:
        ball_pos, _ = p.getBasePositionAndOrientation(ball_id)
        lift_pos = [ball_pos[0], ball_pos[1], 0.4]
        move_arm_to(kuka_id, num_joints, lift_pos)
        wait_for_arm_to_reach(kuka_id, lift_pos)

        for _ in range(240):
            p.stepSimulation()
            time.sleep(1./240.)

        # Prepare ball for clean release
        p.changeDynamics(ball_id, -1, linearDamping=0, angularDamping=0, restitution=0)
        p.resetBaseVelocity(ball_id, [0, 0, 0], [0, 0, 0])

        # Move to drop position
        move_arm_to(kuka_id, num_joints, drop_position)
        wait_for_arm_to_reach(kuka_id, drop_position)

        for _ in range(240):
            p.stepSimulation()
            time.sleep(1./240.)

        # Release and ensure no constraints remain
        release_object(constraint_id)
        release_time = time.time()

        p.resetBaseVelocity(ball_id, [0, 0, 0], [0, 0, 0])

        # Let it fall and print speed
        for _ in range(240):
            vel, _ = p.getBaseVelocity(ball_id)
            print("Current fall speed:", vel[2])
            p.stepSimulation()
            time.sleep(1./240.)

        print("Released object.")
        picked = False
        tracking = False

    p.stepSimulation()
    time.sleep(1./240.)
