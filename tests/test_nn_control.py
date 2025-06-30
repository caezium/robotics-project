import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time
from utils.camera import TopDownCamera
from utils.pybullet_helpers import move_arm_to, wait_for_arm_to_reach, grab_object, release_object

# TODO: Import your neural network inference function here
# from your_nn_module import run_inference

def run_nn_on_image(image):
    """
    Placeholder for neural network inference.
    Replace this with your actual NN call.
    """
    # TODO: Replace with actual NN inference
    # Example: return run_inference(image)
    # For now, just return a dummy target position
    return [0.0, 0.0, 0.2]

release_time = None
cooldown_duration = 2.0  # seconds cooldown after release before repicking allowed

# ----- PyBullet Setup -----
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

plane_id = p.loadURDF("plane.urdf", basePosition=[0, 0, -1.0])

belt_length, belt_width, belt_height = 5, 0.3, 0.02
belt_pos = [-0.3, 0, belt_height / 2]
belt_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[belt_length/2, belt_width/2, belt_height/2])
belt_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[belt_length/2, belt_width/2, belt_height/2], rgbaColor=[0, 0, 0, 1])
belt_id = p.createMultiBody(0, belt_col, belt_vis, belt_pos)

kuka_id = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0.4, 0], useFixedBase=True)
num_joints = p.getNumJoints(kuka_id)

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

camera = TopDownCamera(img_width=200, img_height=200, camera_position=[0, 0, 2], floor_plane_size=1)

picked = False
tracking = False
constraint_id = None
drop_position = [0.5, 0.5, 0.3]

while True:
    contacts = p.getContactPoints(bodyA=ball_id, bodyB=belt_id)
    if contacts and not picked:
        p.resetBaseVelocity(ball_id, linearVelocity=[belt_velocity, 0, 0])

    img = camera.get_image()
    # NN expects RGB or BGR, adjust as needed
    nn_input = img.copy()
    # Run neural network inference to get target position for the arm
    target_pos = run_nn_on_image(nn_input)

    # Optionally visualize
    cv2.imshow("Camera View", img)
    cv2.waitKey(1)

    current_time = time.time()
    if not picked and (release_time is None or current_time - release_time > cooldown_duration):
        # Move arm to the position predicted by the NN
        move_arm_to(kuka_id, num_joints, target_pos)
        wait_for_arm_to_reach(kuka_id, target_pos)

        # Check if close enough to grab
        ee_pos = p.getLinkState(kuka_id, 6)[0]
        dist = np.linalg.norm(np.array(ee_pos) - np.array(target_pos))
        if dist < 0.1:
            constraint_id = grab_object(kuka_id, ball_id)
            picked = True
            tracking = True
            print("Suction applied: NN-controlled arm caught object.")

    if picked and tracking:
        # Lift and drop as before
        ball_pos, _ = p.getBasePositionAndOrientation(ball_id)
        lift_pos = [ball_pos[0], ball_pos[1], 0.4]
        move_arm_to(kuka_id, num_joints, lift_pos)
        wait_for_arm_to_reach(kuka_id, lift_pos)
        for _ in range(240):
            p.stepSimulation()
            time.sleep(1./240.)
        p.changeDynamics(ball_id, -1, linearDamping=0, angularDamping=0, restitution=0)
        p.resetBaseVelocity(ball_id, [0, 0, 0], [0, 0, 0])
        move_arm_to(kuka_id, num_joints, drop_position)
        wait_for_arm_to_reach(kuka_id, drop_position)
        for _ in range(240):
            p.stepSimulation()
            time.sleep(1./240.)
        release_object(constraint_id)
        release_time = time.time()
        p.resetBaseVelocity(ball_id, [0, 0, 0], [0, 0, 0])
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