import pybullet as p
import numpy as np
import time

def move_arm_to(kuka_id, num_joints, target_pos, force=5000, max_velocity=25):
    """
    Move the KUKA arm to the target position using inverse kinematics.
    """
    joint_positions = p.calculateInverseKinematics(kuka_id, 6, target_pos)
    for j in range(num_joints):
        p.setJointMotorControl2(kuka_id, j, p.POSITION_CONTROL, joint_positions[j], force=force, maxVelocity=max_velocity)


def wait_for_arm_to_reach(kuka_id, target_pos, threshold=0.1, max_steps=240):
    """
    Wait until the end effector is close to the target position.
    """
    for _ in range(max_steps):
        ee_pos = p.getLinkState(kuka_id, 6)[0]
        dist = np.linalg.norm(np.array(ee_pos) - np.array(target_pos))
        if dist < threshold:
            return True
        p.stepSimulation()
        time.sleep(1./240.)
    return False


def grab_object(kuka_id, ball_id):
    """
    Create a fixed constraint to grab the object with the end effector.
    """
    constraint_id = p.createConstraint(
        kuka_id, 6, ball_id, -1, p.JOINT_FIXED,
        [0, 0, 0], [0, 0, 0], [0, 0, 0]
    )
    return constraint_id


def release_object(constraint_id):
    """
    Remove the constraint to release the object.
    """
    p.removeConstraint(constraint_id) 