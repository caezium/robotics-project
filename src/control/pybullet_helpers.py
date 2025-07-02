import pybullet as p
import numpy as np
import time

def move_arm_to(kuka_id, num_joints, target_pos, force=10000, max_velocity=50):
    """
    Move the KUKA arm to the target position using inverse kinematics.
    """
    joint_positions = p.calculateInverseKinematics(kuka_id, 6, target_pos)
    for j in range(num_joints):
        p.setJointMotorControl2(kuka_id, j, p.POSITION_CONTROL, joint_positions[j], force=force, maxVelocity=max_velocity)


def wait_for_arm_to_reach(kuka_id, target_pos, threshold=0.1):
    """
    Stateless check: return True if the end effector is close to the target position, False otherwise.
    """
    ee_pos = p.getLinkState(kuka_id, 6)[0]
    dist = np.linalg.norm(np.array(ee_pos) - np.array(target_pos))
    return dist < threshold


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