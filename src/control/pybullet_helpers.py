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


def wait_for_arm_to_reach(kuka_id, target_pos, threshold=0.1, max_steps=240, debug=False):
    """
    Wait until the end effector is close to the target position.
    
    Args:
        kuka_id: Robot body ID
        target_pos: Target position [x, y, z]
        threshold: Distance threshold for considering target reached
        max_steps: Maximum simulation steps to wait
        debug: If True, print debug information about arm movement
    """
    for step in range(max_steps):
        ee_pos = p.getLinkState(kuka_id, 6)[0]
        dist = np.linalg.norm(np.array(ee_pos) - np.array(target_pos))
        
        if debug and step % 30 == 0:  # Print debug info every 30 steps
            print(f"[ARM DEBUG] Step {step}: EE pos {ee_pos}, target {target_pos}, distance {dist:.4f}, threshold {threshold}")
        
        if dist < threshold:
            if debug:
                print(f"[ARM DEBUG] Target reached in {step} steps, final distance {dist:.4f}")
            return True
        p.stepSimulation()
        # Use a consistent timestep (this should match the main simulation timestep)
        time.sleep(1./240.)
    
    if debug:
        print(f"[ARM DEBUG] Failed to reach target after {max_steps} steps, final distance {dist:.4f}")
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