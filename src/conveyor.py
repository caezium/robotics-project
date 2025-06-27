import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.8)
plane_id = p.loadURDF("plane.urdf")

# black conveyor belt (as a box)
belt_length = 1.0  # meters
belt_width = 0.3   # meters
belt_height = 0.02 # meters
belt_pos = [0, 0, belt_height / 2]

belt_col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[belt_length/2, belt_width/2, belt_height/2])
belt_vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[belt_length/2, belt_width/2, belt_height/2], rgbaColor=[0, 0, 0, 1])
belt_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=belt_col_id, baseVisualShapeIndex=belt_vis_id, basePosition=belt_pos)

# create a red ball above the belt
ball_radius = 0.04
ball_start_pos = [0, 0, 0.2]
ball_col_id = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
ball_vis_id = p.createVisualShape(p.GEOM_SPHERE, radius=ball_radius, rgbaColor=[1, 0, 0, 1])
ball_id = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=ball_col_id, baseVisualShapeIndex=ball_vis_id, basePosition=ball_start_pos)

# Conveyor belt velocity (m/s)
belt_velocity = 0.3

# --- Simulation Loop ---
while True:
    # get contact points between ball and belt
    contacts = p.getContactPoints(bodyA=ball_id, bodyB=belt_id)
    if contacts:
        # apply velocity to ball to sim conveyor movement
        p.resetBaseVelocity(ball_id, linearVelocity=[belt_velocity, 0, 0])
    
    p.stepSimulation()
    time.sleep(1./240.)
