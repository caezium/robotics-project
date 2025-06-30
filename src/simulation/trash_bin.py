import pybullet as p
import pybullet as p
import pybullet_data 
import time

p.connect(p.GUI)
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


pplane_visual = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[5, 5, 0.01],   
    rgbaColor=[0.8, 0.8, 0.8, 1.0],
)
plane_collision = p.createCollisionShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[5, 5, 0.01]
)
plane_id = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=plane_collision,
    baseVisualShapeIndex=pplane_visual,
    basePosition=[0, 0, -0.01]
)
bin_recycling = p.loadURDF("trash_bin.urdf", basePosition=[0, 0, 0])
bin_trash = p.loadURDF("trash_bin.urdf", basePosition=[1, 0, 0])
def transparent_box(body_id, body_color, edge_color):
    visual_shapes = p.getVisualShapeData(body_id)
    for shape in visual_shapes:
        link_index = shape[1]
        shape_name = shape[4].decode("utf-8") if isinstance(shape[4], bytes) else shape[4]
        if "edge" in shape_name.lower() or "rim" in shape_name.lower() or link_index == 1:
            p.changeVisualShape(body_id, link_index, rgbaColor=edge_color)
        else:
            p.changeVisualShape(body_id, link_index, rgbaColor=body_color)
blue_body = [0, 0, 1, 0.5]
blue_edge = [0, 0, 1, 1.0]

black_body = [0.1, 0.1, 0.1, 0.5]
black_edge = [0, 0, 0, 1.0]

transparent_box(bin_recycling, blue_body, blue_edge)
transparent_box(bin_trash, black_body, black_edge)
while True:
    p.stepSimulation()
    time.sleep(1./240.)
