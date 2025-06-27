import pybullet as p
import time

p.connect(p.GUI)

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
