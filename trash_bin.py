import pybullet as p
import time

p.connect(p.GUI)

bin_recycling = p.loadURDF("trash_bin.urdf", basePosition=[0, 0, 0],)
bin_trash = p.loadURDF("trash_bin.urdf", basePosition=[1, 0, 0])



def set_color(body_id, color):
    visual_shapes = p.getVisualShapeData(body_id)
    for shape in visual_shapes:
        link_index = shape[1]
        p.changeVisualShape(body_id, link_index, rgbaColor=color)

blue = [0, 0, 1, 1]
black = [0, 0, 0, 1]
set_color(bin_recycling, blue)
set_color(bin_trash, black)

while True:
    p.stepSimulation()
    time.sleep(1./240.)
