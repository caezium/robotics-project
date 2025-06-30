import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time
import sys
import os
import random
from dataclasses import dataclass, field
from enum import Enum, auto
from ultralytics import YOLO

# Path Setup
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, PROJECT_ROOT)

from src.utils.camera import TopDownCamera
from src.utils.pybullet_helpers import move_arm_to, wait_for_arm_to_reach, grab_object, release_object
from src.utils.debug_gui import DebugInterface


@dataclass
class SimConfig:
    """
    Configuration for the simulation environment and robot
    """
    belt_velocity: float = 4.0
    pickup_x_coord: float = 0.2
    detection_line_x: float = -1.0
    confidence_threshold: float = 0.5
    arm_lead_time: float = 0.7
    model_path: str = os.path.join(PROJECT_ROOT, 'models/trash_detector/weights/trash100n(best).pt')
    ycb_urdf_path: str = os.path.join(PROJECT_ROOT, 'assets', 'urdf', 'ycb')
    trash_bin_urdf_path: str = os.path.join(PROJECT_ROOT, "assets/urdf/trash_bin.urdf")
    drop_position: list = field(default_factory=lambda: [0.5, 0.5, 0.3])
    img_width: int = 512
    img_height: int = 512
    camera_position: list = field(default_factory=lambda: [0, 0, 3])
    floor_plane_size: float = 1.0

class ArmState(Enum):
    """
    States for Finite State Machine
    """
    IDLE = auto()
    WAIT_FOR_OBJECT = auto()
    PREPARE_PICK = auto()
    PICKING = auto()
    LIFTING = auto()
    RESETTING = auto()

class RobotController:
    """
    Manages the robot arm simulation, detection, and interaction.
    """
    def __init__(self, config: SimConfig):
        self.config = config
        self.target_info = None
        self.picked = False
        self.tracking = False
        self.constraint_id = None
        self.release_time = None
        self._setup_simulation()
        self.gui = DebugInterface(self.kuka_id, self.num_joints, self.config)
        self.state = ArmState.IDLE
        self.previous_state = self.state
        self.last_img = None
        self.last_results = None
        self.debug_text_id = None
        self.last_debug_info = ""

    def _setup_simulation(self):
        """init"""
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)

        
        pplane_visual = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[5, 5, 0.01],   
            rgbaColor=[255,255,255,1],
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
        # Load conveyor belt
        belt_length, belt_width, belt_height = 5, 1, 0.02
        belt_pos = [-0.3, 0, belt_height / 2]
        belt_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[belt_length / 2, belt_width / 2, belt_height / 2])
        belt_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[belt_length / 2, belt_width / 2, belt_height / 2], rgbaColor=[0, 0, 0, 1])
        self.belt_id = p.createMultiBody(0, belt_col, belt_vis, belt_pos)

        # Load trash bins
        self._set_trash_bins()

        # Load robot arm KUKA
        self.kuka_id = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0.6, 0], useFixedBase=True)
        self.num_joints = p.getNumJoints(self.kuka_id)

        # Load camera and model
        self.camera = TopDownCamera(self.config.img_width, self.config.img_height, self.config.camera_position, self.config.floor_plane_size)
        self.model = YOLO(self.config.model_path)
    
    def _set_trash_bins(self):
        """
        Loads and colors the trash bins.
        """

        bin_recycling = p.loadURDF(self.config.trash_bin_urdf_path, basePosition=[0.90, 1, 0.2], globalScaling=2.0, useFixedBase=True)
        bin_trash     = p.loadURDF(self.config.trash_bin_urdf_path, basePosition=[2.75, 0, 0.2], globalScaling=2.0, useFixedBase=True)

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

    def _load_random_object(self):
        """
        Loads a random YCB object (including variants) onto the conveyor belt
        """
        ycb_dir = self.config.ycb_urdf_path
        variant_dir = os.path.join(os.path.dirname(ycb_dir), 'ycb_variants')
        urdf_files = []
        # Collect URDFs from main ycb dir
        urdf_files += [os.path.join(ycb_dir, f) for f in os.listdir(ycb_dir) if f.endswith('.urdf')]
        # Collect URDFs from variants dir if it exists
        if os.path.exists(variant_dir):
            urdf_files += [os.path.join(variant_dir, f) for f in os.listdir(variant_dir) if f.endswith('.urdf')]
        if not urdf_files:
            raise RuntimeError("No YCB URDF files found in either main or variant directory!")
        random_urdf_file = random.choice(urdf_files)
        print(f"[SPAWN] Loading URDF: {random_urdf_file}")  # Print the URDF path being loaded
        object_start_pos = [-1.4, 0, 0.1]
        self.object_id = p.loadURDF(random_urdf_file, basePosition=object_start_pos, globalScaling=0.25)
        p.resetBaseVelocity(self.object_id, linearVelocity=[self.config.belt_velocity, 0, 0])
        pos, orn = p.getBasePositionAndOrientation(self.object_id)
        p.resetBasePositionAndOrientation(self.object_id, [pos[0], 0, pos[2]], orn)

    def run(self):
        """
        Main simulation loop
        """
        self._load_random_object()
        self.state = ArmState.WAIT_FOR_OBJECT
        frame_count = 0
        sim_time = 0.0
        while True:
            # Prepare info for debug overlay
            if self.last_results is not None and len(self.last_results) > 0 and hasattr(self.last_results[0], 'boxes') and self.last_results[0].boxes is not None:
                boxes = self.last_results[0].boxes.xyxy.cpu().numpy()
            else:
                boxes = []
            self.gui.update(self, fsm_state=self.state.name, sim_time=sim_time, target_info=self.target_info, boxes=boxes)
            contacts = p.getContactPoints(bodyA=self.object_id, bodyB=self.belt_id)
            if contacts and not self.picked:
                p.resetBaseVelocity(self.object_id, linearVelocity=[self.config.belt_velocity, 0, 0])
            
            self.last_img = self.camera.get_image()
            self.last_results = self.model(self.last_img, verbose=False)
            self._step_fsm(sim_time)

            if self.state != self.previous_state:
                print(f"FSM State Change: {self.previous_state.name} -> {self.state.name}")
                self.previous_state = self.state

            # Always show YOLO detection window, even if no detection
            output_img = self.last_img.copy() # Create a writable copy for drawing
            confs = self.last_results[0].boxes.conf.cpu().numpy() if len(self.last_results) > 0 and hasattr(self.last_results[0], 'boxes') else None
            if len(boxes) > 0 and confs is not None and len(confs) > 0:
                idx = np.argmax(confs)
                x1, y1, x2, y2 = boxes[idx]
                cv2.rectangle(output_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.imshow("YOLO Detection", output_img)
            cv2.waitKey(1)

            frame_count += 1
            if frame_count % 30 == 0:
                print(f"Frame: {frame_count}")

            p.stepSimulation()
            time.sleep(1. / 240.)
            sim_time += 1.0 / 240.0

    def _step_fsm(self, sim_time):
        if self.state == ArmState.IDLE:
            self._handle_idle()
        elif self.state == ArmState.WAIT_FOR_OBJECT:
            self._handle_wait_for_object(sim_time)
        elif self.state == ArmState.PREPARE_PICK:
            self._handle_prepare_pick(sim_time)
        elif self.state == ArmState.PICKING:
            self._handle_picking()
        elif self.state == ArmState.LIFTING:
            self._handle_lifting()
        elif self.state == ArmState.RESETTING:
            self._handle_resetting()

    def _handle_idle(self):
        # Could be used for future multi-arm or pause logic
        pass

    def _handle_wait_for_object(self, sim_time):
        boxes = self.last_results[0].boxes.xyxy.cpu().numpy() if len(self.last_results) > 0 and hasattr(self.last_results[0], 'boxes') else []
        confs = self.last_results[0].boxes.conf.cpu().numpy() if len(self.last_results) > 0 and hasattr(self.last_results[0], 'boxes') else []

        if self.target_info is None and len(boxes) > 0:
            idx = np.argmax(confs)
            if confs[idx] >= self.config.confidence_threshold:
                current_object_pos, _ = p.getBasePositionAndOrientation(self.object_id)
                if current_object_pos[0] > self.config.detection_line_x:
                    print(f"\n[DETECTION] Detected at X={current_object_pos[0]:.3f}, detection_line_x={self.config.detection_line_x}, pickup_x={self.config.pickup_x_coord}")
                    print(f"[DETECTION] Detection sim_time: {sim_time:.3f}")
                    self.target_info = {"initial_pos": current_object_pos, "detection_time": sim_time}
                    print(f"\nSUCCESS: Target acquired at position {current_object_pos}")
                    self.state = ArmState.PREPARE_PICK
                else:
                    print(f"Tracking: Object at X={current_object_pos[0]:.2f}, waiting to cross line at X={self.config.detection_line_x:.2f}", end='\r')

    def _handle_prepare_pick(self, sim_time):
        elapsed_time = sim_time - self.target_info["detection_time"]
        predicted_x = self.target_info["initial_pos"][0] + self.config.belt_velocity * elapsed_time
        time_to_pickup = (self.config.pickup_x_coord - predicted_x) / self.config.belt_velocity if self.config.belt_velocity > 0 else float('inf')

        print(f"\n[PREPARE_PICK] initial_x={self.target_info['initial_pos'][0]:.3f}, elapsed={elapsed_time:.3f}, predicted_x={predicted_x:.3f}, pickup_x={self.config.pickup_x_coord}, time_to_pickup={time_to_pickup:.3f}")

        if 0 < time_to_pickup <= self.config.arm_lead_time:
            print(f"\nINFO: Object is {time_to_pickup:.2f}s away. Initiating grab.")
            self.state = ArmState.PICKING

    def _handle_picking(self):
        pickup_pos = [self.config.pickup_x_coord, self.target_info["initial_pos"][1], self.target_info["initial_pos"][2]]
        above_pos = [pickup_pos[0], pickup_pos[1], pickup_pos[2] + 0.15]

        move_arm_to(self.kuka_id, self.num_joints, above_pos)
        wait_for_arm_to_reach(self.kuka_id, above_pos, threshold=0.05)
        move_arm_to(self.kuka_id, self.num_joints, pickup_pos)
        wait_for_arm_to_reach(self.kuka_id, pickup_pos, threshold=0.05)

        self.constraint_id = grab_object(self.kuka_id, self.object_id)
        self.picked = True
        self.tracking = True
        print("SUCCESS: Suction applied, caught object via prediction.")
        self.state = ArmState.LIFTING

    def _handle_lifting(self):
        print("Lifting object...")
        current_pos = p.getLinkState(self.kuka_id, self.num_joints - 1)[0]
        lift_pos = [current_pos[0], current_pos[1], 0.4]
        move_arm_to(self.kuka_id, self.num_joints, lift_pos)
        wait_for_arm_to_reach(self.kuka_id, lift_pos, threshold=0.05)
        
        print("Moving to drop location...")
        move_arm_to(self.kuka_id, self.num_joints, self.config.drop_position)
        wait_for_arm_to_reach(self.kuka_id, self.config.drop_position, threshold=0.05)

        print("Releasing object...")
        release_object(self.constraint_id)
        self.release_time = time.time()

        # Print out the detected class label for the object just picked up
        if self.last_results is not None and len(self.last_results) > 0 and hasattr(self.last_results[0], 'boxes'):
            confs = self.last_results[0].boxes.conf.cpu().numpy()
            if confs is not None and len(confs) > 0:
                idx = np.argmax(confs)
                class_idx = int(self.last_results[0].boxes.cls[idx].cpu().numpy())
                class_name = self.model.names[class_idx] if hasattr(self.model, 'names') and class_idx < len(self.model.names) else str(class_idx)
                print(f"[INFO] Model detected and picked up: {class_name} (class {class_idx})")

        self.picked = False
        self.tracking = False
        self.target_info = None
        self.constraint_id = None
        self.state = ArmState.RESETTING

    def _handle_resetting(self):
        print("Resetting arm and preparing for next object.")
        self._load_random_object()
        self.state = ArmState.WAIT_FOR_OBJECT


if __name__ == "__main__":
    config = SimConfig()
    controller = RobotController(config)
    controller.run()
