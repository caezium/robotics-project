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
from math import radians

# Path Setup
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, PROJECT_ROOT)

from src.utils.camera import TopDownCamera
from src.control.pybullet_helpers import move_arm_to, wait_for_arm_to_reach, grab_object, release_object
from src.utils.debug_gui import DebugInterface
from src.utils.logger import get_logger

logger = get_logger(__name__)



@dataclass
class SimConfig:
    """
    Configuration for the simulation environment and robot
    """
    # Physics settings
    gravity: float = -4.9  # Reduced gravity to prevent excessive bouncing
    simulation_fps: float = 240.0  # Simulation frequency, Hz
    
    # Conveyor belt settings
    belt_velocity: float = 4.0
    belt_length: float = 4.0
    belt_width: float = 1.0
    belt_height: float = 0.02
    belt_position: list = field(default_factory=lambda: [-0.3, 0, 0.01])
    
    # Object spawning settings
    spawn_x_position: float = -1.0
    spawn_z_height: float = 0.15  # set height above belt to prevent clipping
    object_scale: float = 0.25  # Global scaling for objects
    enable_variants: bool = False  # Enable/disable spawning variant objects
    
    # Object physics settings
    object_lateral_friction: float = 0.8
    object_restitution: float = 0.1
    object_linear_damping: float = 0.5
    object_angular_damping: float = 0.5
    
    # Object settling settings
    max_settle_steps: int = 240  # Maximum steps to wait for settling
    movement_threshold: float = 0.001  # Movement threshold for settling
    stable_frames_required: int = 20  # Consecutive stable frames needed
    fall_through_threshold: float = -0.5  # Z position threshold for fall-through detection
    
    # Object removal boundaries
    conveyor_end_x: float = 1.5  # X position where objects are removed
    conveyor_start_x: float = -3.0  # X position where objects are removed
    min_z_position: float = -1.0  # Minimum Z position before removal
    yolo_zone_exit_x: float = 1.5  # X position where processed objects are removed
    
    # YOLO detection settings
    yolo_trigger_margin: float = 0.3  # YOLO activation zone
    camera_center_x: float = 0.0  # center X position
    
    # Robot arm settings
    pickup_x_coord: float = 0.2
    detection_line_x: float = -1.0
    confidence_threshold: float = 0.5
    arm_lead_time: float = 0.65
    arm_above_offset: float = 0.12  # Height above pickup position, should use depth camera later
    arm_lift_height: float = 0.6  # Height to lift object
    arm_threshold: float = 0.14  # Position threshold for arm movement
    arm_reset_threshold: float = 0.6  # Threshold for arm reset/base movement
    arm_base_position: list = field(default_factory=lambda: [0.9, 0.7, 0.3])  # Base position for arm reset
    
    # Trash bin settings
    recycling_bin_position: list = field(default_factory=lambda: [0.90, 1, -0.25])
    trash_bin_position: list = field(default_factory=lambda: [2.5, 0, -0.25])
    bin_scale: float = 2.0
    
    # Floor and environment settings
    floor_size: float = 5.0
    floor_height: float = 0.01
    floor_position: list = field(default_factory=lambda: [0, 0, -0.5])
    robot_arm_position: list = field(default_factory=lambda: [0, 0.6, 0])
    
    # Paths and model settings
    model_path: str = os.path.join(PROJECT_ROOT, 'models/trash_detector/weights/new_best_model.pt')
    ycb_urdf_path: str = os.path.join(PROJECT_ROOT, 'assets', 'urdf', 'ycb')
    trash_bin_urdf_path: str = os.path.join(PROJECT_ROOT, "assets/urdf/trash_bin.urdf")
    drop_position: list = field(default_factory=lambda: [0.9, 0.7, 0.3])
    
    # Camera settings
    img_width: int = 512
    img_height: int = 512
    camera_position: list = field(default_factory=lambda: [0, 0, 3])
    floor_plane_size: float = 1.0
    
    # Spawning settings
    spawn_random_y_low: float = -0.3
    spawn_random_y_high: float = 0.3
    
    # Object lists
    pitch_adjust_list: list = field(default_factory=lambda: [
        "002_master_chef_can.urdf", "003_cracker_box.urdf", "004_sugar_box.urdf", "005_tomato_soup_can.urdf", "006_mustard_bottle.urdf", "007_tuna_fish_can.urdf", "010_potted_meat_can.urdf", "021_bleach_cleanser.urdf", "022_windex_bottle.urdf", "065-a_cups.urdf", "065-b_cups.urdf", "065-c_cups.urdf", "065-d_cups.urdf", "065-e_cups.urdf", "065-f_cups.urdf", "065-g_cups.urdf", "065-h_cups.urdf", "065-i_cups.urdf", "065-j_cups.urdf"
    ])
    recycling_classes: list = field(default_factory=lambda: [
        "Sugar Box", "Tomato Soup Can", "Mustard Bottle", "Pudding Box", "Potted Meat Can", "Banana", "Apple", "Lemon", "Pear", "Orange", "Windex Bottle", "Bowl", "Sponge", "Plate", "Large Marker", "A Cups"
    ])
    
    # Visualization settings
    enable_shadows: bool = False  # Disable shadows for better performance
    enable_gui: bool = True  # Keep GUI enabled for controls

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
    def __init__(self, config: SimConfig, headless: bool = False):
        self.config = config
        self.target_info = None
        self.picked = False
        self.tracking = False
        self.constraint_id = None
        self.release_time = None
        self.object_processed = False  # Track if object has been processed by YOLO
        self.arm_processing_recyclable = False  # Track if arm is actively processing a recyclable object
        self.arm_substate = None  # Track substeps for non-blocking arm movement
        self._setup_simulation(headless=headless)
        self.gui = DebugInterface(self.kuka_id, self.num_joints, self.config)
        self.state = ArmState.IDLE
        self.previous_state = self.state
        self.last_img = None
        self.last_results = None
        self.debug_text_id = None
        self.last_debug_info = ""

    def _setup_simulation(self, headless: bool = False):
        """init"""
        if headless:
            p.connect(p.DIRECT)
        else:
            p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, self.config.gravity)
        
        # Configure visualization settings based on config
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1 if self.config.enable_shadows else 0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1 if self.config.enable_gui else 0)
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        
        # Set background color to white for cleaner look
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.setRealTimeSimulation(0)

        # Load custom floor plane
        pplane_visual = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[self.config.floor_size, self.config.floor_size, self.config.floor_height],
            rgbaColor=[1, 1, 1, 1],  # White in normalized format (0-1 range)
            specularColor=[0, 0, 0]  # No shininess
        )
        plane_collision = p.createCollisionShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[self.config.floor_size, self.config.floor_size, self.config.floor_height]
        )
        plane_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=plane_collision,
            baseVisualShapeIndex=pplane_visual,
            basePosition=self.config.floor_position
        )

        
        # Load conveyor belt
        belt_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[self.config.belt_length / 2, self.config.belt_width / 2, self.config.belt_height / 2])
        belt_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[self.config.belt_length / 2, self.config.belt_width / 2, self.config.belt_height / 2], rgbaColor=[0, 0, 0, 1])
        self.belt_id = p.createMultiBody(0, belt_col, belt_vis, self.config.belt_position)

        # Load trash bins
        self._set_trash_bins()

        # Load robot arm KUKA
        self.kuka_id = p.loadURDF("kuka_iiwa/model.urdf", basePosition=self.config.robot_arm_position, useFixedBase=True)
        self.num_joints = p.getNumJoints(self.kuka_id)
        for link_index in range(-1, self.num_joints):  
            color = [1, 1, 1, 1] if link_index % 2 == 0 else [0, 0, 0, 1]  # black and white colors
            p.changeVisualShape(self.kuka_id, link_index, rgbaColor=color)


        # Load camera and model
        self.camera = TopDownCamera(self.config.img_width, self.config.img_height, self.config.camera_position, self.config.floor_plane_size)
        self.model = YOLO(self.config.model_path)
    
    def _set_trash_bins(self):
        """
        Loads and colors the trash bins.
        """

        bin_recycling = p.loadURDF(self.config.trash_bin_urdf_path, basePosition=self.config.recycling_bin_position, globalScaling=self.config.bin_scale, useFixedBase=True)
        bin_trash     = p.loadURDF(self.config.trash_bin_urdf_path, basePosition=self.config.trash_bin_position, globalScaling=self.config.bin_scale, useFixedBase=True)

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
        Loads a random YCB object (including variants if enabled) onto the conveyor belt with randomized position and orientation.
        Spawns above the belt and waits for contact before applying conveyor velocity.
        """
        ycb_dir = self.config.ycb_urdf_path
        variant_dir = os.path.join(os.path.dirname(ycb_dir), 'ycb_variants')
        urdf_files = []
        # Collect URDFs from main ycb dir
        urdf_files += [os.path.join(ycb_dir, f) for f in os.listdir(ycb_dir) if f.endswith('.urdf')]
        # Collect URDFs from variants dir if it exists
        if os.path.exists(variant_dir) and self.config.enable_variants:
            urdf_files += [os.path.join(variant_dir, f) for f in os.listdir(variant_dir) if f.endswith('.urdf')]
        if not urdf_files:
            raise RuntimeError("No YCB URDF files found in either main or variant directory!")
        random_urdf_file = random.choice(urdf_files)
        logger.info(f"[SPAWN] Loading URDF: {random_urdf_file}")

        # Randomized position
        object_start_pos = [self.config.spawn_x_position, random.uniform(self.config.spawn_random_y_low, self.config.spawn_random_y_high), self.config.spawn_z_height]

        # Randomize yaw (rotation around z)
        random_yaw = random.uniform(0, 360)
        urdf_basename = os.path.basename(random_urdf_file)
        if urdf_basename in self.config.pitch_adjust_list: # list for objects that should be in this pitch
            rotation = [0, -90, random_yaw]
        else:
            rotation = [0, 0, random_yaw]
        quaternion = p.getQuaternionFromEuler([radians(x) for x in rotation])
        
        # Load object with zero initial velocity (let it fall naturally)
        self.object_id = p.loadURDF(random_urdf_file, basePosition=object_start_pos, baseOrientation=quaternion, globalScaling=self.config.object_scale)
        p.resetBaseVelocity(self.object_id, linearVelocity=[0, 0, 0])  # Start with no velocity
        
        # Add damping to prevent bouncing
        p.changeDynamics(self.object_id, -1, lateralFriction=self.config.object_lateral_friction, restitution=self.config.object_restitution, linearDamping=self.config.object_linear_damping, angularDamping=self.config.object_angular_damping)
        
        # Wait for object to settle on the belt before applying conveyor velocity
        self._wait_for_belt_contact()

    def _wait_for_belt_contact(self):
        """
        Waits for the object to make contact with the conveyor belt and settle before applying velocity.
        """
        logger.info("[SPAWN] Waiting for object to settle on belt")
        contact_made = False
        settled = False
        last_pos = None
        stable_count = 0
        
        for step in range(self.config.max_settle_steps):
            p.stepSimulation()
            
            # Check if object is in contact with the belt
            contacts = p.getContactPoints(bodyA=self.object_id, bodyB=self.belt_id)
            if contacts:
                contact_made = True
                
                # Check if object has settled (stopped moving significantly)
                pos, _ = p.getBasePositionAndOrientation(self.object_id)
                if last_pos is not None:
                    movement = abs(pos[2] - last_pos[2])  # Check vertical movement
                    if movement < self.config.movement_threshold:
                        stable_count += 1
                        if stable_count >= self.config.stable_frames_required:
                            settled = True
                            logger.info(f"[SPAWN] Object settled on belt after {step} steps")
                            break
                    else:
                        stable_count = 0  # Reset if object is still moving
                last_pos = pos
            
            # Also check if object has fallen too far (error case just found)
            pos, _ = p.getBasePositionAndOrientation(self.object_id)
            if pos[2] < self.config.fall_through_threshold:  # Object fell through the belt
                logger.warning("[SPAWN] Object fell through belt, respawning...")
                p.removeBody(self.object_id)
                self._load_random_object()  # Recursive call to respawn
                return
        
        if not contact_made:
            logger.warning("[SPAWN] Object did not make contact with belt within timeout")
        elif not settled:
            logger.warning("[SPAWN] Object did not settle within timeout, applying velocity anyway")
        
        # Apply conveyor velocity once settled
        p.resetBaseVelocity(self.object_id, linearVelocity=[self.config.belt_velocity, 0, 0])
        logger.info(f"[SPAWN] Applied conveyor velocity: {self.config.belt_velocity} m/s")

    def run(self):
        """
        Main simulation loop
        """
        self._load_random_object()
        self.state = ArmState.WAIT_FOR_OBJECT
        frame_count = 0
        sim_time = 0.0
        
        # first get, needs this
        self.last_img = self.camera.get_image()
        self.last_results = []
        while True:
            # Get object position
            obj_pos, _ = p.getBasePositionAndOrientation(self.object_id)

            # Remove object if it is off the conveyor or disposed
            if self.object_id is not None:
                try:
                    obj_pos, _ = p.getBasePositionAndOrientation(self.object_id)
                except Exception:
                    self.object_id = None
                    obj_pos = [0,0,0]
                else:
                    # Remove if: off conveyor bounds, fallen too low, or processed and past YOLO zone
                    past_yolo_zone = obj_pos[0] > self.config.yolo_zone_exit_x and self.object_processed  # Past YOLO zone and processed
                    if obj_pos[0] > self.config.conveyor_end_x or obj_pos[0] < self.config.conveyor_start_x or obj_pos[2] < self.config.min_z_position or past_yolo_zone:
                        if past_yolo_zone:
                            logger.info(f"[REMOVE] Object processed and past YOLO zone at X={obj_pos[0]:.2f}")
                        else:
                            logger.info(f"[REMOVE] Object removed at X={obj_pos[0]:.2f}")
                        p.removeBody(self.object_id)
                        self.object_id = None
                        obj_pos = [0,0,0]
                        self.object_processed = False  # Reset for next object
            # Only spawn a new object if there is none AND arm is not processing a recyclable object
            if self.object_id is None and not self.arm_processing_recyclable:
                self._load_random_object()
                self.object_processed = False  # Reset for new object
                obj_pos, _ = p.getBasePositionAndOrientation(self.object_id)
            elif self.object_id is None and self.arm_processing_recyclable:
                logger.info("[SPAWN] Blocked - arm is processing recyclable object")
                obj_pos = [0, 0, 0]  # Set default position when no object exists

            # Check if object is in YOLO detection zone, can't keep model always running as it will mess up loop times
            in_yolo_zone = (self.config.camera_center_x - self.config.yolo_trigger_margin <= obj_pos[0] < self.config.camera_center_x)
            
            # Track when object passes through YOLO zone
            if in_yolo_zone and not self.object_processed:
                self.object_processed = True  # Mark as processed when entering YOLO zone, lol
                logger.info(f"[YOLO] Object entered detection zone at X={obj_pos[0]:.2f}")
            
            inf_ms = 0
            if in_yolo_zone:
                import time as _time
                t0 = _time.time()
                logger.info(f"[YOLO] Running inference at frame {frame_count}")
                
                # Add error handling for camera image capture
                try:
                    self.last_img = self.camera.get_image()
                    # Convert RGB to BGR for YOLO, YOLO was trained on BGR
                    img_bgr = cv2.cvtColor(self.last_img, cv2.COLOR_RGB2BGR)
                    self.last_results = self.model(img_bgr, verbose=False)
                except Exception as e:
                    logger.warning(f"[YOLO] Error during inference: {e}")
                    self.last_img = np.zeros((self.config.img_height, self.config.img_width, 3), dtype=np.uint8)
                    self.last_results = []
                
                t1 = _time.time()
                inf_ms = (t1 - t0) * 1000
                logger.info(f"[YOLO] Inference completed in {inf_ms:.1f}ms")
            else:
                overlay_text = "YOLO: not run"
            # Prepare info for debug overlay
            if self.last_results is not None and len(self.last_results) > 0 and hasattr(self.last_results[0], 'boxes') and self.last_results[0].boxes is not None:
                boxes = self.last_results[0].boxes.xyxy.cpu().numpy()
            else:
                boxes = []
            self.gui.update(self, fsm_state=self.state.name, sim_time=sim_time, target_info=self.target_info, boxes=boxes)
            contacts = p.getContactPoints(bodyA=self.object_id, bodyB=self.belt_id)
            if contacts and not self.picked:
                p.resetBaseVelocity(self.object_id, linearVelocity=[self.config.belt_velocity, 0, 0])
            self._step_fsm(sim_time)
            if self.state != self.previous_state:
                logger.info(f"[FSM] State change: {self.previous_state.name} -> {self.state.name}")
                self.previous_state = self.state
            # Always show YOLO detection window, even if no detection
            try:
                output_img = self.last_img.copy() if self.last_img is not None else np.zeros((self.config.img_height, self.config.img_width, 3), dtype=np.uint8)
            except Exception as e:
                logger.warning(f"[DISPLAY] Error copying image: {e}")
                output_img = np.zeros((self.config.img_height, self.config.img_width, 3), dtype=np.uint8)
            
            # YOLO debugging information
            yolo_debug_info = []
            yolo_debug_info.append(f"Frame: {frame_count}")
            yolo_debug_info.append(f"FSM: {self.state.name}")
            # yolo_debug_info.append(f"Object X: {obj_pos[0]:.2f}")
            yolo_debug_info.append(f"YOLO Zone: {'YES' if in_yolo_zone else 'NO'}")
            yolo_debug_info.append(f"Arm Processing: {'YES' if self.arm_processing_recyclable else 'NO'}")
            
            if in_yolo_zone and inf_ms > 0:
                yolo_debug_info.append(f"Inference: {inf_ms:.1f}ms")
            
            # Add detection information
            confs = self.last_results[0].boxes.conf.cpu().numpy() if len(self.last_results) > 0 and hasattr(self.last_results[0], 'boxes') else None
            if len(boxes) > 0 and confs is not None and len(confs) > 0:
                idx = np.argmax(confs)
                x1, y1, x2, y2 = boxes[idx]
                center_x = int(self.last_results[0].boxes.xywh[idx][0].item())
                center_y = int(self.last_results[0].boxes.xywh[idx][1].item())
                
                # Get class information
                class_idx = int(self.last_results[0].boxes.cls[idx].cpu().numpy())
                class_name = self.model.names[class_idx] if hasattr(self.model, 'names') and class_idx < len(self.model.names) else str(class_idx)
                confidence = confs[idx]
                
                # Convert YOLO coordinates to world coordinates
                detected_world_pos = self.camera.get_pixel_world_coords(center_x, center_y)
                
                # Add detection info
                yolo_debug_info.append(f"Detected: {class_name}")
                yolo_debug_info.append(f"Confidence: {confidence:.2f}")
                yolo_debug_info.append(f"Recycling: {'YES' if class_name in self.config.recycling_classes else 'NO'}")
                yolo_debug_info.append(f"YOLO Pos: ({detected_world_pos[0]:.2f}, {detected_world_pos[1]:.2f})")
                
                # Draw detection box and center point
                cv2.circle(output_img, (center_x, center_y), 5, (0,255,0), -1)
                cv2.rectangle(output_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                
                # Add class name on the box
                cv2.putText(output_img, f"{class_name} ({confidence:.2f})", 
                           (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            else:
                yolo_debug_info.append("Detection: NONE")
            
            # Draw all debug information on the image
            for i, info in enumerate(yolo_debug_info):
                y_pos = 30 + i * 25
                cv2.putText(output_img, info, (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
            
            cv2.imshow("YOLO Detection", output_img)
            cv2.waitKey(1)
            frame_count += 1
            if frame_count % 30 == 0:
                logger.info(f"[SIM] Frame: {frame_count}")
            p.stepSimulation()
            time.sleep(1. / self.config.simulation_fps)
            sim_time += 1.0 / self.config.simulation_fps

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
                # Get YOLO-detected center coordinates
                center_x = int(self.last_results[0].boxes.xywh[idx][0].item())
                center_y = int(self.last_results[0].boxes.xywh[idx][1].item())
                
                # Convert 2D pixel coordinates to 3D world coordinates
                detected_world_pos = self.camera.get_pixel_world_coords(center_x, center_y)
                
                # Get detected class name
                class_idx = int(self.last_results[0].boxes.cls[idx].cpu().numpy())
                class_name = self.model.names[class_idx] if hasattr(self.model, 'names') and class_idx < len(self.model.names) else str(class_idx)
                
                if class_name not in self.config.recycling_classes:
                    logger.info(f"[DETECT] Object '{class_name}' is trash - letting it continue")
                    # For trash objects, we can spawn new objects immediately since they don't get picked up
                    self.arm_processing_recyclable = False
                    return  # Do not pick up trash, let it keep moving
                
                if detected_world_pos[0] > self.config.detection_line_x:
                    # Set the target to the pos from model
                    # logger.info(f"[DETECT] Recyclable object detected at X={detected_world_pos[0]:.3f} (YOLO)")
                    # self.target_info = {"initial_pos": detected_world_pos, "detection_time": sim_time}
                    
                    
                    
                    # ------------ JYOON ---------------
                    # Get the actual object position from simulator for precise targeting
                    current_object_pos, _ = p.getBasePositionAndOrientation(self.object_id)
                    logger.info(f"[DETECT] Recyclable object detected at X={current_object_pos[0]:.3f}")
                    self.target_info = {"initial_pos": current_object_pos, "detection_time": sim_time}
                    # ------------ JYOON ---------------




                    logger.info(f"[DETECT] Target acquired - initiating pickup sequence")
                    self.state = ArmState.PREPARE_PICK
                else:
                    logger.info(f"[DETECT] Tracking object at X={detected_world_pos[0]:.2f}, waiting to cross detection line")

    def _handle_prepare_pick(self, sim_time):
        elapsed_time = sim_time - self.target_info["detection_time"]
        predicted_x = self.target_info["initial_pos"][0] + self.config.belt_velocity * elapsed_time
        time_to_pickup = (self.config.pickup_x_coord - predicted_x) / self.config.belt_velocity if self.config.belt_velocity > 0 else float('inf')

        logger.info(f"[ARM] Object {time_to_pickup:.2f}s from pickup position")

        if 0 < time_to_pickup <= self.config.arm_lead_time:
            logger.info(f"[ARM] Initiating pickup sequence")
            self.arm_processing_recyclable = True  # Set flag to prevent new object spawning
            self.state = ArmState.PICKING

    def _handle_picking(self):
        pickup_pos = [self.config.pickup_x_coord, self.target_info["initial_pos"][1], self.target_info["initial_pos"][2]]
        above_pos = [pickup_pos[0], pickup_pos[1], pickup_pos[2] + self.config.arm_above_offset]
        if self.arm_substate is None:
            logger.info(f"[ARM] Moving to pickup position: {pickup_pos}")
            logger.info(f"[ARM] Moving to above position: {above_pos}")
            move_arm_to(self.kuka_id, self.num_joints, above_pos)
            self.arm_substate = "wait_above"
        elif self.arm_substate == "wait_above":
            if wait_for_arm_to_reach(self.kuka_id, above_pos, threshold=self.config.arm_threshold):
                logger.info(f"[ARM] Moving to final pickup position: {pickup_pos}")
                move_arm_to(self.kuka_id, self.num_joints, pickup_pos)
                self.arm_substate = "wait_pick"
        elif self.arm_substate == "wait_pick":
            if wait_for_arm_to_reach(self.kuka_id, pickup_pos, threshold=self.config.arm_threshold):
                logger.info("[ARM] Grabbing object")
                self.constraint_id = grab_object(self.kuka_id, self.object_id)
                self.picked = True
                self.tracking = True
                logger.info("[ARM] Object grabbed successfully")
                self.arm_substate = None
                self.state = ArmState.LIFTING

    def _handle_lifting(self):
        current_pos = p.getLinkState(self.kuka_id, self.num_joints - 1)[0]
        lift_pos = [current_pos[0], current_pos[1], self.config.arm_lift_height]
        if self.arm_substate is None:
            logger.info("[ARM] Lifting object")
            move_arm_to(self.kuka_id, self.num_joints, lift_pos)
            self.arm_substate = "wait_lift"
        elif self.arm_substate == "wait_lift":
            if wait_for_arm_to_reach(self.kuka_id, lift_pos, threshold=self.config.arm_threshold):
                logger.info("[ARM] Moving to drop location")
                move_arm_to(self.kuka_id, self.num_joints, self.config.drop_position)
                self.arm_substate = "wait_drop"
        elif self.arm_substate == "wait_drop":
            ee_pos = p.getLinkState(self.kuka_id, self.num_joints - 1)[0]
            dist = np.linalg.norm(np.array(ee_pos) - np.array(self.config.drop_position))
            logger.info(f"[ARM DEBUG] EE pos: {ee_pos}, Drop pos: {self.config.drop_position}, Dist: {dist}")
            # Use a slightly larger threshold for drop
            if wait_for_arm_to_reach(self.kuka_id, self.config.drop_position, threshold=self.config.arm_threshold):
                logger.info("[ARM] Releasing object")
                release_object(self.constraint_id)
                self.release_time = time.time()
                # Log the detected class label for the object just picked up
                if self.last_results is not None and len(self.last_results) > 0 and hasattr(self.last_results[0], 'boxes'):
                    confs = self.last_results[0].boxes.conf.cpu().numpy()
                    if confs is not None and len(confs) > 0:
                        idx = np.argmax(confs)
                        class_idx = int(self.last_results[0].boxes.cls[idx].cpu().numpy())
                        class_name = self.model.names[class_idx] if hasattr(self.model, 'names') and class_idx < len(self.model.names) else str(class_idx)
                        logger.info(f"[ARM] Successfully processed: {class_name}")
                self.picked = False
                self.tracking = False
                self.target_info = None
                self.constraint_id = None
                self.arm_substate = None
                self.state = ArmState.RESETTING

    def _handle_resetting(self):
        base_pos = self.config.arm_base_position
        above_base = [base_pos[0], base_pos[1], base_pos[2] + 0.2]
        if self.arm_substate is None:
            logger.info("[ARM] Resetting arm and preparing for next object")
            logger.info("[ARM] Moving above base position")
            move_arm_to(self.kuka_id, self.num_joints, above_base)
            self.arm_substate = "wait_above_base"
        elif self.arm_substate == "wait_above_base":
            ee_pos = p.getLinkState(self.kuka_id, self.num_joints - 1)[0]
            dist = np.linalg.norm(np.array(ee_pos) - np.array(above_base))
            logger.info(f"[ARM DEBUG] EE pos: {ee_pos}, Above base: {above_base}, Dist: {dist}")
            if wait_for_arm_to_reach(self.kuka_id, above_base, threshold=self.config.arm_reset_threshold):
                logger.info("[ARM] Moving to base position")
                move_arm_to(self.kuka_id, self.num_joints, base_pos)
                self.arm_substate = "wait_base"
        elif self.arm_substate == "wait_base":
            ee_pos = p.getLinkState(self.kuka_id, self.num_joints - 1)[0]
            dist = np.linalg.norm(np.array(ee_pos) - np.array(base_pos))
            logger.info(f"[ARM DEBUG] EE pos: {ee_pos}, Base pos: {base_pos}, Dist: {dist}")
            if wait_for_arm_to_reach(self.kuka_id, base_pos, threshold=self.config.arm_reset_threshold):
                # Clear YOLO results to prevent old detections from staying in memory
                self.last_results = []
                self.last_img = None
                # Clear the flag since arm finished processing recyclable object
                self.arm_processing_recyclable = False
                # Load new object
                self._load_random_object()
                # Immediately update camera image for the new object
                try:
                    self.last_img = self.camera.get_image() # keep
                except Exception as e:
                    logger.warning(f"[RESET] Error updating camera image: {e}")
                    self.last_img = np.zeros((self.config.img_height, self.config.img_width, 3), dtype=np.uint8)
                self.arm_substate = None
                self.state = ArmState.WAIT_FOR_OBJECT


if __name__ == "__main__":
    config = SimConfig()
    controller = RobotController(config)
    try:
        controller.run()
    except KeyboardInterrupt:
        logger.info("[SIM] Simulation stopped by user")