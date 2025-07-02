#!/usr/bin/env python3
"""
Simulation Renderer Script

This script runs the simulation and captures frames to create a video file.
The user can specify how many frames to capture and the output video settings.
"""

import os
import sys
import argparse
import time
import cv2
import numpy as np
from pathlib import Path

# Add the src directory to the path so we can import the main simulation
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from main import SimConfig, RobotController
import pybullet as p

def setup_video_writer(output_path, fps=30, frame_size=(1920, 1080)):
    """Setup video writer with specified settings."""
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(output_path, fourcc, fps, frame_size)
    return video_writer

def capture_simulation_frame(controller):
    """Capture a frame from the simulation."""
    # Get the current camera view
    try:
        img = controller.camera.get_image()
        if img is None:
            return None
        
        # Convert RGB to BGR for OpenCV
        img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        
        # Resize to video dimensions
        img_resized = cv2.resize(img_bgr, (1920, 1080))
        
        return img_resized
    except Exception as e:
        print(f"Error capturing frame: {e}")
        return None

def render_simulation(num_frames, output_path, fps=30, config_overrides=None):
    """
    Render the simulation for a specified number of frames.
    
    Args:
        num_frames: Number of frames to capture
        output_path: Path for the output video file
        fps: Frames per second for the output video
        config_overrides: Dictionary of config values to override
    """
    print(f"Starting simulation render: {num_frames} frames at {fps} FPS")
    print(f"Output: {output_path}")
    
    # Setup configuration
    config = SimConfig()
    if config_overrides:
        for key, value in config_overrides.items():
            if hasattr(config, key):
                setattr(config, key, value)
                print(f"Override: {key} = {value}")
    
    # Initialize controller
    controller = RobotController(config)
    
    # Setup video writer
    video_writer = setup_video_writer(output_path, fps, (1920, 1080))
    
    # Initialize simulation
    controller._load_random_object()
    controller.state = controller.state.__class__.WAIT_FOR_OBJECT
    frame_count = 0
    sim_time = 0.0
    
    # Get initial camera image
    controller.last_img = controller.camera.get_image()
    controller.last_results = []
    
    print("Starting frame capture...")
    
    try:
        while frame_count < num_frames:
            # Get object position
            obj_pos, _ = p.getBasePositionAndOrientation(controller.object_id)

            # Remove object if it is off the conveyor or disposed
            if controller.object_id is not None:
                try:
                    obj_pos, _ = p.getBasePositionAndOrientation(controller.object_id)
                except Exception:
                    controller.object_id = None
                    obj_pos = [0,0,0]
                else:
                    # Remove if: off conveyor bounds, fallen too low, or processed and past YOLO zone
                    past_yolo_zone = obj_pos[0] > controller.config.yolo_zone_exit_x and controller.object_processed
                    if obj_pos[0] > controller.config.conveyor_end_x or obj_pos[0] < controller.config.conveyor_start_x or obj_pos[2] < controller.config.min_z_position or past_yolo_zone:
                        if past_yolo_zone:
                            print(f"[REMOVE] Object processed and past YOLO zone at X={obj_pos[0]:.2f}")
                        else:
                            print(f"[REMOVE] Object removed at X={obj_pos[0]:.2f}")
                        p.removeBody(controller.object_id)
                        controller.object_id = None
                        obj_pos = [0,0,0]
                        controller.object_processed = False
            
            # Only spawn a new object if there is none AND arm is not processing a recyclable object
            if controller.object_id is None and not controller.arm_processing_recyclable:
                controller._load_random_object()
                controller.object_processed = False
                obj_pos, _ = p.getBasePositionAndOrientation(controller.object_id)
            elif controller.object_id is None and controller.arm_processing_recyclable:
                print("[SPAWN] Blocked - arm is processing recyclable object")
                obj_pos = [0, 0, 0]

            # Check if object is in YOLO detection zone
            in_yolo_zone = (controller.config.camera_center_x - controller.config.yolo_trigger_margin <= obj_pos[0] < controller.config.camera_center_x)
            
            # Track when object passes through YOLO zone
            if in_yolo_zone and not controller.object_processed:
                controller.object_processed = True
                print(f"[YOLO] Object entered detection zone at X={obj_pos[0]:.2f}")
            
            # Run YOLO inference if in zone
            if in_yolo_zone:
                try:
                    controller.last_img = controller.camera.get_image()
                    img_bgr = cv2.cvtColor(controller.last_img, cv2.COLOR_RGB2BGR)
                    controller.last_results = controller.model(img_bgr, verbose=False)
                except Exception as e:
                    print(f"[YOLO] Error during inference: {e}")
                    controller.last_img = np.zeros((controller.config.img_height, controller.config.img_width, 3), dtype=np.uint8)
                    controller.last_results = []
            
            # Update GUI and step FSM
            boxes = controller.last_results[0].boxes.xyxy.cpu().numpy() if len(controller.last_results) > 0 and hasattr(controller.last_results[0], 'boxes') else []
            controller.gui.update(controller, fsm_state=controller.state.name, sim_time=sim_time, target_info=controller.target_info, boxes=boxes)
            
            # Apply conveyor velocity
            contacts = p.getContactPoints(bodyA=controller.object_id, bodyB=controller.belt_id)
            if contacts and not controller.picked:
                p.resetBaseVelocity(controller.object_id, linearVelocity=[controller.config.belt_velocity, 0, 0])
            
            # Step FSM
            controller._step_fsm(sim_time)
            
            # Capture frame for video
            frame = capture_simulation_frame(controller)
            if frame is not None:
                video_writer.write(frame)
                frame_count += 1
                
                if frame_count % 30 == 0:
                    print(f"Captured frame {frame_count}/{num_frames}")
            
            # Step simulation
            p.stepSimulation()
            time.sleep(1. / controller.config.simulation_fps)
            sim_time += 1.0 / controller.config.simulation_fps
    
    except KeyboardInterrupt:
        print("\nRendering interrupted by user")
    
    finally:
        # Cleanup
        video_writer.release()
        p.disconnect()
        print(f"Video saved to: {output_path}")
        print(f"Total frames captured: {frame_count}")

def main():
    parser = argparse.ArgumentParser(description="Render simulation to video")
    parser.add_argument("--frames", type=int, default=300, help="Number of frames to capture (default: 300)")
    parser.add_argument("--fps", type=int, default=30, help="Output video FPS (default: 30)")
    parser.add_argument("--output", type=str, default="simulation_output.mp4", help="Output video file path")
    parser.add_argument("--belt-velocity", type=float, help="Override belt velocity")
    parser.add_argument("--simulation-fps", type=float, help="Override simulation FPS")
    
    args = parser.parse_args()
    
    # Create output directory if it doesn't exist
    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Setup config overrides
    config_overrides = {}
    if args.belt_velocity:
        config_overrides['belt_velocity'] = args.belt_velocity
    if args.simulation_fps:
        config_overrides['simulation_fps'] = args.simulation_fps
    
    # Render simulation
    render_simulation(
        num_frames=args.frames,
        output_path=args.output,
        fps=args.fps,
        config_overrides=config_overrides
    )

if __name__ == "__main__":
    main() 