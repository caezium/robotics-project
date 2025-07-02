#!/usr/bin/env python3
"""
Simulation Frame Capture Script

This script captures frames from an existing PyBullet simulation window
and saves them as a video file. Useful for recording what you see in the GUI.
"""

import cv2
import numpy as np
import time
import argparse
import os
from pathlib import Path

def capture_pybullet_window(window_title="PyBullet", num_frames=300, fps=30, output_path="captured_simulation.mp4"):
    """
    Capture frames from a PyBullet window and save as video.
    
    Args:
        window_title: Title of the PyBullet window to capture
        num_frames: Number of frames to capture
        fps: Frames per second for output video
        output_path: Path for output video file
    """
    print(f"Capturing {num_frames} frames from PyBullet window: '{window_title}'")
    print(f"Output: {output_path}")
    print(f"FPS: {fps}")
    
    # Setup video writer
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(output_path, fourcc, fps, (1920, 1080))
    
    # Get window handle
    import pyautogui
    try:
        window = pyautogui.getWindowsWithTitle(window_title)[0]
        window.activate()
        time.sleep(1)  # Give window time to activate
    except IndexError:
        print(f"Error: Could not find window with title '{window_title}'")
        print("Available windows:")
        for w in pyautogui.getAllWindows():
            if w.title:
                print(f"  - {w.title}")
        return
    
    print("Starting capture...")
    frame_count = 0
    
    try:
        while frame_count < num_frames:
            # Capture the window
            screenshot = pyautogui.screenshot(region=(window.left, window.top, window.width, window.height))
            
            # Convert to numpy array
            frame = np.array(screenshot)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            # Resize to video dimensions
            frame_resized = cv2.resize(frame, (1920, 1080))
            
            # Write frame
            video_writer.write(frame_resized)
            frame_count += 1
            
            if frame_count % 30 == 0:
                print(f"Captured frame {frame_count}/{num_frames}")
            
            # Wait for next frame
            time.sleep(1.0 / fps)
    
    except KeyboardInterrupt:
        print("\nCapture interrupted by user")
    
    finally:
        video_writer.release()
        print(f"Video saved to: {output_path}")
        print(f"Total frames captured: {frame_count}")

def main():
    parser = argparse.ArgumentParser(description="Capture frames from PyBullet simulation window")
    parser.add_argument("--frames", type=int, default=300, help="Number of frames to capture (default: 300)")
    parser.add_argument("--fps", type=int, default=30, help="Output video FPS (default: 30)")
    parser.add_argument("--output", type=str, default="captured_simulation.mp4", help="Output video file path")
    parser.add_argument("--window-title", type=str, default="PyBullet", help="PyBullet window title to capture")
    
    args = parser.parse_args()
    
    # Create output directory if it doesn't exist
    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Capture simulation
    capture_pybullet_window(
        window_title=args.window_title,
        num_frames=args.frames,
        fps=args.fps,
        output_path=args.output
    )

if __name__ == "__main__":
    main() 