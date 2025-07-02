# Simulation Rendering Scripts

This directory contains scripts for automatically rendering the robotics simulation to video files.

## Scripts Overview

### 1. `render_simulation.py` - Full Simulation Renderer

Runs the complete simulation and captures frames directly from the camera view. This is the recommended approach for creating high-quality videos.

## Usage Examples

### Basic Rendering (300 frames, 30 FPS)

```bash
python scripts/render_simulation.py
```

### Custom Frame Count and FPS

```bash
python scripts/render_simulation.py --frames 600 --fps 60
```

### Custom Output Path

```bash
python scripts/render_simulation.py --frames 450 --output videos/my_simulation.mp4
```

### Override Simulation Parameters

```bash
python scripts/render_simulation.py --frames 300 --belt-velocity 6.0 --simulation-fps 120
```

## Command Line Options

### render_simulation.py

- `--frames`: Number of frames to capture (default: 300)
- `--fps`: Output video FPS (default: 30)
- `--output`: Output video file path (default: simulation_output.mp4)
- `--belt-velocity`: Override belt velocity
- `--simulation-fps`: Override simulation FPS

### capture_simulation.py

- `--frames`: Number of frames to capture (default: 300)
- `--fps`: Output video FPS (default: 30)
- `--output`: Output video file path (default: captured_simulation.mp4)
- `--window-title`: PyBullet window title to capture (default: "PyBullet")

## Video Output

- **Format**: MP4 (H.264 codec)
- **Resolution**: 1920x1080 (Full HD)
- **Codec**: mp4v (OpenCV default)

## Tips

1. **For Best Quality**: Use `render_simulation.py` as it captures directly from the simulation camera
2. **For Real-time Recording**: Use `capture_simulation.py` when you want to record what you see
3. **Frame Rate**: Higher FPS (60) gives smoother motion but larger files
4. **Duration**: 300 frames at 30 FPS = 10 seconds of video
5. **Interruption**: Press Ctrl+C to stop rendering early

## Example Workflows

### Create a 30-second video at 60 FPS

```bash
python scripts/render_simulation.py --frames 1800 --fps 60 --output long_simulation.mp4
```

### Record with custom belt speed

```bash
python scripts/render_simulation.py --frames 600 --belt-velocity 8.0 --output fast_simulation.mp4
```

## Troubleshooting

### "Could not find window with title 'PyBullet'"

- Make sure the simulation is running
- Check the actual window title with: `python scripts/capture_simulation.py --window-title ""`
- This will list all available windows

### Video file is empty or corrupted

- Ensure you have write permissions in the output directory
- Try a different output path
- Check that the simulation is running properly

### Poor performance during rendering

- Reduce the simulation FPS: `--simulation-fps 60`
- Reduce the output FPS: `--fps 15`
- Close other applications to free up resources
