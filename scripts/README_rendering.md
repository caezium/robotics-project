# Quick Guide: Rendering Simulation Videos

This script lets you render videos of your PyBullet simulation from different camera angles, including isometric and top-down views.

## Usage

From the project root, run:

```sh
python scripts/render_simulation.py --headless --frames 1000 --camera-view isometric
```

- `--frames`: Number of frames to render (default: 300)
- `--output`: Output video file (default: simulation_output.mp4)
- `--camera-view`: Choose `isometric` or `topdown` (default: isometric)
- `--headless`: Run without GUI (recommended for video)

Example for top-down:

```sh
python scripts/render_simulation.py --headless --camera-view topdown
```

## Camera Presets

- **isometric**: Diagonal, angled toward the center
- **topdown**: Overhead view

You can add more presets in the script if you want.
